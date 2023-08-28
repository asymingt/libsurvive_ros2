# Copyright 2022 Andrew Symington
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# DESCRIPTION
#
# Registration works by hand-labeling the positions of marker points in some
# global frame. Our goal is to find the transforms that move the lighthouses
# from their internal coordinate frame to some global frame, such that their
# final positions coincide with labeled registration markers.
#
# Let's assume some shorthand notation
#
#   - g: global frame         # This is the global frame for libsurvive
#   - l: lighthouse frame     # This is a frame centered on a lighthouse
#   - c: calibration frame    # This is the config file global frame
#   - m: marker frame         # This is the frame of the registration marker
#
# What we are trying to solve for is a transform (gTl) for each lighthouse
# that moves it from some local frame (l) to the global frame (g). The hand
# labeled registration points are translation elements of gTm, and so can
# be considered PoseTranslationPrior3D factors. To use these labels we need
# to first correct for the marker offset from the  lighthouse to resolve
# gTl = gTm * lTm.inverse(). Then they can be treated as direct observations
# of the translation component of the quantity we are estimating.
#
# Each round of libsurvive calibration results int a config file that is
# locally consistent within an arbitrary global frame -- this is either from
# a tracker's initial orientation, or the first lighthouse that was seen.
# Regardless, what we care about is applying a constraint to our system that
# penalizes pairs of relative lighthouse poses that deviate from the values
# in the calibration files. So, for every pair of lighthouses in each file
# we apply a BetweenFactorPose3 that enforces internal consistency.
#
# The initial values wwe choose for our values (gTl) are copied directly
# from the last value specified in the configuration file. This ensures that
# clusters of lighthouses are "roughly correct", which helps steer the
# optimizer in the correct direction and avoid converging to local minima.

# System includes
import json
import numpy as np
import yaml

from scipy.sparse.csgraph import connected_components

import gtsam
from gtsam.symbol_shorthand import X

# Maximum numver of interfaces
MAX_NUM_LIGHTHOUSES = 16


# The same lighthouse had a different channel in two config files
class ChannelInconsistencyError(Exception):
    pass


# The graph formed by configurations is disconnected, and cannot be registered.
class ChannelOverlapError(Exception):
    pass


# An input configuration file could not be read.
class InputConfigReadError(Exception):
    pass


# The output configuration could not be read
class OutputConfigWriteError(Exception):
    pass


# The output configuration could not be read
class RegistrationReadError(Exception):
    pass


def _load_config_file(path):
    """Load the content from a configration JSON file."""
    try:
        with open(path, "r") as file:
            content = file.read()
            content = content.replace('"lighthouse', ', "lighthouse')
            content = "{" + content + "}"
            return json.loads(content)
    except Exception:
        print("Error: input file is not readable")
        raise InputConfigReadError("input file is not readable")


def _write_config_file(path, config):
    """Write content to a configuration JSON file."""
    try:
        with open(path, "w") as file:
            file.write('"v":"0",\n')
            file.write('"poser":"MPFIT",\n')
            file.write('"disambiguator":"StateBased",\n')
            file.write('"configed-lighthouse-gen":"2",\n')
            file.write('"ootx-ignore-sync-error":"0",\n')
            file.write('"floor-offset":"0"\n')
            for lighthouse_name, lighthouse_data in config.items():
                serialized_content = json.dumps(lighthouse_data, indent=2)
                file.write(f'"{lighthouse_name}": {serialized_content}\n')
    except Exception:
        print("Error: output file is not writable")
        raise OutputConfigWriteError("output file is not writable")


def _read_registration_config(path):
    """Read registration YAML file."""
    try:
        with open(path, "r") as file:
            return yaml.safe_load(file)
    except Exception:
        print("Error: registration file is not readable")
        raise RegistrationReadError("registration file is not readable")


def config_merge(inputs, output, reg_config=None):
    """Merge one or more libsurvive input configuration files."""
    graph = gtsam.NonlinearFactorGraph()
    value = gtsam.Values()

    # Helps ensure that we don't
    print("Loading input configuration files")
    adjacency = np.eye(MAX_NUM_LIGHTHOUSES, dtype=np.float64)
    lighthouses = {}
    for path in inputs:
        input_config = _load_config_file(path=path)

        # Now process all lighthouses within this config file
        edges = set()
        cTl_value = {}
        cTl_error = {}
        for lighthouse_name, lighthouse_data in input_config.items():
            if lighthouse_name.startswith("lighthouse"):
                lh_serial = lighthouse_data["id"]
                lh_mode = lighthouse_data["mode"]

                # Insert the lighthouse.
                if lh_serial in lighthouses.keys():
                    if lighthouses[lh_serial]["mode"] != lh_mode:
                        raise ChannelInconsistencyError(
                            "Inconsistent ID <-> channel mapping"
                        )
                    i = lighthouses[lh_serial]["index"]
                else:
                    i = len(lighthouses)
                    lighthouses[lh_serial] = lighthouse_data
                    lighthouses[lh_serial]["index"] = i

                    # Add a value representing the ith lighthouse pose in the global frame.
                    # This value will be updated continually. This is just to ensure that one
                    # already exists, so that updating it doesn't require an existence check.
                    gTl_value = gtsam.Pose3(
                        gtsam.Rot3.Quaternion(1.0, 0.0, 0.0, 0.0),
                        gtsam.Point3(0.0, 0.0, 0.0),
                    )
                    value.insert(X(i), gTl_value)

                # Add the lighthouse -> config frame transform for each lighthouse i.
                cTl_value[i] = gtsam.Pose3(
                    gtsam.Rot3.Quaternion(
                        float(lighthouse_data["pose"][3]),  # qw
                        float(lighthouse_data["pose"][4]),  # qx
                        float(lighthouse_data["pose"][5]),  # qy
                        float(lighthouse_data["pose"][6]),
                    ),  # qz
                    gtsam.Point3(
                        float(lighthouse_data["pose"][0]),  # x
                        float(lighthouse_data["pose"][1]),  # y
                        float(lighthouse_data["pose"][2]),
                    ),
                )  # z
                cTl_error[i] = np.array(
                    [
                        float(lighthouse_data["variance"][3]),  # rx
                        float(lighthouse_data["variance"][4]),  # ry
                        float(lighthouse_data["variance"][5]),  # rz
                        float(lighthouse_data["variance"][0]),  # tx
                        float(lighthouse_data["variance"][1]),  # ty
                        float(lighthouse_data["variance"][2]),  # tz
                    ],
                    dtype=float,
                )

                # Update our best estimate -- yes, this will overwrite any previous value
                # but that's ok. It just needs to bea reasonably correct.
                value.update(X(i), cTl_value[i])

                # Add to the solvability determinator
                edges.add(i)

        # Complete the solvability determinator
        print("-> ", path, edges)
        for i in edges:
            for j in edges:
                adjacency[i, j] = 1.0
                if i < j:
                    jTi_value = cTl_value[i].between(cTl_value[j])
                    c_sigma = cTl_error[i] + cTl_error[j]
                    lAc = cTl_value[i].inverse().AdjointMap()
                    l_sigma = gtsam.noiseModel.Gaussian.Covariance(
                        lAc * c_sigma * lAc.T
                    )
                    graph.add(gtsam.BetweenFactorPose3(X(i), X(j), jTi_value, l_sigma))

    # Make sure the channel graph is fully connected. If it's not, then we won't be
    # able to register the entire system in a common frame, so there is little point
    # adding it to GTSAM and event trying to find a solutidon.
    num_lighthouses = len(lighthouses)
    adjacency = adjacency[0:num_lighthouses, 0:num_lighthouses]
    n_components, _ = connected_components(adjacency)
    print("Adjacency matrix", adjacency)
    print("Connected components", n_components)
    if n_components != 1:
        print("Error: too many connected components in channel graph")
        raise ChannelOverlapError("channel graph is disconnected")

    # If registration information is supplied.
    if reg_config is not None:

        print("Reading registration from", reg_config)
        registration = _read_registration_config(path=reg_config)

        print("Grabbing lTm estimates")
        lTm = gtsam.Pose3(
            gtsam.Rot3.Quaternion(1.0, 0.0, 0.0, 0.0),  # [qw, qx, qy, qz]
            gtsam.Point3(
                registration["lighthouse_from_marker"]["position"][0],  # x
                registration["lighthouse_from_marker"]["position"][1],  # y
                registration["lighthouse_from_marker"]["position"][2],
            ),
        )  # z

        print("Grabbing gTm estimates")
        assert len(registration["markers"]) > 3, "you need four or more markers"
        for lh_name, marker_data in registration["markers"].items():
            lh_serial = str(marker_data["id"])
            if lh_serial not in lighthouses.keys():
                print(
                    "WARNING: marker for non-existent lighthouse in config files. Ignoring"
                )
                continue
            i = int(lighthouses[lh_serial]["index"])

            # The registration information expresses the position of the marker in the global
            # frame. But what we want is the position of the lighthouse in the global frame.
            gTm_value = gtsam.Pose3(
                gtsam.Rot3.Quaternion(1.0, 0.0, 0.0, 0.0),  # qw  # qx  # qy  # qz
                gtsam.Point3(
                    marker_data["position"][0],  # x
                    marker_data["position"][1],  # y
                    marker_data["position"][2],
                ),  # z
            )
            gTl_value = gTm_value * lTm.inverse()

            # The noise vector expresses how much uncertainty we expect in our ability to
            # measure the marker position in the global frame. Since the parent frame is not
            # changing the
            gTm_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array(
                    [
                        marker_data["sigma"][0],
                        marker_data["sigma"][1],
                        marker_data["sigma"][2],
                    ],
                    dtype=float,
                )
            )
            gTl_noise = gTm_noise

            # Add a factor constraining the position.
            graph.add(gtsam.PoseTranslationPrior3D(X(i), gTl_value, gTl_noise))

    else:
        print("No registration information. Assuming lighthouse 1 is the global frame")
        gTl_value = gtsam.Pose3(
            gtsam.Rot3.Quaternion(1.0, 0.0, 0.0, 0.0),  # qw  # qx  # qy  # qz
            gtsam.Point3(0.0, 0.0, 0.0),  # x  # y
        )
        gTl_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4])
        )
        graph.add(gtsam.PriorFactorPose3(X(0), gTl_value, gTl_noise))

    print("Running optimizer")
    graph.print("full graph")
    value.print("initial values")
    result = gtsam.LevenbergMarquardtOptimizer(graph, value).optimize()
    result.print("final result")
    marginals = gtsam.Marginals(graph, result)

    # Helper function to serialize floating points consistently to strings
    def str_from_float(f):
        return np.format_float_positional(f, precision=9, trim="k", unique=False)

    # Write the updated global configuration.
    print("Writing output config to", output)
    config = {}
    for lh_serial, lh_data in lighthouses.items():
        name = "lighthouse{index}".format(index=lh_data["index"])
        i = int(lh_data["index"])
        cTl_est = result.atPose3(X(i))
        cTl_cov = marginals.marginalCovariance(X(i))
        config[name] = lh_data
        config[name]["pose"] = [
            str_from_float(cTl_est.translation()[0]),  # x
            str_from_float(cTl_est.translation()[1]),  # y
            str_from_float(cTl_est.translation()[2]),  # z
            str_from_float(cTl_est.rotation().toQuaternion().w()),  # qw
            str_from_float(cTl_est.rotation().toQuaternion().x()),  # qx
            str_from_float(cTl_est.rotation().toQuaternion().y()),  # qy
            str_from_float(cTl_est.rotation().toQuaternion().z()),
        ]  # qz
        config[name]["variance"] = [
            str_from_float(cTl_cov[3, 3]),  # tx
            str_from_float(cTl_cov[4, 4]),  # ty
            str_from_float(cTl_cov[5, 5]),  # tz
            str_from_float(cTl_cov[0, 0]),  # rx
            str_from_float(cTl_cov[1, 1]),  # ry
            str_from_float(cTl_cov[2, 2]),
        ]  # rz
    _write_config_file(path=output, config=config)

    # Return the full configuration.
    return config


def config_equal(path1, path2):
    """Load two configs and verify that they are the same."""
    config1 = _load_config_file(path=path1)
    config2 = _load_config_file(path=path2)
    return config1 == config2
