# Copyright 2023 Andrew Symington
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


# System includes
import os
import pytest

# Python API for config merger
from libsurvive_ros2_py.config_tools import (
    ChannelOverlapError,
    InputConfigReadError,
    OutputConfigWriteError,
    config_merge,
    config_equal,
)

# Path of all test fixtures
TEST_FOLDER_PATH = os.path.dirname(__file__)


def test_config_merge_without_registration():
    """Shows a good set of configs merge correctly without registration."""
    inputs = [
        os.path.join(TEST_FOLDER_PATH, "input_config_0_1_2.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_1_2_3.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_2_3_4.json"),
    ]
    output = "merged_without_registration.json"
    config = config_merge(inputs=inputs, output=output)
    for idx in range(0, 5):
        assert f"lighthouse{idx}" in config.keys(), f"missing lighthouse {idx}"
    output_truth = os.path.join(TEST_FOLDER_PATH, "output_without_registration.json")
    assert config_equal(output, output_truth), "configuration mismatch"


def test_config_merge_with_registration():
    """Shows a good set of configs merge correctly with registration."""
    inputs = [
        os.path.join(TEST_FOLDER_PATH, "input_config_0_1_2.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_1_2_3.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_2_3_4.json"),
    ]
    output = "merged_with_registration.json"
    reg_config = os.path.join(TEST_FOLDER_PATH, "example_registration.yaml")
    config = config_merge(inputs=inputs, output=output, reg_config=reg_config)
    for idx in range(0, 5):
        assert f"lighthouse{idx}" in config.keys(), f"missing lighthouse {idx}"
    output_truth = os.path.join(TEST_FOLDER_PATH, "output_with_registration.json")
    assert config_equal(output, output_truth), "configuration mismatch"


def test_malformed_input_config():
    """Add a malformed config file and show that it causes the right error."""
    inputs = [
        os.path.join(TEST_FOLDER_PATH, "input_config_0_1_2.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_1_2_3.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_2_3_4.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_malformed.json"),
    ]
    output = "merged_with_registration.json"
    reg_config = os.path.join(TEST_FOLDER_PATH, "example_registration.yaml")
    with pytest.raises(InputConfigReadError, match="input file is not readable"):
        config_merge(inputs=inputs, output=output, reg_config=reg_config)


def test_non_overlapping_channel_error():
    """Add a non-overlapping config and show that it causes the right error."""
    inputs = [
        os.path.join(TEST_FOLDER_PATH, "input_config_0_1_2.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_1_2_3.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_2_3_4.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_7_8_9.json"),
    ]
    output = "merged_with_registration.json"
    reg_config = os.path.join(TEST_FOLDER_PATH, "example_registration.yaml")
    with pytest.raises(ChannelOverlapError, match="channel graph is disconnected"):
        config_merge(inputs=inputs, output=output, reg_config=reg_config)


def test_unreadable_output_file():
    """Shows that a non-writable output configuration location causes the right error."""
    inputs = [
        os.path.join(TEST_FOLDER_PATH, "input_config_0_1_2.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_1_2_3.json"),
        os.path.join(TEST_FOLDER_PATH, "input_config_2_3_4.json"),
    ]
    output = "/root/merged_config.json"
    reg_config = os.path.join(TEST_FOLDER_PATH, "example_registration.yaml")
    with pytest.raises(OutputConfigWriteError, match="output file is not writable"):
        config_merge(inputs=inputs, output=output, reg_config=reg_config)
