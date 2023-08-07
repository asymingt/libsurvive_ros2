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

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Bas directories for resource loading and writing.
LOG_DIR = launch.logging.launch_config.log_dir
CFG_DIR = os.path.join(get_package_share_directory("libsurvive_ros2"), "config")

# Bag to save data
BAG_FILE = os.path.join(LOG_DIR, "libsurvive.bag")
DRIVER_CONFIG_IN = os.path.join(LOG_DIR, "dummy_driver.json")
DRIVER_CONFIG_OUT = os.path.join(LOG_DIR, "config_driver.json")


def generate_launch_description():
    arguments = [
        # General options
        DeclareLaunchArgument(
            "namespace",
            default_value="libsurvive",
            description="Namespace for the non-TF topics",
        ),
        DeclareLaunchArgument(
            "tracking_frame",
            default_value="libsurvive_world",
            description="Tracking parent frame name",
        ),
        DeclareLaunchArgument(
            "web_bridge",
            default_value="true",
            description="Launch a foxglove web bridge",
        ),
        DeclareLaunchArgument(
            "composable", default_value="false", description="Record data with rosbag"
        ),
        DeclareLaunchArgument(
            "record", default_value=BAG_FILE, description="Record data to this bag path"
        ),
        DeclareLaunchArgument(
            "replay", default_value="", description="Replay data from this bag path"
        ),    
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Launch in a debugger xterm window",
        ),
        # Driver configuration
        DeclareLaunchArgument(
            "driver_config_in",
            default_value=DRIVER_CONFIG_IN,
            description="Input configuration file",
        ),
        DeclareLaunchArgument(
            "driver_config_out",
            default_value=DRIVER_CONFIG_OUT,
            description="Output configuration file",
        ),
        DeclareLaunchArgument(
            "recalibrate",
            default_value="false",
            description="Recalibrate lighthouse positions",
        ),
        # Meta configuration
        DeclareLaunchArgument(
            "meta_config",
            default_value="",
            description="Meta estimation configuration file",
        ),
    ]

    # If we have specified a valid record location, then enable recording
    enable_record = PythonExpression(
        ["bool('", LaunchConfiguration("record"), "')"]
    )

    # If we are replaying, then we should not run the low-level driver
    enable_driver = PythonExpression(
        ["not bool('", LaunchConfiguration("replay"), "')"]
    )

    # If have a meta poser config file, then we need to enable the poser
    enable_poser = PythonExpression(
        ["bool('", LaunchConfiguration("meta_config"), "')"]
    )

    # If have a meta poser config file specified, disable the low-level poser.
    driver_args = PythonExpression(
        ["'-p Null' if bool('", LaunchConfiguration("meta_config"), "') else '-p MPFIT'"]
    )

    # Sow we don't have to repeat for composable and non-composable versions.
    driver_parameters = [
        {"driver_args": driver_args},
        {"driver_config_in": LaunchConfiguration("driver_config_in")},
        {"driver_config_out": LaunchConfiguration("driver_config_out")},
        {"recalibrate": LaunchConfiguration("recalibrate")},
        {"tracking_frame": LaunchConfiguration("tracking_frame")},
    ]
    poser_parameters = [
        {"meta_config": LaunchConfiguration("meta_config")},
        {"tracking_frame": LaunchConfiguration("tracking_frame")},
    ]

    # If have a meta poser config file specified, disable the low-level poser.
    launch_prefix = PythonExpression(
        ["'xterm -e gdb -ex=r --args' if '", LaunchConfiguration("debug"), "'=='true' else ''"]
    )


    # Composed pipeline.
    composed_pipeline = GroupAction(
        actions=[
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container",
                name="libsurvive_ros2_container",
                namespace=LaunchConfiguration("namespace"),
                prefix=launch_prefix,
                output="screen",
            ),
            LoadComposableNodes(
                target_container="libsurvive_ros2_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="libsurvive_ros2",
                        plugin="libsurvive_ros2::DriverComponent",
                        name="libsurvive_ros2_driver_component",
                        namespace=LaunchConfiguration("namespace"),
                        parameters=driver_parameters,
                        extra_arguments=[{"use_intra_process_comms": False}],
                    ),
                ],
                condition=IfCondition(enable_driver),
            ),
            LoadComposableNodes(
                target_container="libsurvive_ros2_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="libsurvive_ros2",
                        plugin="libsurvive_ros2::PoserComponent",
                        name="libsurvive_ros2_poser_component",
                        namespace=LaunchConfiguration("namespace"),
                        parameters=poser_parameters,
                        extra_arguments=[{"use_intra_process_comms": False}],
                    ),
                ],
                condition=IfCondition(enable_poser),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("composable")),
    )

    # Non-composed pipeline
    non_composed_pipeline = GroupAction(
        actions=[
            Node(
                package="libsurvive_ros2",
                executable="libsurvive_ros2_driver_node",
                name="libsurvive_ros2_driver_node",
                namespace=LaunchConfiguration("namespace"),
                condition=IfCondition(enable_driver),
                prefix=launch_prefix,
                output="screen",
                parameters=driver_parameters,
            ),
            Node(
                package="libsurvive_ros2",
                executable="libsurvive_ros2_poser_node",
                name="libsurvive_ros2_poser_node",
                namespace=LaunchConfiguration("namespace"),
                condition=IfCondition(enable_poser),
                prefix=launch_prefix,
                output="screen",
                parameters=poser_parameters,
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration("composable")),
    )

    # For bridging connection to foxglove running on a remote server.
    web_bridge_node = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {"address": "0.0.0.0"},
            {"port": 8765},
            {"send_buffer_limit": 100000000},
            {"min_qos_depth": 100},
        ],
        condition=IfCondition(LaunchConfiguration("web_bridge")),
        output="log",
    )

    # For recording all data from the experiment. We'll use the MCAP format by
    # default in order to keep consistency across ROS2 releases.
    bag_recorder_node = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", "-o", LaunchConfiguration("record"), "-a"],
        condition=IfCondition(enable_record),
        output="log",
    )

    # In replay we're going to take the raw data and recalculate the poses and TF2
    # transforms. To avoid interference with the old estimates, we're going to map
    # the replayed values to the /old prefix.
    bag_player_node = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-s", "mcap", LaunchConfiguration("replay"), "--remap",
                "/roslog:=/old/roslog",
                "/tf:=/old/tf",
                "/tf_static:=/old/tf_static",
                "/libsurvive/pose/body:=/old/pose/body",
                "/libsurvive/pose/tracker:=/old/pose/tracker",
                "/libsurvive/pose/lighthouse:=/old/pose/lighthouse",
            ],
        condition=UnlessCondition(enable_driver),
        output="log",
    )

    return LaunchDescription(
        arguments + [
            composed_pipeline,
            non_composed_pipeline,
            web_bridge_node,
            bag_recorder_node,
            bag_player_node
        ]
    )
