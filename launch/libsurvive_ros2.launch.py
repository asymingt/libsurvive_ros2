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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Bag to save data
BAG_FILE = os.path.join(launch.logging.launch_config.log_dir, 'libsurvive.bag')

# Default libsurvive configuration file
CFG_FILE = os.path.join(
    get_package_share_directory('libsurvive_ros2'), 'config', 'config.json'
)

# Sow we don't have to repeat for composable and non-composable versions.
PARAMETERS = [
    {'driver_args': f'--force-recalibrate 1 -c {CFG_FILE}'},
    {'tracking_frame': 'libsurvive_world'},
    {'imu_topic': 'imu'},
    {'joy_topic': 'joy'},
    {'cfg_topic': 'cfg'},
    {'lighthouse_rate': 4.0}]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('namespace', default_value='libsurvive',
                              description='Namespace for the non-TF topics'),
        DeclareLaunchArgument('composable', default_value='false',
                              description='Launch in a composable container'),
        DeclareLaunchArgument('rosbridge', default_value='false',
                              description='Launch a rosbridge'),
        DeclareLaunchArgument('foxbridge', default_value='false',
                              description='Launch a foxglove bridge'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record data with rosbag')]

    # Non-composable launch (regular node)
    libsurvive_node = Node(
        package='libsurvive_ros2',
        executable='libsurvive_ros2_node',
        name='libsurvive_ros2_node',
        namespace=LaunchConfiguration('namespace'),
        condition=UnlessCondition(LaunchConfiguration('composable')),
        output='screen',
        parameters=PARAMETERS)

    # Composable launch (zero-copy node example)
    libsurvive_composable_node = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='libsurvive_ros2_container',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('composable')),
        composable_node_descriptions=[
            ComposableNode(
                package='libsurvive_ros2',
                plugin='libsurvive_ros2::Component',
                name='libsurvive_ros2_component',
                parameters=PARAMETERS,
                extra_arguments=[
                    {'use_intra_process_comms': True}
                ]
            )
        ],
        output='log')

    # For ros webdocker bridge
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_server_node',
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        parameters=[
            {'port': 9090},
        ],
        output='log')
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        output='log')

    # For foxglove websocket bridge.
    foxbridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('foxbridge')),
        parameters=[
            {'port': 8765},
        ],
        output='log')

    # For recording all data from the experiment
    bag_record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', BAG_FILE] + [
            '/tf',
            '/tf_static'
        ],
        condition=IfCondition(LaunchConfiguration('record')),
        output='log')

    return LaunchDescription(
        arguments + [
            libsurvive_node,
            libsurvive_composable_node,
            foxbridge_node,
            rosbridge_node,
            rosapi_node,
            bag_record_node
        ])
