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
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node

BAG_FILE = os.path.join(launch.logging.launch_config.log_dir, "libsurvive.bag")
CFG_FILE = os.path.join(
    get_package_share_directory('libsurvive_ros2'), 'config', 'home.json'
)

def generate_launch_description():
    return LaunchDescription([
        # 
        Node(
            package='libsurvive_ros2',
            executable='libsurvive_node',
            name='vive_node',
            output='screen',
            arguments=[
                '--force-calibrate', '1',
                '--globalscenesolver', '1',
                '--lighthouse-gen', '2',
                '--lighthousecount', '2'
            ]
        ),
        # For bridging connection to foxglove running on a remote server.
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server_node',
            parameters=[
                {"port": 54321},
            ],
            output='log'
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='log'
        ),
        # For recording all data from the experiment
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', BAG_FILE] + [
                '/tf',
                '/tf_static'
            ],
            output='screen'
        )
    ])