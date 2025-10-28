#!/usr/bin/env python3
# Copyright 2020 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Get package directories
    darknet_ros_dir = get_package_share_directory('darknet_ros')
    darknet_ros_3d_dir = get_package_share_directory('darknet_ros_3d')

    # Configuration files
    darknet_3d_config = os.path.join(darknet_ros_3d_dir, 'config', 'darknet_3d.yaml')
    darknet_ros_launch = os.path.join(darknet_ros_dir, 'launch', 'darknet_ros.launch.py')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Launch darknet_ros for 2D detection
    darknet_ros_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(darknet_ros_launch)
    )

    # Launch darknet_ros_3d for 3D bounding boxes
    darknet_ros_3d_node = Node(
        package='darknet_ros_3d',
        executable='darknet3d_node',
        name='darknet3d_node',
        output='screen',
        parameters=[darknet_3d_config]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(darknet_ros_launch_cmd)
    ld.add_action(darknet_ros_3d_node)

    return ld
