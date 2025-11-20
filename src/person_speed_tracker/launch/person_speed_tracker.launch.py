#!/usr/bin/env python3
"""
Launch file for person speed tracker node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('person_speed_tracker')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'tracker_params.yaml'),
        description='Path to config file for person speed tracker'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Create node
    person_speed_tracker_node = Node(
        package='person_speed_tracker',
        executable='person_speed_tracker_node',
        name='person_speed_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        person_speed_tracker_node
    ])
