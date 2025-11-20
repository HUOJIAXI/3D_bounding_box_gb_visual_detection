#!/usr/bin/env python3
"""
Simplified complete launch file for person tracking system
Launches all three components: YOLO detector -> 3D bbox -> Person tracker
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8m',
        description='YOLO model to use (yolov8n, yolov8s, yolov8m, yolov8l, yolov8x, yolov11m)'
    )

    # Get package directories
    darknet_ros_3d_dir = get_package_share_directory('darknet_ros_3d')
    person_speed_tracker_dir = get_package_share_directory('person_speed_tracker')

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    yolo_model = LaunchConfiguration('yolo_model')

    # Environment variable
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # 1. Include darknet_ros_3d complete launch (includes YOLO + 3D bbox)
    darknet_3d_complete_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(darknet_ros_3d_dir, 'launch', 'darknet_ros_3d_complete.launch.py')
        ),
        launch_arguments={
            'use_yolov8': 'true',
            'yolo_model': yolo_model
        }.items()
    )

    # 2. Launch person speed tracker
    person_tracker_config = os.path.join(
        person_speed_tracker_dir, 'config', 'tracker_params.yaml'
    )

    person_speed_tracker_node = Node(
        package='person_speed_tracker',
        executable='person_speed_tracker_node',
        name='person_speed_tracker',
        output='screen',
        parameters=[
            person_tracker_config,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(use_sim_time_arg)
    ld.add_action(yolo_model_arg)
    ld.add_action(darknet_3d_complete_launch)
    ld.add_action(person_speed_tracker_node)

    return ld
