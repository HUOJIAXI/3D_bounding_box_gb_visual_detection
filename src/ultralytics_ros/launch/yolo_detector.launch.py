#!/usr/bin/env python3
"""
Launch file for Ultralytics YOLO detector
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for YOLO detector
    """

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='yolov8n.yaml',
        description='Config file name (e.g., yolov8n.yaml, yolov11m.yaml)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Device to run inference on (cuda:0, cpu)'
    )

    # Get package share directory
    package_share_dir = FindPackageShare('ultralytics_ros')

    # Config file path
    config_path = PathJoinSubstitution([
        package_share_dir,
        'config',
        LaunchConfiguration('config_file')
    ])

    # YOLO detector node
    yolo_detector_node = Node(
        package='ultralytics_ros',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            config_path,
            {
                'input_topic': LaunchConfiguration('image_topic'),
                'device': LaunchConfiguration('device')
            }
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        image_topic_arg,
        device_arg,
        yolo_detector_node
    ])
