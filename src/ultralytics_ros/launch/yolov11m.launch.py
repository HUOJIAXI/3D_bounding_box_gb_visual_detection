#!/usr/bin/env python3
"""
Launch file for YOLOv11 Medium detector (latest generation)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for YOLOv11m
    """

    package_share_dir = FindPackageShare('ultralytics_ros')

    yolo_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                package_share_dir,
                'launch',
                'yolo_detector.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': 'yolov11m.yaml'
        }.items()
    )

    return LaunchDescription([
        yolo_detector_launch
    ])
