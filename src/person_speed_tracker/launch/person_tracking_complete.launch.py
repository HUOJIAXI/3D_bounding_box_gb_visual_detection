#!/usr/bin/env python3
"""
Complete launch file for person tracking system
Includes: YOLO detector, 3D bounding box generator, and person speed tracker
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    use_yolov8_arg = DeclareLaunchArgument(
        'use_yolov8',
        default_value='true',
        description='Use YOLO V8 (Ultralytics) for 2D detection. Default: true'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8m',
        description='YOLO model variant: yolov8n, yolov8s, yolov8m, yolov8l, yolov8x, yolov11m. Default: yolov8m'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time. Default: true'
    )

    tracker_config_arg = DeclareLaunchArgument(
        'tracker_config',
        default_value='',
        description='Path to person speed tracker config file (optional)'
    )

    # Get package directories
    try:
        ultralytics_ros_dir = get_package_share_directory('ultralytics_ros')
    except:
        ultralytics_ros_dir = None

    darknet_ros_3d_dir = get_package_share_directory('darknet_ros_3d')
    person_speed_tracker_dir = get_package_share_directory('person_speed_tracker')

    # Configuration files
    darknet_3d_config = os.path.join(darknet_ros_3d_dir, 'config', 'darknet_3d.yaml')

    # Use custom tracker config if provided, otherwise use default
    tracker_config = LaunchConfiguration('tracker_config')
    default_tracker_config = os.path.join(person_speed_tracker_dir, 'config', 'tracker_params.yaml')

    # Environment variable for line buffering
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    use_yolov8 = LaunchConfiguration('use_yolov8')
    yolo_model = LaunchConfiguration('yolo_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node list
    nodes_to_launch = []

    # 1. Launch YOLO detector node (YOLOv8 via ultralytics_ros)
    if ultralytics_ros_dir:
        yolo_config_path = PathJoinSubstitution([
            ultralytics_ros_dir,
            'config',
            [yolo_model, '.yaml']
        ])

        yolo_detector_node = Node(
            package='ultralytics_ros',
            executable='yolo_detector_node.py',
            name='yolo_detector_node',
            output='screen',
            parameters=[
                yolo_config_path,
                {'use_sim_time': use_sim_time}
            ],
            condition=conditions.IfCondition(use_yolov8),
            emulate_tty=True
        )
        nodes_to_launch.append(yolo_detector_node)

    # 2. Launch darknet_ros_3d for 3D bounding box estimation
    darknet_ros_3d_node = Node(
        package='darknet_ros_3d',
        executable='darknet3d_node',
        name='darknet3d_node',
        output='screen',
        parameters=[
            darknet_3d_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    nodes_to_launch.append(darknet_ros_3d_node)

    # 3. Launch person speed tracker
    person_speed_tracker_node = Node(
        package='person_speed_tracker',
        executable='person_speed_tracker_node',
        name='person_speed_tracker',
        output='screen',
        parameters=[
            default_tracker_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    nodes_to_launch.append(person_speed_tracker_node)

    # Build launch description
    ld = LaunchDescription()

    # Add environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add launch arguments
    ld.add_action(use_yolov8_arg)
    ld.add_action(yolo_model_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(tracker_config_arg)

    # Add all nodes
    for node in nodes_to_launch:
        ld.add_action(node)

    return ld
