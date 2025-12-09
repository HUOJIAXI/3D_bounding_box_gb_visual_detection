#!/usr/bin/env python3
"""
Complete launch file for person tracking and clustering system
Includes: YOLO detector, 3D bounding box generator, person tracker with clustering
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
import os


def generate_launch_nodes(context):
    """Generate nodes with resolved launch arguments"""
    # Get resolved launch argument values
    gpu_id_value = context.launch_configurations.get('gpu_id', '0')
    use_yolov8_value = context.launch_configurations.get('use_yolov8', 'true')
    yolo_model_value = context.launch_configurations.get('yolo_model', 'yolov8m')
    use_sim_time_value = context.launch_configurations.get('use_sim_time', 'true')
    
    # YOLO parameters
    yolo_confidence_threshold = float(context.launch_configurations.get('yolo_confidence_threshold', '0.25'))
    yolo_iou_threshold = float(context.launch_configurations.get('yolo_iou_threshold', '0.45'))
    yolo_input_topic = context.launch_configurations.get('yolo_input_topic', '/task_generator_node/tiago_base/rgbd_camera/image')
    yolo_max_det = int(context.launch_configurations.get('yolo_max_det', '300'))
    yolo_enable_visualization = context.launch_configurations.get('yolo_enable_visualization', 'true').lower() == 'true'
    yolo_publish_image = context.launch_configurations.get('yolo_publish_image', 'true').lower() == 'true'
    yolo_image_size = int(context.launch_configurations.get('yolo_image_size', '640'))
    
    # Get package directories
    try:
        ultralytics_ros_dir = get_package_share_directory('ultralytics_ros')
    except:
        ultralytics_ros_dir = None

    darknet_ros_3d_dir = get_package_share_directory('darknet_ros_3d')
    person_tracker_dir = get_package_share_directory('person_tracker')

    # Configuration files
    darknet_3d_config = os.path.join(darknet_ros_3d_dir, 'config', 'darknet_3d.yaml')
    default_tracker_config = os.path.join(person_tracker_dir, 'config', 'clustering_params.yaml')

    nodes_to_launch = []

    # 1. Launch YOLO detector node (YOLOv8 via ultralytics_ros)
    if ultralytics_ros_dir and use_yolov8_value == 'true':
        yolo_detector_node = Node(
            package='ultralytics_ros',
            executable='yolo_detector_node.py',
            name='yolo_detector_node',
            output='screen',
            parameters=[
                {
                    'model_path': f'{yolo_model_value}.pt',
                    'model_type': 'yolov8',
                    'gpu_id': gpu_id_value,  # Pass gpu_id directly
                    'confidence_threshold': yolo_confidence_threshold,
                    'iou_threshold': yolo_iou_threshold,
                    'input_topic': yolo_input_topic,
                    'max_det': yolo_max_det,
                    'enable_visualization': yolo_enable_visualization,
                    'publish_image': yolo_publish_image,
                    'image_size': yolo_image_size,
                    'use_sim_time': use_sim_time_value == 'true'
                }
            ],
            emulate_tty=True,
            # Set CUDA_VISIBLE_DEVICES to only show the specified GPU
            # This prevents PyTorch from initializing GPU 0 when using other GPUs
            additional_env={'CUDA_VISIBLE_DEVICES': gpu_id_value}
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
            {'use_sim_time': use_sim_time_value == 'true'}
        ]
    )
    nodes_to_launch.append(darknet_ros_3d_node)

    # 3. Launch person tracker with clustering
    person_tracker_node = Node(
        package='person_tracker',
        executable='person_tracker_node',
        name='person_tracker',
        output='screen',
        parameters=[
            default_tracker_config,
            {'use_sim_time': use_sim_time_value == 'true'}
        ]
    )
    nodes_to_launch.append(person_tracker_node)

    # 4. Launch person tracker visualizer for RViz2
    person_visualizer_node = Node(
        package='person_tracker',
        executable='person_tracker_visualizer',
        name='person_tracker_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time_value == 'true'}
        ]
    )
    nodes_to_launch.append(person_visualizer_node)

    return nodes_to_launch


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

    gpu_id_arg = DeclareLaunchArgument(
        'gpu_id',
        default_value='0',
        description='GPU device ID for YOLO detection. Default: 0'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time. Default: true'
    )

    tracker_config_arg = DeclareLaunchArgument(
        'tracker_config',
        default_value='',
        description='Path to person tracker config file (optional)'
    )

    # YOLO parameters
    yolo_confidence_threshold_arg = DeclareLaunchArgument(
        'yolo_confidence_threshold',
        default_value='0.25',
        description='YOLO confidence threshold (0.0-1.0). Default: 0.25'
    )

    yolo_iou_threshold_arg = DeclareLaunchArgument(
        'yolo_iou_threshold',
        default_value='0.45',
        description='YOLO IOU threshold for NMS (0.0-1.0). Default: 0.45'
    )

    yolo_input_topic_arg = DeclareLaunchArgument(
        'yolo_input_topic',
        default_value='/task_generator_node/tiago_base/rgbd_camera/image',
        description='Input image topic for YOLO detector'
    )

    yolo_max_det_arg = DeclareLaunchArgument(
        'yolo_max_det',
        default_value='300',
        description='Maximum number of detections per image. Default: 300'
    )

    yolo_enable_visualization_arg = DeclareLaunchArgument(
        'yolo_enable_visualization',
        default_value='true',
        description='Enable visualization on detection image. Default: true'
    )

    yolo_publish_image_arg = DeclareLaunchArgument(
        'yolo_publish_image',
        default_value='true',
        description='Publish detection image. Default: true'
    )

    yolo_image_size_arg = DeclareLaunchArgument(
        'yolo_image_size',
        default_value='640',
        description='Input image size for YOLO (must be multiple of 32). Default: 640'
    )

    # Environment variable for line buffering
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Build launch description
    ld = LaunchDescription()

    # Add environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add launch arguments
    ld.add_action(use_yolov8_arg)
    ld.add_action(yolo_model_arg)
    ld.add_action(gpu_id_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(tracker_config_arg)
    
    # Add YOLO parameter arguments
    ld.add_action(yolo_confidence_threshold_arg)
    ld.add_action(yolo_iou_threshold_arg)
    ld.add_action(yolo_input_topic_arg)
    ld.add_action(yolo_max_det_arg)
    ld.add_action(yolo_enable_visualization_arg)
    ld.add_action(yolo_publish_image_arg)
    ld.add_action(yolo_image_size_arg)

    # Add nodes using OpaqueFunction to resolve launch arguments
    ld.add_action(OpaqueFunction(function=generate_launch_nodes))

    return ld
