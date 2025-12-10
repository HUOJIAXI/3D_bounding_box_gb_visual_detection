#!/usr/bin/env python3
"""
Ultralytics YOLO ROS2 Node
Supports YOLOv8, YOLOv11 for object detection with ROS2 integration
Compatible with darknet_ros message types
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, ObjectCount
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time


class YoloDetectorNode(Node):
    """
    ROS2 Node for YOLO object detection using Ultralytics
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('model_type', 'yolov8')  # yolov8, yolov11
        self.declare_parameter('device', 'cuda:0')  # cuda:0, cpu
        # Declare gpu_id as string to accept launch file input, then convert to int
        self.declare_parameter('gpu_id', '-1')  # GPU device ID (overrides device if >= 0)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('max_det', 300)  # Maximum detections per image
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('image_size', 640)  # Input image size for YOLO

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.model_type = self.get_parameter('model_type').value
        gpu_id_param = self.get_parameter('gpu_id').value
        self.device = self.get_parameter('device').value

        # Debug: log received parameters
        self.get_logger().info(f'Received gpu_id parameter: {gpu_id_param} (type: {type(gpu_id_param).__name__})')
        self.get_logger().info(f'Received device parameter: {self.device}')

        # Convert gpu_id to int (always treat as string first, then convert)
        try:
            if isinstance(gpu_id_param, str):
                gpu_id = int(gpu_id_param)
            else:
                gpu_id = int(gpu_id_param) if gpu_id_param is not None else -1
            self.get_logger().info(f'Parsed gpu_id: {gpu_id} (from input: {gpu_id_param})')
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid gpu_id value: {gpu_id_param} (error: {e}), using default device')
            gpu_id = -1

        # If gpu_id is provided (>= 0), use cuda:0
        # CUDA_VISIBLE_DEVICES in launch file restricts which GPU is visible,
        # so we always use cuda:0 (the first visible GPU)
        if gpu_id >= 0:
            self.device = 'cuda:0'
            self.get_logger().info(f'Using GPU device ID: {gpu_id} -> device: {self.device} (via CUDA_VISIBLE_DEVICES)')
        else:
            self.get_logger().info(f'gpu_id is {gpu_id} (not set or invalid), using device parameter: {self.device}')

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.iou_thresh = self.get_parameter('iou_threshold').value
        self.input_topic = self.get_parameter('input_topic').value
        self.max_det = self.get_parameter('max_det').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.publish_img = self.get_parameter('publish_image').value
        self.img_size = self.get_parameter('image_size').value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.get_logger().info(f'Loading {self.model_type} model from {self.model_path}...')
        self.get_logger().info(f'Device parameter received: {self.device}')
        
        # Check GPU availability
        import torch
        if torch.cuda.is_available():
            gpu_count = torch.cuda.device_count()
            self.get_logger().info(f'CUDA available with {gpu_count} GPU(s)')
            # Show info for all GPUs
            for i in range(gpu_count):
                gpu_name = torch.cuda.get_device_name(i)
                gpu_memory = torch.cuda.get_device_properties(i).total_memory / 1024**3  # GB
                self.get_logger().info(f'  GPU {i}: {gpu_name} ({gpu_memory:.1f} GB)')
        else:
            self.get_logger().warn('CUDA not available! Running on CPU (will be slow)')
        
        try:
            self.model = YOLO(self.model_path)

            # Verify device before moving model
            if 'cuda' in self.device:
                if not torch.cuda.is_available():
                    self.get_logger().warn(f'CUDA requested ({self.device}) but not available. Falling back to CPU.')
                    self.device = 'cpu'
                else:
                    device_id = int(self.device.split(':')[1]) if ':' in self.device else 0
                    gpu_count = torch.cuda.device_count()
                    if device_id >= gpu_count:
                        self.get_logger().warn(
                            f'Device cuda:{device_id} not found. Only {gpu_count} GPU(s) visible to PyTorch '
                            f'(check CUDA_VISIBLE_DEVICES if using specific GPU). Using cuda:0.'
                        )
                        self.device = 'cuda:0'
            
            self.model.to(self.device)
            
            # Verify model is on correct device
            if 'cuda' in self.device:
                actual_device = next(self.model.model.parameters()).device
                self.get_logger().info(f'Model loaded successfully on {actual_device}')
                if str(actual_device) != self.device:
                    self.get_logger().warn(f'Model device ({actual_device}) differs from requested ({self.device})')
            else:
                self.get_logger().info(f'Model loaded successfully on {self.device}')
            
            # Verify model is on correct device
            if 'cuda' in self.device:
                actual_device = next(self.model.model.parameters()).device
                self.get_logger().info(f'Model loaded successfully on {actual_device}')
                if str(actual_device) != self.device:
                    self.get_logger().warn(f'Model device ({actual_device}) differs from requested ({self.device})')
            else:
                self.get_logger().info(f'Model loaded successfully on {self.device}')
            
            self.get_logger().info(f'Model classes: {self.model.names}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        # Create QoS profile for image subscription
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos
        )

        # Create publishers (compatible with darknet_ros)
        self.bbox_pub = self.create_publisher(
            BoundingBoxes,
            '/darknet_ros/bounding_boxes',
            10
        )

        self.obj_count_pub = self.create_publisher(
            ObjectCount,
            '/darknet_ros/found_object',
            10
        )

        if self.publish_img:
            self.detection_img_pub = self.create_publisher(
                Image,
                '/darknet_ros/detection_image',
                10
            )

        # Statistics
        self.frame_count = 0
        self.total_time = 0.0
        self.last_fps_print = time.time()

        self.get_logger().info('Ultralytics YOLO Detector Node initialized')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Publishing to: /darknet_ros/bounding_boxes')
        self.get_logger().info(f'Confidence threshold: {self.conf_thresh}')
        self.get_logger().info(f'IOU threshold: {self.iou_thresh}')

    def image_callback(self, msg):
        """
        Callback function for image subscription
        """
        try:
            # Log first received image
            if self.frame_count == 0:
                self.get_logger().info(f'Received first image: {msg.width}x{msg.height}')
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Record start time
            start_time = time.time()

            # Run YOLO inference
            results = self.model.predict(
                cv_image,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                device=self.device,
                max_det=self.max_det,
                imgsz=self.img_size,
                verbose=False
            )

            # Calculate inference time
            inference_time = time.time() - start_time
            self.total_time += inference_time
            self.frame_count += 1

            # Process results
            result = results[0]  # Get first result (single image)

            # Create BoundingBoxes message
            bbox_msg = BoundingBoxes()
            bbox_msg.header = msg.header
            bbox_msg.image_header = msg.header

            # Extract detections
            boxes = result.boxes

            for box in boxes:
                # Get box coordinates (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Get confidence and class
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                # Create BoundingBox message
                bbox = BoundingBox()
                bbox.probability = conf
                bbox.xmin = int(x1)
                bbox.ymin = int(y1)
                bbox.xmax = int(x2)
                bbox.ymax = int(y2)
                bbox.id = cls_id
                bbox.class_id = cls_name

                bbox_msg.bounding_boxes.append(bbox)

            # Publish bounding boxes
            self.bbox_pub.publish(bbox_msg)

            # Create and publish object count
            obj_count_msg = ObjectCount()
            obj_count_msg.header = msg.header
            obj_count_msg.count = len(boxes)
            self.obj_count_pub.publish(obj_count_msg)

            # Visualize and publish detection image
            if self.publish_img and self.enable_viz:
                # Get annotated image from ultralytics
                annotated_img = result.plot()

                # Add FPS and detection count
                fps = 1.0 / inference_time if inference_time > 0 else 0
                text = f'FPS: {fps:.1f} | Detections: {len(boxes)}'
                cv2.putText(annotated_img, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Convert to ROS Image and publish
                detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
                detection_img_msg.header = msg.header
                self.detection_img_pub.publish(detection_img_msg)

            # Print FPS every 2 seconds
            if time.time() - self.last_fps_print > 2.0:
                avg_fps = self.frame_count / self.total_time if self.total_time > 0 else 0
                
                # Add GPU info if using CUDA
                gpu_info = ""
                if 'cuda' in self.device:
                    import torch
                    if torch.cuda.is_available():
                        # Extract device ID from self.device (e.g., 'cuda:1' -> 1)
                        device_id = int(self.device.split(':')[1]) if ':' in self.device else 0
                        gpu_mem_used = torch.cuda.memory_allocated(device_id) / 1024**3  # GB
                        gpu_mem_total = torch.cuda.get_device_properties(device_id).total_memory / 1024**3  # GB
                        gpu_info = f" | GPU{device_id}: {gpu_mem_used:.1f}/{gpu_mem_total:.1f} GB"
                
                self.get_logger().info(
                    f'Avg FPS: {avg_fps:.1f} | Last inference: {inference_time*1000:.1f}ms | Detections: {len(boxes)}{gpu_info}'
                )
                self.last_fps_print = time.time()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    """
    Main function
    """
    rclpy.init(args=args)

    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
