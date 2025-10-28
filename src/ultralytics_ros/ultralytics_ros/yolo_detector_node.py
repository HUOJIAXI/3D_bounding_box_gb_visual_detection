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
        self.device = self.get_parameter('device').value
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
        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
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
                self.get_logger().info(
                    f'Avg FPS: {avg_fps:.1f} | Last inference: {inference_time*1000:.1f}ms | Detections: {len(boxes)}'
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
