#!/usr/bin/env python3
"""
F1Tenth Opponent Car Detection Node
Real-time YOLO-based detection using ZED camera
Author: F1Tenth Team
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class OpponentYOLODetector(Node):
    """
    ROS2 Node for real-time opponent car detection using YOLO
    """
    
    def __init__(self):
        super().__init__('opponent_detector')
        
        # Declare parameters WITHOUT defaults - must be provided via config file
        self.declare_parameter('model_path')
        self.declare_parameter('confidence_threshold')
        self.declare_parameter('camera_topic')
        self.declare_parameter('detection_topic')
        self.declare_parameter('visualization_topic')
        self.declare_parameter('publish_visualization')
        self.declare_parameter('use_gpu')
        self.declare_parameter('log_frequency')
        
        # Get parameters and validate they exist
        try:
            model_path = self.get_parameter('model_path').value
            self.conf_threshold = self.get_parameter('confidence_threshold').value
            camera_topic = self.get_parameter('camera_topic').value
            detection_topic = self.get_parameter('detection_topic').value
            viz_topic = self.get_parameter('visualization_topic').value
            self.publish_viz = self.get_parameter('publish_visualization').value
            use_gpu = self.get_parameter('use_gpu').value
            self.log_freq = self.get_parameter('log_frequency').value
        except Exception as e:
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ CONFIGURATION ERROR!')
            self.get_logger().error('=' * 60)
            self.get_logger().error('Failed to load required parameters from config file.')
            self.get_logger().error('Please ensure you are using the config file:')
            self.get_logger().error('  config/opponent_detector_params.yaml')
            self.get_logger().error('')
            self.get_logger().error('Run the node with:')
            self.get_logger().error('  ros2 run opponent_detection opponent_detector --ros-args --params-file config/opponent_detector_params.yaml')
            self.get_logger().error('  OR use launch file: ros2 launch opponent_detection opponent_detector.launch.py')
            self.get_logger().error('=' * 60)
            raise RuntimeError(f'Missing required parameter: {str(e)}')
        
        # Validate parameter types and values
        if not isinstance(model_path, str) or not model_path:
            raise ValueError('model_path must be a non-empty string')
        if not isinstance(self.conf_threshold, (int, float)) or not (0.0 <= self.conf_threshold <= 1.0):
            raise ValueError('confidence_threshold must be a number between 0.0 and 1.0')
        if not isinstance(camera_topic, str) or not camera_topic:
            raise ValueError('camera_topic must be a non-empty string')
        if not isinstance(detection_topic, str) or not detection_topic:
            raise ValueError('detection_topic must be a non-empty string')
        if not isinstance(viz_topic, str) or not viz_topic:
            raise ValueError('visualization_topic must be a non-empty string')
        if not isinstance(self.publish_viz, bool):
            raise ValueError('publish_visualization must be a boolean')
        if not isinstance(use_gpu, bool):
            raise ValueError('use_gpu must be a boolean')
        if not isinstance(self.log_freq, (int, float)) or self.log_freq <= 0:
            raise ValueError('log_frequency must be a positive number')
        
        # If model_path is relative, resolve it from package share directory
        if not os.path.isabs(model_path):
            try:
                package_share_dir = get_package_share_directory('opponent_detection')
                model_path = os.path.join(package_share_dir, model_path)
            except Exception as e:
                self.get_logger().warn(
                    f'Could not find package share directory, using relative path: {e}'
                )
        
        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ MODEL FILE NOT FOUND!')
            self.get_logger().error('=' * 60)
            self.get_logger().error(f'Model path: {model_path}')
            self.get_logger().error('Please ensure the model file exists in opponent_detection/models/')
            self.get_logger().error('=' * 60)
            raise FileNotFoundError(f'Model file not found: {model_path}')
        
        # Initialize YOLO model
        self.device = 0 if (torch.cuda.is_available() and use_gpu) else 'cpu'
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('F1Tenth Opponent Car Detection Node')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ Model loaded successfully!')
            self.get_logger().info(f'Device: {self.device}')
            if torch.cuda.is_available():
                self.get_logger().info(f'GPU: {torch.cuda.get_device_name(0)}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {str(e)}')
            raise
        
        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            30
        )
        
        # Publishers
        # 1. Detection results (vision_msgs/Detection2DArray)
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            detection_topic,
            30
        )
        
        # 2. Visualization image (optional)
        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                Image,
                viz_topic,
                10
            )
        
        # Statistics
        self.frame_count = 0
        self.total_detections = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info('-' * 60)
        self.get_logger().info('Configuration:')
        self.get_logger().info(f'  Camera topic: {camera_topic}')
        self.get_logger().info(f'  Detection topic: {detection_topic}')
        if self.publish_viz:
            self.get_logger().info(f'  Visualization topic: {viz_topic}')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  Device: {self.device}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('🚀 Node is ready! Waiting for camera images...')
        self.get_logger().info('=' * 60)
    
    def image_callback(self, msg):
        """
        Callback function for processing camera images
        
        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model.predict(
                cv_image,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
                stream=False
            )[0]
            
            # Create Detection2DArray message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # Process each detection
            num_detections = 0
            if results.boxes is not None and len(results.boxes) > 0:
                for box in results.boxes:
                    # Extract bounding box coordinates (xyxy format)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    
                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Bounding box center and size (convert to integers for pixel coordinates)
                    detection.bbox.center.x = float(int((x1 + x2) / 2.0))
                    detection.bbox.center.y = float(int((y1 + y2) / 2.0))
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = float(int(x2 - x1))
                    detection.bbox.size_y = float(int(y2 - y1))
                    
                    # Object hypothesis (class and confidence)
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = "opponent_car"
                    hypothesis.score = confidence
                    detection.results.append(hypothesis)
                    
                    # Add to detections array
                    detections_msg.detections.append(detection)
                    num_detections += 1
                
                self.total_detections += num_detections
            
            # Publish detections
            self.detection_pub.publish(detections_msg)
            
            # Publish visualization (optional)
            if self.publish_viz:
                viz_image = results.plot()  # YOLO built-in visualization
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)
            
            # Update statistics
            self.frame_count += 1
            
            # Periodic logging
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_log_time).nanoseconds / 1e9
            
            if time_diff >= self.log_freq:
                if num_detections > 0:
                    self.get_logger().info(
                        f'Frame {self.frame_count}: Detected {num_detections} opponent car(s) '
                        f'| Total: {self.total_detections}'
                    )
                self.last_log_time = current_time
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    """
    Main function to initialize and run the node
    """
    rclpy.init(args=args)
    
    try:
        node = OpponentYOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
