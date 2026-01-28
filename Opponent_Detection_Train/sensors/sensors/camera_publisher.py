#!/usr/bin/env python3
"""
ZED Camera Publisher Node
Publishes ZED camera images to ROS2 with configurable parameters
Author: F1Tenth Team
Date: January 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class CameraPublisher(Node):
    """
    ROS2 Node for publishing ZED camera images
    """
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters WITHOUT defaults - must be provided via config file
        self.declare_parameter('camera_topic')
        self.declare_parameter('frame_id')
        self.declare_parameter('publish_rate')
        self.declare_parameter('image_width')
        self.declare_parameter('image_height')
        self.declare_parameter('camera_index_start')
        self.declare_parameter('camera_index_end')
        self.declare_parameter('min_camera_width')
        
        # Get parameters and validate they exist
        try:
            camera_topic = self.get_parameter('camera_topic').value
            frame_id = self.get_parameter('frame_id').value
            publish_rate = self.get_parameter('publish_rate').value
            image_width = self.get_parameter('image_width').value
            image_height = self.get_parameter('image_height').value
            camera_index_start = self.get_parameter('camera_index_start').value
            camera_index_end = self.get_parameter('camera_index_end').value
            min_camera_width = self.get_parameter('min_camera_width').value
        except Exception as e:
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ CONFIGURATION ERROR!')
            self.get_logger().error('=' * 60)
            self.get_logger().error('Failed to load required parameters from config file.')
            self.get_logger().error('Please ensure you are using the config file:')
            self.get_logger().error('  config/camera_params.yaml')
            self.get_logger().error('')
            self.get_logger().error('Run the node with:')
            self.get_logger().error('  ros2 run sensors camera_publisher --ros-args --params-file config/camera_params.yaml')
            self.get_logger().error('  OR use launch file: ros2 launch sensors camera.launch.py')
            self.get_logger().error('=' * 60)
            raise RuntimeError(f'Missing required parameter: {str(e)}')
        
        # Validate parameter types and values
        if not isinstance(camera_topic, str) or not camera_topic:
            raise ValueError('camera_topic must be a non-empty string')
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError('frame_id must be a non-empty string')
        if not isinstance(publish_rate, (int, float)) or publish_rate <= 0:
            raise ValueError('publish_rate must be a positive number')
        if not isinstance(image_width, int) or image_width <= 0:
            raise ValueError('image_width must be a positive integer')
        if not isinstance(image_height, int) or image_height <= 0:
            raise ValueError('image_height must be a positive integer')
        if not isinstance(camera_index_start, int) or camera_index_start < 0:
            raise ValueError('camera_index_start must be a non-negative integer')
        if not isinstance(camera_index_end, int) or camera_index_end < camera_index_start:
            raise ValueError('camera_index_end must be >= camera_index_start')
        if not isinstance(min_camera_width, int) or min_camera_width <= 0:
            raise ValueError('min_camera_width must be a positive integer')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('F1Tenth ZED Camera Publisher Node')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Searching for ZED camera...')
        
        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        self.frame_id = frame_id
        
        # Find and open ZED camera
        self.cap = None
        for camera_index in range(camera_index_start, camera_index_end + 1):
            self.get_logger().info(f'Trying camera index {camera_index}...')
            test_cap = cv2.VideoCapture(camera_index)
            if test_cap.isOpened():
                ret, frame = test_cap.read()
                if ret:
                    height, width = frame.shape[:2]
                    self.get_logger().info(f'  Camera {camera_index}: {width}x{height}')
                    
                    # ZED cameras typically have higher resolution
                    if width >= min_camera_width:
                        self.get_logger().info(f'✅ Found ZED camera at index {camera_index}')
                        self.cap = test_cap
                        break
                    else:
                        test_cap.release()
                else:
                    test_cap.release()
        
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ CAMERA NOT FOUND!')
            self.get_logger().error('=' * 60)
            self.get_logger().error('Failed to find ZED camera in the specified index range.')
            self.get_logger().error('Please ensure:')
            self.get_logger().error('  1. ZED camera is connected')
            self.get_logger().error('  2. Camera index range is correct')
            self.get_logger().error('  3. You have camera access permissions')
            self.get_logger().error('=' * 60)
            raise RuntimeError('Camera not found')
        
        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        
        # Verify actual resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Publisher
        self.publisher = self.create_publisher(Image, camera_topic, 10)
        
        # Timer to publish at specified rate
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Statistics
        self.frame_count = 0
        
        self.get_logger().info('-' * 60)
        self.get_logger().info('Configuration:')
        self.get_logger().info(f'  Camera topic: {camera_topic}')
        self.get_logger().info(f'  Frame ID: {frame_id}')
        self.get_logger().info(f'  Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'  Resolution: {actual_width}x{actual_height}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('🚀 Camera publisher is ready!')
        self.get_logger().info('=' * 60)
    
    def timer_callback(self):
        """
        Timer callback to capture and publish camera frames
        """
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # Publish image
            self.publisher.publish(msg)
            
            self.frame_count += 1
        else:
            self.get_logger().warn('Failed to capture frame from camera')
    
    def destroy_node(self):
        """
        Cleanup when node is destroyed
        """
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info(f'Camera released. Total frames published: {self.frame_count}')
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the node
    """
    rclpy.init(args=args)
    
    try:
        node = CameraPublisher()
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
