#!/usr/bin/env python3
"""
ZED Camera Publisher Node
Publishes ZED camera images to ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Find and open ZED camera
        self.cap = None
        for camera_index in range(5):
            test_cap = cv2.VideoCapture(camera_index)
            if test_cap.isOpened():
                ret, frame = test_cap.read()
                if ret:
                    height, width = frame.shape[:2]
                    if width >= 1280:
                        self.get_logger().info(f'Found ZED camera at index {camera_index} ({width}x{height})')
                        self.cap = test_cap
                        break
                    else:
                        test_cap.release()
                else:
                    test_cap.release()
        
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Failed to open ZED camera')
            raise RuntimeError('Camera not found')
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Timer to publish at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.get_logger().info('Camera publisher started')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.publisher.publish(msg)
    
    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
