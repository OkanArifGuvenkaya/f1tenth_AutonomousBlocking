#!/usr/bin/env python3
"""
Simple Detection Viewer
Displays the visualization topic with OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        
        self.bridge = CvBridge()
        
        # Subscribe to visualization topic
        self.subscription = self.create_subscription(
            Image,
            '/opponent_detections/visualization',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Detection Viewer started')
        self.get_logger().info('Press "q" in the image window to quit')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display image
            cv2.imshow('Detection Visualization', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    viewer = DetectionViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
