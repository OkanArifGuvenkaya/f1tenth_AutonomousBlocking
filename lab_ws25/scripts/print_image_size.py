#!/usr/bin/env python3
"""Print the width and height of the next Image message on the camera topic.

Usage:
  python3 tools/print_image_size.py

This subscribes once to `/camera/image_raw` by default and logs the
image width and height, then exits.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SizePrinter(Node):
    def __init__(self, topic: str = '/camera/image_raw'):
        super().__init__('size_printer')
        self.sub = self.create_subscription(Image, topic, self.cb, 1)

    def cb(self, msg: Image):
        # Use logger so output is clearly visible in ROS2 context
        self.get_logger().info(f'width={msg.width} height={msg.height}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SizePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
