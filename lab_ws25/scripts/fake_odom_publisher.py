#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.publisher = self.create_publisher(Odometry, '/pf/pose/odom', 10)
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz
        
        # Simulated car position (moves in a circle)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 2.0  # m/s
        
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'laser'
        
        # Update position (circular motion for test)
        self.theta += 0.05  # rotation
        self.x += self.speed * 0.05 * math.cos(self.theta)
        self.y += self.speed * 0.05 * math.sin(self.theta)
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        msg.twist.twist.linear.x = self.speed
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()