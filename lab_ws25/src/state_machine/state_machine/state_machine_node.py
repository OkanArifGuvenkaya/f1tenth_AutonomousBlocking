#!/usr/bin/env python3
"""Simple state machine: publishes selected raceline id.

This node cycles through raceline ids from 1..max_raceline every switch_period seconds,
and publishes the current selected raceline every publish_period seconds on `/selected_raceline`.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__('simple_state_machine')

        # parameters
        self.declare_parameter('selected_topic', '/selected_raceline')
        self.declare_parameter('max_raceline', 3)
        self.declare_parameter('switch_period_sec', 5.0)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('raceline_mode', 1)  # 0=fixed, 1=cycling, 2=keyboard
        self.declare_parameter('fixed_raceline_id', 2)
        self.declare_parameter('keyboard_topic', '/keyboard_input')

        self.selected_topic = self.get_parameter('selected_topic').get_parameter_value().string_value
        self.max_raceline = int(self.get_parameter('max_raceline').get_parameter_value().integer_value)
        self.switch_period = float(self.get_parameter('switch_period_sec').get_parameter_value().double_value)
        self.publish_period = float(self.get_parameter('publish_period_sec').get_parameter_value().double_value)
        self.raceline_mode = int(self.get_parameter('raceline_mode').get_parameter_value().integer_value)
        self.fixed_raceline_id = int(self.get_parameter('fixed_raceline_id').get_parameter_value().integer_value)
        self.keyboard_topic = self.get_parameter('keyboard_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Int32, self.selected_topic, 10)
        
        # Keyboard subscriber (for mode 2)
        if self.raceline_mode == 2:
            self.keyboard_sub = self.create_subscription(
                String, self.keyboard_topic, self.keyboard_callback, 10
            )

        # Set initial raceline based on mode
        if self.raceline_mode == 0:
            self.current = self.fixed_raceline_id
        else:
            self.current = 1
        
        # Timer to switch raceline every switch_period seconds (only in cycling mode)
        if self.raceline_mode == 1:
            self.switch_timer = self.create_timer(self.switch_period, self.switch_raceline_cb)
        
        # Timer to publish current raceline every publish_period seconds
        self.publish_timer = self.create_timer(self.publish_period, self.publish_raceline_cb)
        
        if self.raceline_mode == 0:
            self.get_logger().info(
                f'SimpleStateMachine [FIXED MODE]: Using raceline {self.fixed_raceline_id} only, '
                f'publishing to {self.selected_topic} every {self.publish_period}s'
            )
        elif self.raceline_mode == 1:
            self.get_logger().info(
                f'SimpleStateMachine [CYCLING MODE]: switching raceline every {self.switch_period}s, '
                f'publishing to {self.selected_topic} every {self.publish_period}s (1..{self.max_raceline})'
            )
        elif self.raceline_mode == 2:
            self.get_logger().info(
                f'SimpleStateMachine [KEYBOARD MODE]: Use arrow keys to control, '
                f'listening on {self.keyboard_topic}, publishing to {self.selected_topic} every {self.publish_period}s (1..{self.max_raceline})'
            )

    def switch_raceline_cb(self):
        """Switch to next raceline every switch_period seconds (only in cycling mode)"""
        if self.raceline_mode == 1:
            self.current = (self.current % self.max_raceline) + 1
            self.get_logger().info(f'🔄 Raceline CHANGED → raceline={self.current}')
    
    def keyboard_callback(self, msg):
        """Handle keyboard arrow keys for raceline switching"""
        if self.raceline_mode != 2:
            return
        
        if msg.data == 'left':
            self.change_raceline(-1)
        elif msg.data == 'right':
            self.change_raceline(1)
    
    def change_raceline(self, direction):
        """Change raceline with bounds checking
        
        Args:
            direction: -1 for left (decrease), +1 for right (increase)
        """
        new_raceline = self.current + direction
        
        # Check bounds
        if new_raceline < 1:
            self.get_logger().warn(f'⚠️  Already at leftmost raceline (1) - Cannot decrease!')
            return
        
        if new_raceline > self.max_raceline:
            self.get_logger().warn(f'⚠️  Already at rightmost raceline ({self.max_raceline}) - Cannot increase!')
            return
        
        # Update and publish
        self.current = new_raceline
        direction_arrow = '⬅️' if direction < 0 else '➡️'
        self.get_logger().info(f'{direction_arrow}  Raceline changed: {self.current - direction} → {self.current}')
        
        # Immediate publish
        self.publish_raceline_cb()

    def publish_raceline_cb(self):
        """Publish current raceline every publish_period seconds"""
        msg = Int32()
        msg.data = int(self.current)
        self.pub.publish(msg)
        # Silent publish - only log on raceline change


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
