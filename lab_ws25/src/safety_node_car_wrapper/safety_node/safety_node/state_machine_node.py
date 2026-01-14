#!/usr/bin/env python3
"""Simple state machine: publishes selected raceline id every N seconds.

This node is intentionally minimal for testing. It cycles integer ids
from 1..max_raceline and publishes them on `/selected_raceline` (or the
topic given by parameter). Use this to simulate dynamic raceline switching.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__('simple_state_machine')

        # parameters
        self.declare_parameter('selected_topic', '/selected_raceline')
        self.declare_parameter('max_raceline', 5)
        self.declare_parameter('period_sec', 5.0)

        self.selected_topic = self.get_parameter('selected_topic').get_parameter_value().string_value
        self.max_raceline = int(self.get_parameter('max_raceline').get_parameter_value().integer_value)
        self.period = float(self.get_parameter('period_sec').get_parameter_value().double_value)

        self.pub = self.create_publisher(Int32, self.selected_topic, 10)

        self.current = 0
        self.timer = self.create_timer(self.period, self.timer_cb)
        self.get_logger().info(f'SimpleStateMachine publishing to {self.selected_topic} every {self.period}s (1..{self.max_raceline})')

    def timer_cb(self):
        # cycle 1..max_raceline
        self.current = (self.current % self.max_raceline) + 1
        msg = Int32()
        msg.data = int(self.current)
        self.pub.publish(msg)
        self.get_logger().info(f'Published selected_raceline={msg.data}')


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
