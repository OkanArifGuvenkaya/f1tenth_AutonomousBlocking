#!/usr/bin/env python3
"""
Keyboard Listener Node for Raceline Control
Listens to arrow keys (LEFT/RIGHT) and publishes to /keyboard_input topic
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select


class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        
        # Publisher for key events
        self.key_pub = self.create_publisher(String, '/keyboard_input', 10)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎹 Keyboard Listener Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('   ⬅️  LEFT Arrow:  Switch to left raceline (decrease)')
        self.get_logger().info('   ➡️  RIGHT Arrow: Switch to right raceline (increase)')
        self.get_logger().info('   Press Ctrl+C to exit')
        self.get_logger().info('=' * 60)
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Start keyboard reading timer (50Hz)
        self.timer = self.create_timer(0.02, self.read_keyboard)
        
        # Set terminal to raw mode
        tty.setraw(sys.stdin.fileno())
    
    def read_keyboard(self):
        """Read keyboard input in non-blocking mode"""
        try:
            # Check if input is available
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                
                # Escape sequences for arrow keys
                if char == '\x1b':  # ESC
                    # Read next two characters for arrow keys
                    char += sys.stdin.read(2)
                    
                    if char == '\x1b[D':  # Left arrow
                        msg = String()
                        msg.data = 'left'
                        self.key_pub.publish(msg)
                        self.get_logger().info('⬅️  LEFT Arrow pressed - Switching to left raceline')
                    
                    elif char == '\x1b[C':  # Right arrow
                        msg = String()
                        msg.data = 'right'
                        self.key_pub.publish(msg)
                        self.get_logger().info('➡️  RIGHT Arrow pressed - Switching to right raceline')
                
                elif char == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
        
        except Exception as e:
            if not isinstance(e, KeyboardInterrupt):
                self.get_logger().error(f'Error reading keyboard: {e}')
    
    def destroy_node(self):
        """Restore terminal settings on exit"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.get_logger().info('🛑 Keyboard listener stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
