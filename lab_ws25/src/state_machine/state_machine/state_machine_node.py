#!/usr/bin/env python3
"""Simple state machine: publishes selected raceline id.

This node cycles through raceline ids from 1..max_raceline every switch_period seconds,
and publishes the current selected raceline every publish_period seconds on `/selected_raceline`.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from vision_msgs.msg import Detection2DArray


class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__('simple_state_machine')

        # parameters
        self.declare_parameter('selected_topic', '/selected_raceline')
        self.declare_parameter('max_raceline', 3)
        self.declare_parameter('switch_period_sec', 5.0)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('raceline_mode', 1)  # 0=fixed, 1=cycling, 2=keyboard, 3=opponent_detection
        self.declare_parameter('fixed_raceline_id', 2)
        self.declare_parameter('keyboard_topic', '/keyboard_input')
        self.declare_parameter('opponent_detection_topic', '/opponent_detections')
        self.declare_parameter('detection_print_interval', 2.0)
        self.declare_parameter('detection_height_threshold', 100.0)

        self.selected_topic = self.get_parameter('selected_topic').get_parameter_value().string_value
        self.max_raceline = int(self.get_parameter('max_raceline').get_parameter_value().integer_value)
        self.switch_period = float(self.get_parameter('switch_period_sec').get_parameter_value().double_value)
        self.publish_period = float(self.get_parameter('publish_period_sec').get_parameter_value().double_value)
        self.raceline_mode = int(self.get_parameter('raceline_mode').get_parameter_value().integer_value)
        self.fixed_raceline_id = int(self.get_parameter('fixed_raceline_id').get_parameter_value().integer_value)
        self.keyboard_topic = self.get_parameter('keyboard_topic').get_parameter_value().string_value
        self.opponent_detection_topic = self.get_parameter('opponent_detection_topic').get_parameter_value().string_value
        self.detection_print_interval = float(self.get_parameter('detection_print_interval').get_parameter_value().double_value)
        self.detection_height_threshold = float(self.get_parameter('detection_height_threshold').get_parameter_value().double_value)

        self.pub = self.create_publisher(Int32, self.selected_topic, 10)
        
        # Keyboard subscriber (for mode 2)
        if self.raceline_mode == 2:
            self.keyboard_sub = self.create_subscription(
                String, self.keyboard_topic, self.keyboard_callback, 10
            )
        
        # Opponent detection subscriber (for mode 3)
        if self.raceline_mode == 3:
            self.opponent_sub = self.create_subscription(
                Detection2DArray, self.opponent_detection_topic, self.opponent_detection_callback, 10
            )
            self.detection_count = 0  # Counter for received detections
            
            # ZED camera stereo image parameters
            self.image_width = 1344  # Total width of stereo image
            self.image_height = 376  # Height of stereo image
            self.center_divider = self.image_width / 2  # 672 pixels - divides left/right cameras
            
            # Throttling for printing (print every N seconds from config)
            self.print_interval = self.detection_print_interval
            self.last_print_time = self.get_clock().now()
            self.latest_detection_msg = None

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
        elif self.raceline_mode == 3:
            self.get_logger().info(
                f'SimpleStateMachine [OPPONENT DETECTION MODE]: Monitoring opponent detections, '
                f'listening on {self.opponent_detection_topic}, publishing to {self.selected_topic} every {self.publish_period}s'
            )
            self.get_logger().info(
                f'ZED Stereo Camera: {self.image_width}x{self.image_height} pixels '
                f'(Left: 0-{int(self.center_divider)}, Right: {int(self.center_divider)}-{self.image_width})'
            )
            self.get_logger().info(f'Detection print interval: {self.print_interval}s (throttled for readability)')
            self.get_logger().info(
                f'Distance estimation threshold: {self.detection_height_threshold} pixels '
                f'(height > {self.detection_height_threshold}px = CLOSE, <= {self.detection_height_threshold}px = FAR)'
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
    
    def opponent_detection_callback(self, msg):
        """Handle opponent detection messages and print details (throttled to 2 seconds)"""
        if self.raceline_mode != 3:
            return
        
        # Store the latest detection message
        self.latest_detection_msg = msg
        
        # Check if enough time has passed since last print
        current_time = self.get_clock().now()
        time_since_last_print = (current_time - self.last_print_time).nanoseconds / 1e9
        
        if time_since_last_print < self.print_interval:
            # Skip printing, but update the stored message
            return
        
        # Update last print time
        self.last_print_time = current_time
        self.detection_count += 1
        
        num_detections = len(msg.detections)
        
        # Separate detections by camera (left vs right)
        left_detections = []
        right_detections = []
        
        for detection in msg.detections:
            bbox = detection.bbox
            center_x = bbox.center.x
            
            # Determine which camera based on center X position
            if center_x < self.center_divider:
                left_detections.append(detection)
            else:
                right_detections.append(detection)
        
        # Print header
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'🚗 OPPONENT DETECTION #{self.detection_count}')
        self.get_logger().info(f'   Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        self.get_logger().info(f'   Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'   Total detections: {num_detections} (Left: {len(left_detections)}, Right: {len(right_detections)})')
        self.get_logger().info('=' * 80)
        
        if num_detections == 0:
            self.get_logger().info('   ℹ️  No opponent cars detected')
        else:
            # Print LEFT camera detections
            if len(left_detections) > 0:
                self.get_logger().info('')
                self.get_logger().info('📷 LEFT CAMERA DETECTIONS:')
                self.get_logger().info('-' * 80)
                for idx, detection in enumerate(left_detections, 1):
                    self._print_detection_details(detection, idx, 'LEFT')
            
            # Print RIGHT camera detections
            if len(right_detections) > 0:
                self.get_logger().info('')
                self.get_logger().info('📷 RIGHT CAMERA DETECTIONS:')
                self.get_logger().info('-' * 80)
                for idx, detection in enumerate(right_detections, 1):
                    self._print_detection_details(detection, idx, 'RIGHT')
        
        self.get_logger().info('=' * 80)
    
    def _print_detection_details(self, detection, idx, camera_side):
        """Helper function to print detection details
        
        Args:
            detection: Detection2D message
            idx: Detection index number
            camera_side: 'LEFT' or 'RIGHT'
        """
        bbox = detection.bbox
        center_x = bbox.center.x
        center_y = bbox.center.y
        width = bbox.size_x
        height = bbox.size_y
        
        # Calculate normalized position within the specific camera (0-672 pixels)
        if camera_side == 'LEFT':
            camera_x = center_x  # Already in 0-672 range
            normalized_x = center_x / self.center_divider
        else:  # RIGHT
            camera_x = center_x - self.center_divider  # Convert to 0-672 range
            normalized_x = camera_x / self.center_divider
        
        normalized_y = center_y / self.image_height
        
        # Get confidence score if available
        confidence = 0.0
        class_id = "unknown"
        if detection.results and len(detection.results) > 0:
            confidence = detection.results[0].score
            class_id = detection.results[0].id
        
        # Calculate bounding box corners
        x1 = center_x - width / 2
        y1 = center_y - height / 2
        x2 = center_x + width / 2
        y2 = center_y + height / 2
        
        # Estimate distance based on bounding box height
        if height > self.detection_height_threshold:
            distance_estimate = "CLOSE 🔴"
            distance_indicator = "⚠️ "
        else:
            distance_estimate = "FAR 🟢"
            distance_indicator = "ℹ️  "
        
        self.get_logger().info(f'   Detection {idx}:')
        self.get_logger().info(f'      Class: {class_id}')
        self.get_logger().info(f'      Confidence: {confidence:.2%}')
        self.get_logger().info(f'      Camera: {camera_side}')
        self.get_logger().info(f'      {distance_indicator}Distance Estimate: {distance_estimate}')
        self.get_logger().info(f'      Bounding Box:')
        self.get_logger().info(f'         Center (Stereo): ({center_x:.1f}, {center_y:.1f}) pixels')
        self.get_logger().info(f'         Center (Single Camera): ({camera_x:.1f}, {center_y:.1f}) pixels')
        self.get_logger().info(f'         Normalized (Camera): ({normalized_x:.3f}, {normalized_y:.3f})')
        self.get_logger().info(f'         Size: {width:.1f} x {height:.1f} pixels')
        self.get_logger().info(f'         Area: {width * height:.0f} pixels²')
        self.get_logger().info(f'         Corners (Stereo): ({x1:.1f}, {y1:.1f}) to ({x2:.1f}, {y2:.1f})')
        self.get_logger().info(f'      ---')
    
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
