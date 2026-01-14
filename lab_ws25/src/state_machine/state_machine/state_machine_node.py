#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import time


class LaneBlockingNode(Node):
    def __init__(self):
        super().__init__('lane_blocking')

        # ==========================================
        # 1. CONFIGURATION (Based on your data)
        # ==========================================
        self.IMAGE_WIDTH_PX = 1344.0  # Camera Resolution
        self.IMAGE_CENTER_X = 672.0  # 1344 / 2

        self.TRACK_WIDTH_M = 1.60  # 160 cm Track
        self.REAL_CAR_WIDTH_M = 0.30  # 30 cm Car

        # Lane Math:
        # 1.6m Track / 3 Lanes = ~0.53m per lane.
        # The boundary between lanes is half that width (~0.26m).
        self.LANE_BOUNDARY_M = (self.TRACK_WIDTH_M / 3.0) / 2.0

        self.TIMEOUT_DURATION = 0.5

        # Ordered list of lanes
        self.LANE_ORDER = ["LEFT", "CENTER", "RIGHT"]

        # ==========================================
        # 2. STATE
        # ==========================================
        self.current_lane_index = 1  # Start assuming we are CENTER (Index 1)
        self.target_lane_cmd = "CENTER"
        self.last_detection_time = time.time()

        # ==========================================
        # 3. COMMUNICATION
        # ==========================================

        # INPUT: From Controller (Where am I?)
        # Expects: "LEFT", "CENTER", or "RIGHT"
        self.sub_controller = self.create_subscription(
            String,
            '/control/current_lane',
            self.controller_callback,
            10)

        # INPUT: From Camera (Where are they?)
        self.sub_perception = self.create_subscription(
            Detection2DArray,
            '/opponent_detections',
            self.perception_callback,
            10)

        # OUTPUT: To Planner (Where to go)
        self.pub_planning = self.create_publisher(
            String,
            '/planning/target_lane',
            10)

        self.timer = self.create_timer(0.1, self.watchdog_loop)

        self.get_logger().info(f"Blocking Node Started. Boundary: +/- {self.LANE_BOUNDARY_M:.2f}m")

    def controller_callback(self, msg):
        """Update our knowledge of which lane we are currently following."""
        if msg.data in self.LANE_ORDER:
            self.current_lane_index = self.LANE_ORDER.index(msg.data)

    def perception_callback(self, msg):
        opponent_found = False
        bbox_center_x = 0.0
        bbox_width_px = 1.0

        # 1. Find Opponent
        for detection in msg.detections:
            for result in detection.results:
                if result.id == "opponent_car":
                    bbox_center_x = detection.bbox.center.x
                    bbox_width_px = detection.bbox.size_x
                    opponent_found = True
                    break
            if opponent_found: break

        if opponent_found:

            self.get_logger().info(
                f"Raw X: {bbox_center_x:.0f} | Center: {self.IMAGE_CENTER_X} | Width: {bbox_width_px:.0f}", 
                throttle_duration_sec=0.5
            )

            
            self.last_detection_time = time.time()

            # --- STEP 1: PIXELS TO METERS ---
            pixel_offset = bbox_center_x - self.IMAGE_CENTER_X

            # Safety check for width
            if bbox_width_px < 1.0: bbox_width_px = 1.0

            # "Scale Trick": Use the known car width (0.3m) to find real distance
            # (Offset / Car_Px_Width) * Car_Real_Width
            real_offset_m = (pixel_offset / bbox_width_px) * self.REAL_CAR_WIDTH_M

            # --- STEP 2: DETERMINE SHIFT (Left/Straight/Right) ---
            lane_shift = 0

            if real_offset_m < -self.LANE_BOUNDARY_M:
                lane_shift = -1  # They are on the LEFT
                self.get_logger().info("Detect: LEFT ({real_offset_m:.2}m)")

            elif real_offset_m > self.LANE_BOUNDARY_M:
                lane_shift = 1  # They are on the RIGHT
                self.get_logger().info("Detect: RIGHT ({real_offset_m:.2}m)")

            else:
                lane_shift = 0  # They are STRAIGHT (Center relative to us)
                self.get_logger().info("Detect: STRAIGHT ({real_offset_m:.2}m)")

            # --- STEP 3: CALCULATE NEW TARGET ---
            # Example: I am CENTER (1) + They are LEFT (-1) = Target LEFT (0)
            target_index = self.current_lane_index + lane_shift

            # Limit index to 0, 1, or 2
            if target_index < 0: target_index = 0
            if target_index > 2: target_index = 2

            self.target_lane_cmd = self.LANE_ORDER[target_index]
            self.publish_command()

    def watchdog_loop(self):
        # If no opponent seen for 0.5s, return to CENTER
        if time.time() - self.last_detection_time > self.TIMEOUT_DURATION:
            if self.target_lane_cmd != "CENTER":
                self.target_lane_cmd = "CENTER"
                self.publish_command()

    def publish_command(self):
        msg = String()
        msg.data = self.target_lane_cmd
        self.pub_planning.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneBlockingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
