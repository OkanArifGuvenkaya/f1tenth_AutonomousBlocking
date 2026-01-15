#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class SimpleCarSimulator(Node):
    def __init__(self):
        super().__init__('simple_car_simulator')
        
        # Subscribe to drive commands from pure pursuit
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )
        
        # Publish odometry (simulated particle filter output)
        self.odom_pub = self.create_publisher(Odometry, '/pf/pose/odom', 10)
        
        # TF broadcaster for visualization in RViz
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for physics update (50 Hz)
        self.timer = self.create_timer(0.02, self.update_pose)
        
        # Car state - Start on raceline_1
        self.x = -0.86  # Match raceline_1 start position
        self.y = -3.19
        self.theta = -0.5  # Initial heading (approximate, will adjust quickly)
        self.velocity = 0.0  # Current velocity
        self.steering_angle = 0.0  # Current steering
        
        # Car parameters (F1TENTH approximate)
        self.wheelbase = 0.33  # meters (distance between front and rear axles)
        self.max_steering = 0.4  # radians (~23 degrees)
        self.max_speed = 8.0  # m/s
        
        # Commanded values from pure pursuit
        self.cmd_velocity = 0.0
        self.cmd_steering = 0.0
        
        self.get_logger().info("🚗 Simple Car Simulator started!")
        self.get_logger().info(f"   Initial position: ({self.x:.2f}, {self.y:.2f})")
        
    def drive_callback(self, msg):
        """Receive drive commands from pure pursuit"""
        # ✅ Negatif hızları pozitif yap (CSV'deki negatif hızlar yüzünden)
        self.cmd_velocity = abs(msg.drive.speed)
        self.cmd_steering = msg.drive.steering_angle
        
        # Clamp to limits
        self.cmd_velocity = max(0, min(self.max_speed, self.cmd_velocity))
        self.cmd_steering = max(-self.max_steering, min(self.max_steering, self.cmd_steering))
        
        # ✅ Sürekli log (spam değil, sadece değiştiğinde)
        self.get_logger().info(f"📥 CMD: speed={self.cmd_velocity:.2f}m/s, steering={self.cmd_steering:.2f}rad", throttle_duration_sec=1.0)
        
    def update_pose(self):
        """Update car pose using bicycle kinematic model"""
        dt = 0.02  # 50 Hz
        
        # Simple first-order lag for more realistic dynamics
        alpha_v = 0.3  # Velocity response rate
        alpha_s = 0.5  # Steering response rate
        
        old_velocity = self.velocity
        self.velocity += alpha_v * (self.cmd_velocity - self.velocity)
        self.steering_angle += alpha_s * (self.cmd_steering - self.steering_angle)
        
        # ✅ Log velocity changes
        if abs(self.velocity - old_velocity) > 0.01:
            self.get_logger().info(f"🏎️ VEL: {self.velocity:.2f}m/s (cmd: {self.cmd_velocity:.2f})", throttle_duration_sec=1.0)
        
        # Bicycle kinematic model
        if abs(self.steering_angle) > 0.001:
            # Turning radius
            turning_radius = self.wheelbase / math.tan(self.steering_angle)
            
            # Angular velocity
            omega = self.velocity / turning_radius
            
            # Update position and heading
            self.x += self.velocity * math.cos(self.theta) * dt
            self.y += self.velocity * math.sin(self.theta) * dt
            self.theta += omega * dt
        else:
            # Straight line motion
            self.x += self.velocity * math.cos(self.theta) * dt
            self.y += self.velocity * math.sin(self.theta) * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # ✅ Log position periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:  # Every 2 seconds
            self.get_logger().info(f"📍 POS: ({self.x:.2f}, {self.y:.2f}), θ={self.theta:.2f}, v={self.velocity:.2f}", throttle_duration_sec=2.0)
        
        # Publish odometry
        self.publish_odom()
        
    def publish_odom(self):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'laser'  # Pure pursuit expects 'laser' frame
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        msg.twist.twist.linear.x = self.velocity
        msg.twist.twist.angular.z = self.velocity * math.tan(self.steering_angle) / self.wheelbase if abs(self.steering_angle) > 0.001 else 0.0
        
        self.odom_pub.publish(msg)
        
        # Broadcast TF for RViz visualization
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser'  # Match odometry child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = SimpleCarSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()