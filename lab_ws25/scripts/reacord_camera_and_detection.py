#!/usr/bin/env python3
"""
Detection Recorder
Records the detection visualization topic and saves it as a video when terminated
Optimized to write frames directly to disk to avoid RAM issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class DetectionRecorder(Node):
    def __init__(self):
        super().__init__('detection_recorder')
        
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_count = 0
        self.frame_width = None
        self.frame_height = None
        self.fps = 30.0  # Default FPS for output video
        self.output_file = None
        
        # Create output directory and filename
        output_dir = os.path.expanduser('~/detection_recordings')
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = os.path.join(output_dir, f'detection_recording_{timestamp}.mp4')
        
        # Subscribe to visualization topic
        self.subscription = self.create_subscription(
            Image,
            '/opponent_detections/visualization',
            self.image_callback,
            30
        )
        
        self.get_logger().info('Detection Recorder started')
        self.get_logger().info(f'Video will be saved to: {self.output_file}')
        self.get_logger().info('Recording detections... Press Ctrl+C to stop and save video')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Initialize VideoWriter on first frame
            if self.video_writer is None:
                self.frame_height, self.frame_width = cv_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(
                    self.output_file, 
                    fourcc, 
                    self.fps,
                    (self.frame_width, self.frame_height)
                )
                self.get_logger().info(f'Started recording at {self.frame_width}x{self.frame_height}')
            
            # Write frame directly to video file (no RAM accumulation!)
            self.video_writer.write(cv_image)
            self.frame_count += 1
            
            # Optional: Display the frame while recording
            cv2.imshow('Recording Detection Visualization (Press q to stop)', cv_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('Quit key pressed, stopping...')
                raise KeyboardInterrupt
            
            # Log progress every 100 frames
            if self.frame_count % 100 == 0:
                duration = self.frame_count / self.fps
                self.get_logger().info(f'Recorded {self.frame_count} frames ({duration:.1f}s)')
            
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')
    
    def close_video(self):
        """Finalize and close the video file"""
        if self.video_writer is None:
            self.get_logger().warn('No frames recorded, nothing to save')
            return
        
        self.get_logger().info('Finalizing video...')
        
        # Release the VideoWriter (this finalizes the file)
        self.video_writer.release()
        
        duration = self.frame_count / self.fps
        self.get_logger().info(f'✓ Video saved successfully to: {self.output_file}')
        self.get_logger().info(f'✓ Total frames: {self.frame_count}')
        self.get_logger().info(f'✓ Duration: {duration:.2f} seconds')
        self.get_logger().info(f'✓ Resolution: {self.frame_width}x{self.frame_height}')


def main(args=None):
    rclpy.init(args=args)
    
    recorder = DetectionRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Recording stopped by user')
    finally:
        # Finalize video before cleanup
        recorder.close_video()
        
        cv2.destroyAllWindows()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
