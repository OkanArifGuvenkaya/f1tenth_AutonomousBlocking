"""
F1Tenth Full System Launch File
Launches both camera and opponent detection
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate launch description for full system
    """
    
    # Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sensors'), '/launch/camera.launch.py'
        ])
    )
    
    # Opponent detection launch
    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('opponent_detection'), '/launch/opponent_detector.launch.py'
        ])
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            '🏎️  F1TENTH FULL SYSTEM STARTUP\n',
            '=' * 80, '\n',
            'Starting components:\n',
            '  1. ZED Camera Publisher\n',
            '  2. YOLO Opponent Detector\n',
            '=' * 80, '\n'
        ]
    )
    
    return LaunchDescription([
        # Environment setup
        SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/aarch64-linux-gnu/libgomp.so.1'),
        
        # Startup message
        startup_msg,
        
        # Launch both nodes
        camera_launch,
        detection_launch
    ])
