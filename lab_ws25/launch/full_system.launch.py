"""
F1Tenth Full System Launch File
Launches all components of the autonomous racing system:
  1. State Machine (raceline selection)
  2. Pure Pursuit (path tracking controller)
  3. Particle Filter (localization)
  4. ZED Camera Publisher
  5. YOLO Opponent Detector
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
    
    # 1. State Machine launch
    state_machine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('state_machine'), '/launch/state_machine.launch.py'
        ])
    )
    
    # 2. Pure Pursuit launch
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('pure_pursuit'), '/launch/pure_pursuit_launch.py'
        ])
    )
    
    # 3. Particle Filter (Localization) launch
    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('particle_filter'), '/launch/localize_launch.py'
        ])
    )
    
    # 4. Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sensors'), '/launch/camera.launch.py'
        ])
    )
    
    # 5. Opponent detection launch
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
            '🏎️  F1TENTH FULL AUTONOMOUS SYSTEM STARTUP\n',
            '=' * 80, '\n',
            'Starting components:\n',
            '  1. State Machine (Raceline Selection)\n',
            '  2. Pure Pursuit (Path Tracking Controller)\n',
            '  3. Particle Filter (Localization)\n',
            '  4. ZED Camera Publisher\n',
            '  5. YOLO Opponent Detector\n',
            '=' * 80, '\n',
            'System initialization in progress...\n',
            '=' * 80, '\n'
        ]
    )
    
    return LaunchDescription([
        # Environment setup
        SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/aarch64-linux-gnu/libgomp.so.1'),
        
        # Startup message
        startup_msg,
        
        # Launch all components
        state_machine_launch,
        pure_pursuit_launch,
        localize_launch,
        camera_launch,
        detection_launch
    ])
