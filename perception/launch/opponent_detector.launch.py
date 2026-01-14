"""
F1Tenth Opponent Detector Launch File
Launches the opponent car detection node with ZED camera
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for YOLO detection system
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('perception')
    
    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'opponent_detector_params.yaml')
    
    # Verify config file exists
    if not os.path.exists(default_config):
        raise FileNotFoundError(
            f"Configuration file not found: {default_config}\n"
            f"Please ensure opponent_detector_params.yaml exists in perception/config/"
        )
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the opponent detector parameters YAML file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # Opponent Detector Node
    opponent_detector_node = Node(
        package='perception',
        executable='opponent_detector',
        name='opponent_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            'F1Tenth Opponent Car Detection System\n',
            '=' * 70, '\n',
            'Configuration file: ', LaunchConfiguration('config_file'), '\n',
            'Use sim time: ', LaunchConfiguration('use_sim_time'), '\n',
            'Log level: ', LaunchConfiguration('log_level'), '\n',
            '=' * 70, '\n',
            'Waiting for ZED camera images...\n',
            '=' * 70
        ]
    )
    
    return LaunchDescription([
        # Environment variables
        SetEnvironmentVariable('LD_PRELOAD', '/usr/lib/aarch64-linux-gnu/libgomp.so.1'),
        
        # Arguments
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        
        # Nodes
        startup_msg,
        opponent_detector_node
    ])
