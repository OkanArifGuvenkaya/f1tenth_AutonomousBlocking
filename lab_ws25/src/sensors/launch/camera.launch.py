"""
F1Tenth Camera Publisher Launch File
Launches the ZED camera publisher node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for camera publisher
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('sensors')
    
    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    
    # Verify config file exists
    if not os.path.exists(default_config):
        raise FileNotFoundError(
            f"Configuration file not found: {default_config}\n"
            f"Please ensure camera_params.yaml exists in sensors/config/"
        )
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the camera parameters YAML file'
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
    
    # Camera Publisher Node
    camera_publisher_node = Node(
        package='sensors',
        executable='camera_publisher',
        name='camera_publisher',
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
            'F1Tenth ZED Camera Publisher\n',
            '=' * 70, '\n',
            'Configuration file: ', LaunchConfiguration('config_file'), '\n',
            'Use sim time: ', LaunchConfiguration('use_sim_time'), '\n',
            'Log level: ', LaunchConfiguration('log_level'), '\n',
            '=' * 70, '\n',
            'Starting camera publisher...\n',
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
        camera_publisher_node
    ])
