"""
State Machine Launch File
Launches the state machine node for dynamic raceline switching
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for state machine
    """
    
    # Get package directory
    pkg_state_machine = get_package_share_directory('state_machine')
    
    # Path to config file
    config_file = os.path.join(
        pkg_state_machine,
        'config',
        'state_machine_config.yaml'
    )
    
    # Declare launch arguments
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use config file for parameters'
    )
    
    max_raceline_arg = DeclareLaunchArgument(
        'max_raceline',
        default_value='3',
        description='Maximum number of racelines available'
    )
    
    switch_period_arg = DeclareLaunchArgument(
        'switch_period_sec',
        default_value='5.0',
        description='Period (in seconds) for switching between racelines'
    )
    
    publish_period_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='0.2',
        description='Period (in seconds) for publishing current raceline'
    )
    
    selected_topic_arg = DeclareLaunchArgument(
        'selected_topic',
        default_value='/selected_raceline',
        description='Topic name for publishing selected raceline ID'
    )
    
    raceline_mode_arg = DeclareLaunchArgument(
        'raceline_mode',
        default_value='1',
        description='0=fixed mode (use fixed_raceline_id), 1=cycling mode'
    )
    
    fixed_raceline_id_arg = DeclareLaunchArgument(
        'fixed_raceline_id',
        default_value='2',
        description='Fixed raceline ID to use when raceline_mode=0'
    )
    
    # State machine node
    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='simple_state_machine',  # ✅ Match config file node name
        output='screen',
        parameters=[config_file]  # ✅ Load config file
    )
    
    # Note: keyboard_listener must be run in separate terminal for stdin access
    # If using raceline_mode=2, run in another terminal:
    #   ros2 run state_machine keyboard_listener
    
    return LaunchDescription([
        use_config_arg,
        max_raceline_arg,
        switch_period_arg,
        publish_period_arg,
        selected_topic_arg,
        raceline_mode_arg,
        fixed_raceline_id_arg,
        state_machine_node
    ])
