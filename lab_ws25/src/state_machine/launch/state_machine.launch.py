"""
State Machine Launch File
Launches the state machine node for dynamic raceline switching
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for state machine
    """
    
    # Declare launch arguments
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
    
    # State machine node
    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='state_machine',
        output='screen',
        parameters=[{
            'max_raceline': LaunchConfiguration('max_raceline'),
            'switch_period_sec': LaunchConfiguration('switch_period_sec'),
            'publish_period_sec': LaunchConfiguration('publish_period_sec'),
            'selected_topic': LaunchConfiguration('selected_topic')
        }]
    )
    
    return LaunchDescription([
        max_raceline_arg,
        switch_period_arg,
        publish_period_arg,
        selected_topic_arg,
        state_machine_node
    ])
