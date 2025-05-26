from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the Nevil-picar v2.0 navigation system
    
    This launch file starts the navigation nodes with appropriate parameters
    and priorities.
    """
    # Launch arguments
    navigation_mode_arg = DeclareLaunchArgument(
        'navigation_mode',
        default_value='manual',
        description='Navigation mode (manual, autonomous, learning)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    # Nodes
    motion_control_node = Node(
        package='nevil_navigation',
        executable='motion_control_node',
        name='motion_control',
        output='screen',
        parameters=[
            {'max_linear_speed': LaunchConfiguration('max_speed')},
            {'max_angular_speed': 1.0},
            {'safety_enabled': True}
        ]
    )
    
    navigation_node = Node(
        package='nevil_navigation',
        executable='navigation_node.py',
        name='navigation',
        output='screen',
        parameters=[
            {'navigation_mode': LaunchConfiguration('navigation_mode')},
            {'max_speed': LaunchConfiguration('max_speed')}
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        navigation_mode_arg,
        max_speed_arg,
        motion_control_node,
        navigation_node
    ])