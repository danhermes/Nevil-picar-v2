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
    
    # use_hardware_bridge_arg = DeclareLaunchArgument(
    #     'use_hardware_bridge',
    #     default_value='true',
    #     description='Enable hardware bridge for actual movement'
    # )
    
    # Navigation node (Python)
    navigation_node = Node(
        package='nevil_navigation',
        executable='navigation_node.py',
        name='navigation_node',
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
        #use_hardware_bridge_arg,
        navigation_node
    ])