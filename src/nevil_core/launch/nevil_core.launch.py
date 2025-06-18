from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the Nevil-picar v2.0 core system
    
    This launch file starts the core system nodes with appropriate parameters
    and priorities.
    """
    # Launch arguments
    system_mode_arg = DeclareLaunchArgument(
        'system_mode',
        default_value='active',
        description='Initial system mode (standby, manual, autonomous)'
    )
    
    # Nodes
    system_manager_node = Node(
        package='nevil_core',
        executable='system_manager.py',
        name='system_manager',
        output='screen',
        parameters=[
            {'system_mode': LaunchConfiguration('system_mode')}
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        system_mode_arg,
        system_manager_node
    ])