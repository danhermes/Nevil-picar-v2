from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nevil_interfaces_ai',
            executable='ai_interface_node.py',
            name='ai_interface',
            output='screen',
        ),
    ])
