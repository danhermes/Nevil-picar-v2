#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('nevil_interfaces_ai')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add AI interface node
    ld.add_action(Node(
        package='nevil_interfaces_ai',
        executable='ai_interface_node',
        name='ai_interface',
        output='screen',
    ))
    
    # Include the speech interface launch file
    speech_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'speech_interface.launch.py')
        ])
    )
    ld.add_action(speech_interface_launch)
    
    return ld
