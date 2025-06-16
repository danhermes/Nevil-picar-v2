#!/usr/bin/env python3

import os, sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from nevil_bringup.package_finder import get_python_node_executable_path, get_python_interpreter_executable

def generate_launch_description():
    # Get the package directory (still useful for other paths like sub-launch files)
    # pkg_dir = get_package_share_directory('nevil_interfaces_ai') # You can keep this or not, depending on usage

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
    # This one will likely need the same fix applied to *it*
    speech_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nevil_interfaces_ai'), 'launch', 'speech_interface.launch.py')
        ])
    )
    ld.add_action(speech_interface_launch)

    return ld