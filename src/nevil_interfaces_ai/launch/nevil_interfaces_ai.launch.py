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
    """
    Generate launch description for Nevil AI interfaces with action execution.
    
    This launch file includes:
    - Speech recognition and synthesis
    - AI interface with action execution
    - Navigation system for action execution
    - Motion control for hardware interface
    """
    # Create launch description
    ld = LaunchDescription()

    # Include the speech interface launch file which now includes:
    # - Speech recognition node
    # - Speech synthesis node
    # - AI interface node (with action execution)
    # - Navigation node (with action subscription)
    # - Motion control node
    speech_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nevil_interfaces_ai'), 'launch', 'speech_interface.launch.py')
        ])
    )
    ld.add_action(speech_interface_launch)

    return ld