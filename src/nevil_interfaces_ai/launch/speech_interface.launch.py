#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the speech interface."""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('nevil_interfaces_ai')
    
    # Declare launch arguments
    use_online_recognition = LaunchConfiguration('use_online_recognition', default='true')
    language = LaunchConfiguration('language', default='en')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_online_recognition',
        default_value='true',
        description='Whether to use online speech recognition'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'language',
        default_value='en',
        description='Language code for speech recognition'
    ))
    
    # Get the source directory path
    src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # Add speech recognition node
    # Using ExecuteProcess instead of Node to avoid executable path issues
    ld.add_action(ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'nevil_interfaces_ai', 'speech_recognition_node.py')],
        name='speech_recognition_node',
        output='screen'
    ))
    
    # Add speech synthesis node
    # Using ExecuteProcess instead of Node to avoid executable path issues
    ld.add_action(ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'nevil_interfaces_ai', 'speech_synthesis_node.py')],
        name='speech_synthesis_node',
        output='screen'
    ))
    
    # Add dialog manager node
    # Using ExecuteProcess instead of Node to avoid executable path issues
    ld.add_action(ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'nevil_interfaces_ai', 'dialog_manager_node.py')],
        name='dialog_manager_node',
        output='screen'
    ))
    
    return ld