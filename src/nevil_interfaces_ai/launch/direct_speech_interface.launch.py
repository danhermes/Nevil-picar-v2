#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the speech interface using direct paths."""
    
    # Get the absolute path to the source directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(os.path.dirname(current_dir))
    nevil_interfaces_ai_dir = os.path.join(src_dir, 'nevil_interfaces_ai', 'nevil_interfaces_ai')
    
    # Print paths for debugging
    print(f"Current directory: {current_dir}")
    print(f"Source directory: {src_dir}")
    print(f"Nevil interfaces AI directory: {nevil_interfaces_ai_dir}")
    
    # Verify files exist
    speech_recognition_path = os.path.join(nevil_interfaces_ai_dir, 'speech_recognition_node.py')
    speech_synthesis_path = os.path.join(nevil_interfaces_ai_dir, 'speech_synthesis_node.py')
    dialog_manager_path = os.path.join(nevil_interfaces_ai_dir, 'dialog_manager_node.py')
    
    print(f"Speech recognition path: {speech_recognition_path}, exists: {os.path.exists(speech_recognition_path)}")
    print(f"Speech synthesis path: {speech_synthesis_path}, exists: {os.path.exists(speech_synthesis_path)}")
    print(f"Dialog manager path: {dialog_manager_path}, exists: {os.path.exists(dialog_manager_path)}")
    
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
    
    # Add speech recognition node
    ld.add_action(ExecuteProcess(
        cmd=['python3', speech_recognition_path],
        name='speech_recognition_node',
        output='screen'
    ))
    
    # Add speech synthesis node
    ld.add_action(ExecuteProcess(
        cmd=['python3', speech_synthesis_path],
        name='speech_synthesis_node',
        output='screen'
    ))
    
    # Add dialog manager node
    ld.add_action(ExecuteProcess(
        cmd=['python3', dialog_manager_path],
        name='dialog_manager_node',
        output='screen'
    ))
    
    return ld