#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for the speech interface using source directory paths."""
    
    # Get the absolute path to the source directory
    # Use the actual source directory path
    src_dir = '/home/dan/Documents/Cursor Projects/Nevil-picar-v2'
    
    # Print paths for debugging
    print(f"Source directory: {src_dir}")
    
    # Get the paths to the Python scripts
    speech_recognition_path = os.path.join(src_dir, 'src', 'nevil_interfaces_ai', 'nevil_interfaces_ai', 'speech_recognition_node.py')
    speech_synthesis_path = os.path.join(src_dir, 'src', 'nevil_interfaces_ai', 'nevil_interfaces_ai', 'speech_synthesis_node.py')
    dialog_manager_path = os.path.join(src_dir, 'src', 'nevil_interfaces_ai', 'nevil_interfaces_ai', 'dialog_manager_node.py')
    
    # Verify files exist
    print(f"Speech recognition path: {speech_recognition_path}, exists: {os.path.exists(speech_recognition_path)}")
    print(f"Speech synthesis path: {speech_synthesis_path}, exists: {os.path.exists(speech_synthesis_path)}")
    print(f"Dialog manager path: {dialog_manager_path}, exists: {os.path.exists(dialog_manager_path)}")
    
    # Set up environment variables
    env = os.environ.copy()
    # Add src directory to PYTHONPATH so modules can find each other
    env['PYTHONPATH'] = f"{src_dir}:{env.get('PYTHONPATH', '')}"
    
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
    
    # Add speech recognition node - run as a module
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_recognition_node'],
        cwd=src_dir,  # Set the working directory
        name='speech_recognition_node',
        output='screen',
        env=env  # Pass the environment variables
    ))
    
    # Add speech synthesis node - run as a module
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_synthesis_node'],
        cwd=src_dir,  # Set the working directory
        name='speech_synthesis_node',
        output='screen',
        env=env  # Pass the environment variables
    ))
    
    # Add dialog manager node - run as a module
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.dialog_manager_node'],
        cwd=src_dir,  # Set the working directory
        name='dialog_manager_node',
        output='screen',
        env=env  # Pass the environment variables
    ))
    
    return ld