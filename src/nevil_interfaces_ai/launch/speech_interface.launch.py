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
    # Using Node instead of ExecuteProcess to use entry points
    ld.add_action(Node(
        package='nevil_interfaces_ai',
        executable='speech_recognition_node',
        name='speech_recognition_node',
        output='screen'
    ))
    
    # Add speech synthesis node
    # Using Node instead of ExecuteProcess to use entry points
    ld.add_action(Node(
        package='nevil_interfaces_ai',
        executable='speech_synthesis_node',
        name='speech_synthesis_node',
        output='screen'
    ))
    
    # Add AI interface node for action execution
    ld.add_action(Node(
        package='nevil_interfaces_ai',
        executable='ai_interface_node',
        name='ai_interface_node',
        output='screen'
    ))
    
    # Add navigation node for action execution
    ld.add_action(Node(
        package='nevil_navigation',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        parameters=[
            {'navigation_mode': 'autonomous'},
            {'max_speed': 0.5}
        ]
    ))
    
    # Add motion control node
    ld.add_action(Node(
        package='nevil_navigation',
        executable='motion_control_node',
        name='motion_control_node',
        output='screen',
        parameters=[
            {'max_linear_speed': 0.5},
            {'max_angular_speed': 1.0},
            {'safety_enabled': True}
        ]
    ))
    
    # # Add dialog manager node
    # # Using Node instead of ExecuteProcess to use entry points
    # ld.add_action(Node(
    #     package='nevil_interfaces_ai',
    #     executable='dialog_manager_node',
    #     name='dialog_manager_node',
    #     output='screen'
    # ))
    
    return ld