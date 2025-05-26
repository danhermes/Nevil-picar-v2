#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    
    # Create the launch configuration variables
    use_sim = LaunchConfiguration('use_sim')
    use_online_recognition = LaunchConfiguration('use_online_recognition')
    use_online_tts = LaunchConfiguration('use_online_tts')
    use_cloud_ai = LaunchConfiguration('use_cloud_ai')
    api_key = LaunchConfiguration('api_key')
    
    # Declare the launch arguments
    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to use simulation (true) or physical hardware (false)'
    )
    
    declare_use_online_recognition_cmd = DeclareLaunchArgument(
        'use_online_recognition',
        default_value='true',
        description='Whether to use online speech recognition (true) or offline (false)'
    )
    
    declare_use_online_tts_cmd = DeclareLaunchArgument(
        'use_online_tts',
        default_value='false',
        description='Whether to use online text-to-speech (true) or offline (false)'
    )
    
    declare_use_cloud_ai_cmd = DeclareLaunchArgument(
        'use_cloud_ai',
        default_value='true',
        description='Whether to use cloud AI for natural language processing (true) or local (false)'
    )
    
    declare_api_key_cmd = DeclareLaunchArgument(
        'api_key',
        default_value='',
        description='API key for cloud services'
    )
    
    # Create the nodes
    text_command_processor_node = Node(
        package='nevil_interfaces_ai',
        executable='text_command_processor',
        name='text_command_processor',
        output='screen',
        parameters=[{
            'use_cloud_ai': use_cloud_ai,
            'api_key': api_key
        }]
    )
    
    speech_recognition_node = Node(
        package='nevil_interfaces_ai',
        executable='speech_recognition_node',
        name='speech_recognition_node',
        output='screen',
        parameters=[{
            'use_online_recognition': use_online_recognition,
            'online_api': 'google',
            'language': 'en-US',
            'energy_threshold': 300,
            'pause_threshold': 0.8,
            'dynamic_energy_threshold': True,
            'api_key': api_key
        }]
    )
    
    speech_synthesis_node = Node(
        package='nevil_interfaces_ai',
        executable='speech_synthesis_node',
        name='speech_synthesis_node',
        output='screen',
        parameters=[{
            'use_online_tts': use_online_tts,
            'online_service': 'google',
            'voice_id': '',
            'speaking_rate': 1.0,
            'pitch': 1.0,
            'volume': 1.0,
            'api_key': api_key
        }]
    )
    
    dialog_manager_node = Node(
        package='nevil_interfaces_ai',
        executable='dialog_manager_node',
        name='dialog_manager_node',
        output='screen',
        parameters=[{
            'max_context_size': 10,
            'idle_timeout': 60.0,
            'use_cloud_ai': use_cloud_ai,
            'api_key': api_key
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_cmd)
    ld.add_action(declare_use_online_recognition_cmd)
    ld.add_action(declare_use_online_tts_cmd)
    ld.add_action(declare_use_cloud_ai_cmd)
    ld.add_action(declare_api_key_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(text_command_processor_node)
    ld.add_action(speech_recognition_node)
    ld.add_action(speech_synthesis_node)
    ld.add_action(dialog_manager_node)
    
    return ld