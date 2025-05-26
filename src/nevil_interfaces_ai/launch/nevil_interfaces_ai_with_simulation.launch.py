#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    
    # Create the launch configuration variables
    use_online_recognition = LaunchConfiguration('use_online_recognition')
    use_online_tts = LaunchConfiguration('use_online_tts')
    use_cloud_ai = LaunchConfiguration('use_cloud_ai')
    api_key = LaunchConfiguration('api_key')
    environment = LaunchConfiguration('environment')
    
    # Declare the launch arguments
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
    
    declare_environment_cmd = DeclareLaunchArgument(
        'environment',
        default_value='empty',
        description='Simulation environment to use (empty, maze, obstacle_course)'
    )
    
    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_simulation_dir, 'launch', 'nevil_simulation.launch.py')
        ]),
        launch_arguments={
            'environment': environment
        }.items()
    )
    
    # Include the AI interfaces launch file
    ai_interfaces_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_interfaces_ai_dir, 'launch', 'nevil_interfaces_ai.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'true',
            'use_online_recognition': use_online_recognition,
            'use_online_tts': use_online_tts,
            'use_cloud_ai': use_cloud_ai,
            'api_key': api_key
        }.items()
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_online_recognition_cmd)
    ld.add_action(declare_use_online_tts_cmd)
    ld.add_action(declare_use_cloud_ai_cmd)
    ld.add_action(declare_api_key_cmd)
    ld.add_action(declare_environment_cmd)
    
    # Add the included launch files
    ld.add_action(simulation_launch)
    ld.add_action(ai_interfaces_launch)
    
    return ld