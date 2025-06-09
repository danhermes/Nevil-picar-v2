#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """
    Launch setup function for the physical robot system
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    nevil_core_dir = get_package_share_directory('nevil_core')
    #nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    #nevil_perception_dir = get_package_share_directory('nevil_perception')
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    nevil_realtime_dir = get_package_share_directory('nevil_realtime')
    
    # Set config file to physical robot config if not specified
    config_file = LaunchConfiguration('config_file').perform(context)
    if config_file == os.path.join(nevil_bringup_dir, 'config', 'default_config.yaml'):
        config_file = os.path.join(nevil_bringup_dir, 'config', 'physical_robot_config.yaml')
    
    # Get launch configurations
    enable_rt = LaunchConfiguration('enable_rt').perform(context)
    enable_voice = LaunchConfiguration('enable_voice').perform(context)
    
    # Create actions list
    actions = []
    
    # Include the core system launch file
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_core_dir, 'launch', 'nevil_core.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'false',
            'config_file': config_file
        }.items()
    )
    actions.append(core_launch)
    
    # # Include the navigation launch file
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(nevil_navigation_dir, 'launch', 'nevil_navigation.launch.py')
    #     ]),
    #     launch_arguments={
    #         'use_sim': 'false',
    #         'config_file': config_file
    #     }.items()
    # )
    # actions.append(navigation_launch)
    
    # # Include the perception launch file
    # perception_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
    #     ]),
    #     launch_arguments={
    #         'use_sim': 'false',
    #         'config_file': config_file
    #     }.items()
    # )
    # actions.append(perception_launch)
    
    # Include the AI interfaces launch file if voice is enabled
    if enable_voice.lower() in ['true', 't', 'yes', 'y', '1']:
        ai_interfaces_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_interfaces_ai_dir, 'launch', 'nevil_interfaces_ai.launch.py')
            ]),
            launch_arguments={
                'use_sim': 'false',
                'config_file': config_file
            }.items()
        )
        actions.append(ai_interfaces_launch)
    
    # Include the real-time components launch file if RT is enabled
    if enable_rt.lower() in ['true', 't', 'yes', 'y', '1']:
        realtime_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_realtime_dir, 'launch', 'nevil_realtime.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file,
                'enable_rt': 'true'
            }.items()
        )
        actions.append(realtime_launch)
    
    # Add hardware initialization node
    hardware_init_node = Node(
        package='nevil_bringup',
        executable='hardware_init',
        name='hardware_init',
        output='screen',
        parameters=[
            {'config_file': config_file}
        ]
    )
    actions.append(hardware_init_node)
    
    # Add system monitor node
    system_monitor_node = Node(
        package='nevil_bringup',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'physical_mode': True}
        ]
    )
    actions.append(system_monitor_node)
    
    # Add battery monitor node
    battery_monitor_node = Node(
        package='nevil_bringup',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'low_battery_threshold': 7.2}  # V
        ]
    )
    actions.append(battery_monitor_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for the physical robot Nevil-picar v2.0 system
    
    This launch file starts the system on the physical robot hardware.
    """
    # Include common launch file
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_bringup'),
                'launch',
                'common.launch.py'
            ])
        ])
    )
    
    # Declare physical robot-specific launch arguments
    enable_rt_arg = DeclareLaunchArgument(
        'enable_rt',
        default_value='true',
        description='Whether to enable real-time features'
    )
    
    enable_voice_arg = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Whether to enable voice interface'
    )
    
    # Create the launch description
    ld = LaunchDescription([
        common_launch,
        enable_rt_arg,
        enable_voice_arg
    ])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld