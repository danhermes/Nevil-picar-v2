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
    Launch setup function for the simulation-only system
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    
    # Set config file to simulation config if not specified
    config_file = LaunchConfiguration('config_file').perform(context)
    if config_file == os.path.join(nevil_bringup_dir, 'config', 'default_config.yaml'):
        config_file = os.path.join(nevil_bringup_dir, 'config', 'simulation_config.yaml')
    
    # Get environment parameter
    environment = LaunchConfiguration('environment').perform(context)
    
    # Create actions list
    actions = []
    
    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_simulation_dir, 'launch', 'nevil_simulation.launch.py')
        ]),
        launch_arguments={
            'config_file': config_file,
            'environment': environment,
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )
    actions.append(simulation_launch)
    
    # Include the core system launch file
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_core_dir, 'launch', 'nevil_core.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'true',
            'config_file': config_file
        }.items()
    )
    actions.append(core_launch)
    
    # Include the navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_navigation_dir, 'launch', 'nevil_navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'true',
            'config_file': config_file
        }.items()
    )
    actions.append(navigation_launch)
    
    # Include the perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'true',
            'config_file': config_file
        }.items()
    )
    actions.append(perception_launch)
    
    # Include the AI interfaces launch file if enabled
    if LaunchConfiguration('enable_ai').perform(context).lower() in ['true', 't', 'yes', 'y', '1']:
        ai_interfaces_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_interfaces_ai_dir, 'launch', 'nevil_interfaces_ai_with_simulation.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file
            }.items()
        )
        actions.append(ai_interfaces_launch)
    
    # Add simulation bridge node
    simulation_bridge_node = Node(
        package='nevil_simulation',
        executable='hardware_abstraction_bridge',
        name='hardware_abstraction_bridge',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'use_sim': True}
        ]
    )
    actions.append(simulation_bridge_node)
    
    # Add system monitor node
    system_monitor_node = Node(
        package='nevil_bringup',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'simulation_mode': True}
        ]
    )
    actions.append(system_monitor_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for the simulation-only Nevil-picar v2.0 system
    
    This launch file starts the system in simulation mode.
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
    
    # Declare simulation-specific launch arguments
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='obstacle_course',
        description='Environment to load (empty, obstacle_course, maze)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    enable_ai_arg = DeclareLaunchArgument(
        'enable_ai',
        default_value='true',
        description='Whether to enable AI interfaces'
    )
    
    # Create the launch description
    ld = LaunchDescription([
        common_launch,
        environment_arg,
        use_rviz_arg,
        enable_ai_arg
    ])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld