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
    Launch setup function for the minimal system
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
    
    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_sim = use_sim.lower() in ['true', 't', 'yes', 'y', '1']
    
    # Set config file to minimal config if not specified
    config_file = LaunchConfiguration('config_file').perform(context)
    if config_file == os.path.join(nevil_bringup_dir, 'config', 'default_config.yaml'):
        config_file = os.path.join(nevil_bringup_dir, 'config', 'minimal_config.yaml')
    
    # Create actions list
    actions = []
    
    # Include the core system launch file (essential)
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_core_dir, 'launch', 'nevil_core.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': config_file
        }.items()
    )
    actions.append(core_launch)
    
    # Include the navigation launch file (essential)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_navigation_dir, 'launch', 'nevil_navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': config_file
        }.items()
    )
    actions.append(navigation_launch)
    
    # Include the perception launch file (essential for obstacle detection)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': config_file,
            'enable_detection': 'true',
            'enable_object_recognition': 'false',
            'enable_lane_detection': 'false'
        }.items()
    )
    actions.append(perception_launch)
    
    # Include the simulation launch file if using simulation
    if use_sim:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nevil_simulation'), 
                             'launch', 'nevil_simulation.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file,
                'use_rviz': 'false',  # Disable RViz for minimal system
                'environment': 'empty'  # Use empty environment for minimal system
            }.items()
        )
        actions.append(simulation_launch)
    
    # Add minimal system monitor node
    system_monitor_node = Node(
        package='nevil_bringup',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'minimal_mode': True}
        ]
    )
    actions.append(system_monitor_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for the minimal Nevil-picar v2.0 system
    
    This launch file starts only the essential components needed for basic operation.
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
    
    # Create the launch description
    ld = LaunchDescription([common_launch])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld