#!/usr/bin/env python3

print("*" * 80)
print("LOADING MODIFIED FULL_SYSTEM.LAUNCH.PY")
print("*" * 80)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """
    Launch setup function for the full system
    """
    # Add debug logging
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger('nevil_launch')
    logger.debug("Starting launch_setup function")
    
    # Get package directories
    logger.debug("Getting package directories")
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    logger.debug(f"nevil_bringup_dir: {nevil_bringup_dir}")
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    nevil_realtime_dir = get_package_share_directory('nevil_realtime')
    
    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_sim = use_sim.lower() in ['true', 't', 'yes', 'y', '1']
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # Create actions list
    actions = []
    
    # Include the core system launch file
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_core_dir, 'launch', 'nevil_core.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': LaunchConfiguration('config_file')
        }.items()
    )
    actions.append(core_launch)
    
    # Include the navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_navigation_dir, 'launch', 'nevil_navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': LaunchConfiguration('config_file')
        }.items()
    )
    actions.append(navigation_launch)
    
    # Include the perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': LaunchConfiguration('config_file')
        }.items()
    )
    actions.append(perception_launch)
    
    # Include the AI interfaces launch file
    ai_interfaces_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_interfaces_ai_dir, 'launch', 'nevil_interfaces_ai.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': LaunchConfiguration('config_file')
        }.items()
    )
    actions.append(ai_interfaces_launch)
    
    # Include the real-time components launch file
    if not use_sim:
        realtime_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_realtime_dir, 'launch', 'nevil_realtime.launch.py')
            ]),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file')
            }.items()
        )
        actions.append(realtime_launch)
    
    # Include the simulation launch file if using simulation
    if use_sim:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nevil_simulation'), 
                             'launch', 'nevil_simulation.launch.py')
            ]),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file')
            }.items()
        )
        actions.append(simulation_launch)
    
    # Add system monitor node - using cmd instead of executable
    logger.debug("Setting up system_monitor node")
    system_monitor_script = os.path.join(nevil_bringup_dir, 'scripts', 'system_monitor.py')
    logger.debug(f"system_monitor_script path: {system_monitor_script}")
    logger.debug(f"Does script exist? {os.path.exists(system_monitor_script)}")
    
    # Check if the script is executable
    is_executable = os.access(system_monitor_script, os.X_OK)
    logger.debug(f"Is script executable? {is_executable}")
    
    logger.debug("Creating Node with executable parameter")
    system_monitor_node = Node(
        package='nevil_bringup',
        name='system_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file}
        ],
        executable='system_monitor_wrapper'
    )
    logger.debug("Node created successfully")
    actions.append(system_monitor_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for the full Nevil-picar v2.0 system
    """
    # Add debug logging
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger('nevil_launch')
    logger.debug("Starting generate_launch_description function")
    
    # Include common launch file
    logger.debug("Including common launch file")
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_bringup'),
                'launch',
                'common.launch.py'
            ])
        ])
    )
    logger.debug("Common launch file included")
    
    # Create the launch description
    ld = LaunchDescription([common_launch])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld