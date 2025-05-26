#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
    
    # Launch configuration variables
    environment = LaunchConfiguration('environment')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare the launch arguments
    declare_environment_cmd = DeclareLaunchArgument(
        'environment',
        default_value='empty',
        description='Environment to load (empty, random, obstacle_course, maze)')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
    
    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_simulation_dir, 'launch', 'nevil_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true',
            'environment': environment,
            'use_rviz': use_rviz
        }.items()
    )
    
    # Include the core system launch file
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_core_dir, 'launch', 'nevil_core.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true'
        }.items()
    )
    
    # Include the navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_navigation_dir, 'launch', 'nevil_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true'
        }.items()
    )
    
    # Include the perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ),
        launch_arguments={
            'use_sim': 'true'
        }.items()
    )
    
    # Create the hardware abstraction bridge node
    hardware_bridge_node = Node(
        package='nevil_simulation',
        executable='hardware_abstraction_bridge',
        name='hardware_abstraction_bridge',
        output='screen',
        parameters=[{
            'use_sim': True,
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_environment_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add the launch files to the launch description
    ld.add_action(simulation_launch)
    ld.add_action(core_launch)
    ld.add_action(navigation_launch)
    ld.add_action(perception_launch)
    
    # Add the bridge node to the launch description
    ld.add_action(hardware_bridge_node)
    
    return ld