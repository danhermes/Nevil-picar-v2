#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    
    # Launch configuration variables
    use_sim = LaunchConfiguration('use_sim')
    environment = LaunchConfiguration('environment')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare the launch arguments
    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (true) or real hardware (false)')
    
    declare_environment_cmd = DeclareLaunchArgument(
        'environment',
        default_value='empty',
        description='Environment to load (empty, random, obstacle_course, maze)')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nevil_simulation_dir, 'config', 'simulation.rviz'),
        description='Absolute path to rviz config file')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
    
    # Create the simulation node
    simulation_node = Node(
        package='nevil_simulation',
        executable='simulation_node',
        name='simulation_node',
        output='screen',
        parameters=[{
            'use_sim': use_sim,
            'default_environment': environment,
            'update_rate': 30.0,
            'physics_update_rate': 100.0,
            'auto_start': True,
            'visualization_enabled': True,
            'robot.mass': 1.5,
            'robot.width': 0.15,
            'robot.length': 0.2,
            'robot.wheel_radius': 0.03,
            'robot.max_linear_velocity': 0.5,
            'robot.max_angular_velocity': 1.0,
        }]
    )
    
    # Create the visualization node
    visualization_node = Node(
        package='nevil_simulation',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[{
            'update_rate': 30.0,
            'show_robot': True,
            'show_sensors': True,
            'show_debug': False,
            'show_camera': True,
            'show_ultrasonic': True,
            'show_trajectories': True,
        }]
    )
    
    # Create the environment generator node
    environment_generator_node = Node(
        package='nevil_simulation',
        executable='environment_generator',
        name='environment_generator',
        output='screen',
        parameters=[{
            'environments_path': os.path.join(nevil_simulation_dir, 'environments'),
            'default_environment': environment,
            'default_size_x': 10.0,
            'default_size_y': 10.0,
            'default_obstacle_count': 5,
            'wall_height': 0.5,
        }]
    )
    
    # Create the simulation manager node
    simulation_manager_node = Node(
        package='nevil_simulation',
        executable='simulation_manager',
        name='simulation_manager',
        output='screen',
        parameters=[{
            'update_rate': 10.0,
            'auto_start': True,
            'default_environment': environment,
            'environments_path': os.path.join(nevil_simulation_dir, 'environments'),
        }]
    )
    
    # Create the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )
    
    # Create the static transform publisher for the map frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_cmd)
    ld.add_action(declare_environment_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(static_transform_publisher)
    ld.add_action(environment_generator_node)
    ld.add_action(simulation_node)
    ld.add_action(visualization_node)
    ld.add_action(simulation_manager_node)
    ld.add_action(rviz_node)
    
    return ld