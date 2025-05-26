#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nevil_testing_dir = get_package_share_directory('nevil_testing')
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    
    # Launch configuration variables
    use_simulation = LaunchConfiguration('use_simulation')
    test_type = LaunchConfiguration('test_type')
    test_package = LaunchConfiguration('test_package')
    test_file = LaunchConfiguration('test_file')
    verbose = LaunchConfiguration('verbose')
    
    # Declare the launch arguments
    declare_use_simulation_cmd = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Whether to use simulation or real hardware'
    )
    
    declare_test_type_cmd = DeclareLaunchArgument(
        'test_type',
        default_value='unit',
        description='Type of tests to run (unit, integration, system, all)'
    )
    
    declare_test_package_cmd = DeclareLaunchArgument(
        'test_package',
        default_value='all',
        description='Package to test (core, navigation, perception, realtime, interfaces_ai, simulation, all)'
    )
    
    declare_test_file_cmd = DeclareLaunchArgument(
        'test_file',
        default_value='',
        description='Specific test file to run (empty for all tests)'
    )
    
    declare_verbose_cmd = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )
    
    # Include the Nevil system launch file with simulation if requested
    include_nevil_system_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_core_dir, 'launch', 'nevil_system.launch.py')
        ),
        condition=UnlessCondition(use_simulation)
    )
    
    # Include the Nevil system with simulation launch file if requested
    include_nevil_system_with_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nevil_simulation_dir, 'launch', 'nevil_system_with_simulation.launch.py')
        ),
        condition=IfCondition(use_simulation)
    )
    
    # Create the test runner node
    test_runner_cmd = Node(
        package='nevil_testing',
        executable='run_all_tests',
        name='test_runner',
        output='screen',
        parameters=[
            {'test_type': test_type},
            {'use_simulation': use_simulation},
            {'verbose': verbose}
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_simulation_cmd)
    ld.add_action(declare_test_type_cmd)
    ld.add_action(declare_test_package_cmd)
    ld.add_action(declare_test_file_cmd)
    ld.add_action(declare_verbose_cmd)
    
    # Add the system launch files
    ld.add_action(include_nevil_system_cmd)
    ld.add_action(include_nevil_system_with_simulation_cmd)
    
    # Add the test runner
    ld.add_action(test_runner_cmd)
    
    return ld