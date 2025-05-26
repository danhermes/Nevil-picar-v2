#!/usr/bin/env python3

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
    Launch setup function for integration testing
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    nevil_testing_dir = get_package_share_directory('nevil_testing')
    
    # Get launch configurations
    test_type = LaunchConfiguration('test_type').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_sim = use_sim.lower() in ['true', 't', 'yes', 'y', '1']
    
    # Create actions list
    actions = []
    
    # Include the appropriate system launch file based on test type
    if test_type == 'full':
        # Launch the full system in the appropriate mode
        if use_sim:
            system_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nevil_bringup_dir, 'launch', 'simulation.launch.py')
                ]),
                launch_arguments={
                    'environment': 'empty',
                    'use_rviz': 'false'
                }.items()
            )
        else:
            system_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nevil_bringup_dir, 'launch', 'physical_robot.launch.py')
                ]),
                launch_arguments={
                    'enable_voice': 'false'
                }.items()
            )
        actions.append(system_launch)
    elif test_type == 'minimal':
        # Launch the minimal system
        system_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_bringup_dir, 'launch', 'minimal_system.launch.py')
            ]),
            launch_arguments={
                'use_sim': LaunchConfiguration('use_sim')
            }.items()
        )
        actions.append(system_launch)
    elif test_type == 'component':
        # For component testing, we don't launch the full system
        pass
    else:
        # Default to minimal system
        system_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_bringup_dir, 'launch', 'minimal_system.launch.py')
            ]),
            launch_arguments={
                'use_sim': LaunchConfiguration('use_sim')
            }.items()
        )
        actions.append(system_launch)
    
    # Launch the test runner
    test_runner_node = Node(
        package='nevil_testing',
        executable='test_runner.py',
        name='test_runner',
        output='screen',
        parameters=[
            {'test_type': test_type},
            {'use_sim': use_sim},
            {'test_suite': LaunchConfiguration('test_suite')},
            {'output_dir': LaunchConfiguration('output_dir')}
        ]
    )
    actions.append(test_runner_node)
    
    # Launch specific test nodes based on test type
    if test_type == 'integration':
        # Launch integration test nodes
        integration_test_node = Node(
            package='nevil_testing',
            executable='run_integration_tests.py',
            name='integration_tests',
            output='screen',
            parameters=[
                {'test_suite': LaunchConfiguration('test_suite')},
                {'use_sim': use_sim}
            ]
        )
        actions.append(integration_test_node)
    elif test_type == 'system':
        # Launch system test nodes
        system_test_node = Node(
            package='nevil_testing',
            executable='run_system_tests.py',
            name='system_tests',
            output='screen',
            parameters=[
                {'test_suite': LaunchConfiguration('test_suite')},
                {'use_sim': use_sim}
            ]
        )
        actions.append(system_test_node)
    elif test_type == 'component':
        # Launch component test nodes
        component_test_node = Node(
            package='nevil_testing',
            executable='run_component_tests.py',
            name='component_tests',
            output='screen',
            parameters=[
                {'test_suite': LaunchConfiguration('test_suite')},
                {'use_sim': use_sim}
            ]
        )
        actions.append(component_test_node)
    
    # Add test results collector
    test_results_node = Node(
        package='nevil_bringup',
        executable='test_results_collector.py',
        name='test_results_collector',
        output='screen',
        parameters=[
            {'output_dir': LaunchConfiguration('output_dir')}
        ]
    )
    actions.append(test_results_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for integration testing
    
    This launch file starts the system and runs integration tests.
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    
    # Declare test-specific launch arguments
    test_type_arg = DeclareLaunchArgument(
        'test_type',
        default_value='integration',
        description='Type of test to run (integration, system, component)'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to use simulation (true) or physical hardware (false)'
    )
    
    test_suite_arg = DeclareLaunchArgument(
        'test_suite',
        default_value='all',
        description='Test suite to run (all, navigation, perception, ai)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(nevil_bringup_dir, 'test_results'),
        description='Directory to store test results'
    )
    
    # Create the launch description
    ld = LaunchDescription([
        test_type_arg,
        use_sim_arg,
        test_suite_arg,
        output_dir_arg
    ])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld