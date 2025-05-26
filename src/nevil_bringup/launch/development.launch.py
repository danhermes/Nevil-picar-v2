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
    Launch setup function for the development mode system
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    nevil_core_dir = get_package_share_directory('nevil_core')
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
    nevil_interfaces_ai_dir = get_package_share_directory('nevil_interfaces_ai')
    nevil_realtime_dir = get_package_share_directory('nevil_realtime')
    nevil_simulation_dir = get_package_share_directory('nevil_simulation')
    nevil_testing_dir = get_package_share_directory('nevil_testing')
    
    # Set config file to development config if not specified
    config_file = LaunchConfiguration('config_file').perform(context)
    if config_file == os.path.join(nevil_bringup_dir, 'config', 'default_config.yaml'):
        config_file = os.path.join(nevil_bringup_dir, 'config', 'development_config.yaml')
    
    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_sim = use_sim.lower() in ['true', 't', 'yes', 'y', '1']
    enable_testing = LaunchConfiguration('enable_testing').perform(context)
    enable_testing = enable_testing.lower() in ['true', 't', 'yes', 'y', '1']
    enable_monitoring = LaunchConfiguration('enable_monitoring').perform(context)
    enable_monitoring = enable_monitoring.lower() in ['true', 't', 'yes', 'y', '1']
    
    # Create actions list
    actions = []
    
    # Include the core system launch file
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
    
    # Include the navigation launch file
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
    
    # Include the perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': config_file
        }.items()
    )
    actions.append(perception_launch)
    
    # Include the AI interfaces launch file
    ai_interfaces_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_interfaces_ai_dir, 'launch', 
                         'nevil_interfaces_ai_with_simulation.launch.py' if use_sim else 'nevil_interfaces_ai.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': config_file
        }.items()
    )
    actions.append(ai_interfaces_launch)
    
    # Include the simulation launch file if using simulation
    if use_sim:
        simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_simulation_dir, 'launch', 'nevil_simulation.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file,
                'environment': LaunchConfiguration('environment'),
                'use_rviz': 'true'
            }.items()
        )
        actions.append(simulation_launch)
    else:
        # Include the real-time components launch file if not using simulation
        realtime_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_realtime_dir, 'launch', 'nevil_realtime.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file,
                'enable_rt': 'false'  # Disable RT for easier debugging
            }.items()
        )
        actions.append(realtime_launch)
    
    # Add development tools
    
    # Parameter tuning UI
    parameter_tuning_node = Node(
        package='nevil_bringup',
        executable='parameter_tuning_ui.py',
        name='parameter_tuning_ui',
        output='screen',
        parameters=[
            {'config_file': config_file}
        ]
    )
    actions.append(parameter_tuning_node)
    
    # Performance profiling
    if enable_monitoring:
        performance_profiling_node = Node(
            package='nevil_bringup',
            executable='performance_profiler.py',
            name='performance_profiler',
            output='screen',
            parameters=[
                {'config_file': config_file},
                {'monitoring_rate': 1.0}  # Hz
            ]
        )
        actions.append(performance_profiling_node)
        
        # Memory monitoring
        memory_monitoring_node = Node(
            package='nevil_bringup',
            executable='memory_monitor.py',
            name='memory_monitor',
            output='screen',
            parameters=[
                {'config_file': config_file},
                {'monitoring_rate': 1.0}  # Hz
            ]
        )
        actions.append(memory_monitoring_node)
    
    # Testing tools if enabled
    if enable_testing:
        test_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nevil_testing_dir, 'launch', 'test_launcher.launch.py')
            ]),
            launch_arguments={
                'config_file': config_file
            }.items()
        )
        actions.append(test_launcher)
    
    # Start RViz with development configuration
    rviz_config_file = os.path.join(nevil_bringup_dir, 'config', 'development_rviz.rviz')
    if os.path.exists(rviz_config_file):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
        actions.append(rviz_node)
    
    # Start rqt with development plugins
    rqt_process = ExecuteProcess(
        cmd=['rqt', '--perspective-file', 
             os.path.join(nevil_bringup_dir, 'config', 'development_perspective.perspective')],
        output='screen'
    )
    actions.append(rqt_process)
    
    # Add system monitor node
    system_monitor_node = Node(
        package='nevil_bringup',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'development_mode': True}
        ]
    )
    actions.append(system_monitor_node)
    
    return actions

def generate_launch_description():
    """
    Generate launch description for the development mode Nevil-picar v2.0 system
    
    This launch file starts the system with debugging tools and development features.
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
    
    # Declare development-specific launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to use simulation (true) or physical hardware (false)'
    )
    
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='empty',
        description='Environment to load (empty, obstacle_course, maze)'
    )
    
    enable_testing_arg = DeclareLaunchArgument(
        'enable_testing',
        default_value='true',
        description='Whether to enable testing tools'
    )
    
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Whether to enable performance monitoring'
    )
    
    # Create the launch description
    ld = LaunchDescription([
        common_launch,
        use_sim_arg,
        environment_arg,
        enable_testing_arg,
        enable_monitoring_arg
    ])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld