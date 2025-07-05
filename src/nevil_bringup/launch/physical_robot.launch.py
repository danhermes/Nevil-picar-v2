#!/usr/bin/env python3

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
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
    nevil_navigation_dir = get_package_share_directory('nevil_navigation')
    nevil_perception_dir = get_package_share_directory('nevil_perception')
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
    
    # Green field validation and cleanup
    green_field_check = ExecuteProcess(
        cmd=['bash', '-c', '''
            echo "🔍 Performing green field validation..."
            
            # Check for existing problematic processes
            CONFLICTS=0
            
            # Check for hardware_init conflicts
            if pgrep -f "nevil_bringup.*hardware_init" >/dev/null 2>&1; then
                echo "⚠️  Found existing hardware_init process"
                CONFLICTS=$((CONFLICTS + 1))
            fi
            
            # Check for system_monitor conflicts
            if pgrep -f "nevil_bringup.*system_monitor" >/dev/null 2>&1; then
                echo "⚠️  Found existing system_monitor process"
                CONFLICTS=$((CONFLICTS + 1))
            fi
            
            # Check for battery_monitor conflicts
            if pgrep -f "nevil_bringup.*battery_monitor" >/dev/null 2>&1; then
                echo "⚠️  Found existing battery_monitor process"
                CONFLICTS=$((CONFLICTS + 1))
            fi
            
            # Check for navigation_node conflicts
            if pgrep -f "navigation_node.py" >/dev/null 2>&1; then
                echo "⚠️  Found existing navigation_node process"
                CONFLICTS=$((CONFLICTS + 1))
            fi
            
            if [ $CONFLICTS -gt 0 ]; then
                echo "🧹 Cleaning up $CONFLICTS conflicting processes..."
                
                # Clean up only the specific conflicting processes
                pkill -f "nevil_bringup.*hardware_init" 2>/dev/null || true
                pkill -f "nevil_bringup.*system_monitor" 2>/dev/null || true
                pkill -f "nevil_bringup.*battery_monitor" 2>/dev/null || true
                pkill -f "navigation_node.py" 2>/dev/null || true
                
                # Wait for cleanup to complete
                sleep 1
                
                # Verify cleanup
                REMAINING=0
                pgrep -f "nevil_bringup.*hardware_init" >/dev/null 2>&1 && REMAINING=$((REMAINING + 1))
                pgrep -f "nevil_bringup.*system_monitor" >/dev/null 2>&1 && REMAINING=$((REMAINING + 1))
                pgrep -f "nevil_bringup.*battery_monitor" >/dev/null 2>&1 && REMAINING=$((REMAINING + 1))
                pgrep -f "navigation_node.py" >/dev/null 2>&1 && REMAINING=$((REMAINING + 1))
                
                if [ $REMAINING -eq 0 ]; then
                    echo "✅ Green field achieved - environment is clean"
                else
                    echo "❌ Warning: $REMAINING processes still running after cleanup"
                fi
            else
                echo "✅ Green field confirmed - no conflicts detected"
            fi
            
            echo "🚀 Ready to start new nodes"
        '''],
        output='screen',
        name='green_field_check'
    )
    actions.append(green_field_check)
    
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
    
    # Add a single navigation node directly instead of including navigation launch
    # This prevents duplicate navigation nodes that cause GPIO conflicts
    navigation_node = Node(
        package='nevil_navigation',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        parameters=[
            {'navigation_mode': 'manual'},
            {'max_speed': 0.5}
        ]
    )
    actions.append(navigation_node)
    
    # Include the perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nevil_perception_dir, 'launch', 'nevil_perception.launch.py')
        ]),
        launch_arguments={
            'use_sim': 'false',
            'config_file': config_file
        }.items()
    )
    actions.append(perception_launch)
    
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
    
    # # Add battery monitor node
    # battery_monitor_node = Node(
    #     package='nevil_bringup',
    #     executable='battery_monitor',
    #     name='battery_monitor',
    #     output='screen',
    #     parameters=[
    #         {'config_file': config_file},
    #         {'low_battery_threshold': 7.2}  # V
    #     ]
    # )
    # actions.append(battery_monitor_node)
    
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