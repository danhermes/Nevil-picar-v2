#!/usr/bin/env python3

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_with_priority(context, *args, **kwargs):
    """Launch nodes with real-time priority using chrt."""
    
    # Get launch configurations
    enable_rt = LaunchConfiguration('enable_rt').perform(context)
    enable_rt = enable_rt.lower() in ['true', 't', 'yes', 'y', '1']
    
    isolated_core = LaunchConfiguration('isolated_core').perform(context)
    isolated_core = int(isolated_core) if isolated_core.isdigit() else -1
    
    # Get hardware backend configuration
    hardware_backend = LaunchConfiguration('hardware_backend').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_sim = use_sim.lower() in ['true', 't', 'yes', 'y', '1']
    
    # Define nodes with their priorities
    nodes = [
        {
            'package': 'nevil_realtime',
            'executable': 'rt_motor_control_node',
            'name': 'rt_motor_control',
            'priority': 90,
            'cpu': isolated_core if isolated_core >= 0 else None,
            'parameters': [
                {'control_rate': 50.0},
                {'max_linear_speed': 0.5},
                {'max_angular_speed': 1.5},
                {'emergency_stop_distance': 0.2}
            ]
        },
        {
            'package': 'nevil_realtime',
            'executable': 'rt_sensor_node',
            'name': 'rt_sensor',
            'priority': 85,
            'cpu': isolated_core if isolated_core >= 0 else None,
            'parameters': [
                {'update_rate': 50.0},
                {'filter_window_size': 5}
            ]
        }#,
        # {
        #     'package': 'nevil_realtime',
        #     'executable': 'rt_config_manager',
        #     'name': 'rt_config_manager',
        #     'priority': 40,
        #     'cpu': None,
        #     'parameters': []
        # },
        # {
        #     'package': 'nevil_realtime',
        #     'executable': 'hardware_bridge_node.py',
        #     'name': 'hardware_bridge',
        #     'priority': 80,
        #     'cpu': isolated_core if isolated_core >= 0 else None,
        #     'parameters': [
        #         {'hardware_backend': hardware_backend},
        #         {'use_sim': use_sim}
        #     ]
        # }
    ]
    
    # Create launch actions
    actions = []
    
    # Get config file path
    config_path = os.path.join(
        get_package_share_directory('nevil_realtime'),
        'config',
        'rt_config.yaml'
    )
    
    # Add config file to parameters for all nodes
    for node in nodes:
        node['parameters'].append(config_path)
    
    # Launch nodes with appropriate priority
    for node in nodes:
        if enable_rt:
            # Launch with real-time priority using chrt
            # Use the wrapper scripts directly instead of ros2 run
            from ament_index_python.packages import get_package_prefix
            package_prefix = get_package_prefix(node['package'])
            executable_path = os.path.join(package_prefix, 'lib', node['package'], node['executable'])
            
            cmd = [executable_path, '--ros-args']
            cmd.extend(['-r', '__node:=' + node['name']])

            for param in node['parameters']:
                if isinstance(param, dict):
                    for key, value in param.items():
                        cmd.extend(['--param', f'{key}:={value}'])
                else:
                    cmd.extend(['--params-file', param])

            if node['cpu'] is not None:
                cmd = ['taskset', '-c', str(node['cpu'])] + cmd

            # Create and add the process
            process = ExecuteProcess(
                cmd=cmd,
                output='screen',
                shell=False
            )
            actions.append(process)
        else:
            # Launch without real-time priority using standard ROS2 node
            ros_node = Node(
                package=node['package'],
                executable=node['executable'],
                name=node['name'],
                parameters=node['parameters'],
                output='screen'
            )
            actions.append(ros_node)
    
    return actions


def generate_launch_description():
    """Generate launch description for Nevil real-time components."""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_rt',
            default_value='true',
            description='Enable real-time features'
        ),
        
        DeclareLaunchArgument(
            'isolated_core',
            default_value='-1',
            description='CPU core to isolate for real-time tasks (-1 to disable)'
        ),
        
        DeclareLaunchArgument(
            'hardware_backend',
            default_value='auto',
            description='Hardware backend to use (auto, physical, simulation)'
        ),
        
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Force simulation mode'
        ),
        
        # Launch nodes with real-time priority
        OpaqueFunction(function=launch_with_priority)
    ])