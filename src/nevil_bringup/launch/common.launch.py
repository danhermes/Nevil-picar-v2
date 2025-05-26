#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    Common launch file for Nevil-picar v2.0
    
    This launch file contains common launch arguments and utilities
    that are used by other launch files.
    """
    # Get package directories
    nevil_bringup_dir = get_package_share_directory('nevil_bringup')
    
    # Declare common launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(nevil_bringup_dir, 'config', 'default_config.yaml'),
        description='Path to the configuration file'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to use simulation (true) or physical hardware (false)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(config_file_arg)
    ld.add_action(use_sim_arg)
    ld.add_action(log_level_arg)
    
    return ld

def load_yaml_config(context, config_file_path):
    """
    Load YAML configuration file and return as a dictionary
    """
    import yaml
    
    # Get the configuration file path
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # Load the configuration file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    return config

def get_remappings(config, node_name):
    """
    Get topic remappings for a node from the configuration
    """
    if 'remappings' not in config or node_name not in config['remappings']:
        return []
    
    remappings = []
    for source, target in config['remappings'][node_name].items():
        remappings.append((source, target))
    
    return remappings