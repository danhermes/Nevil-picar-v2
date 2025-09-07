#!/usr/bin/env python3

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """
    Launch setup function for the physical robot system
    """
    # Configure logging to use workspace logs directory
    workspace_log_dir = os.path.join(os.path.expanduser('~'), 'Nevil-picar-v2', 'logs')
    launch_log_dir = os.path.join(workspace_log_dir, 'launch')
    os.makedirs(launch_log_dir, exist_ok=True)
    
    # Set environment variables to fix rcutils_expand_user issues
    os.environ['ROS_LOG_DIR'] = launch_log_dir
    os.environ['HOME'] = os.path.expanduser('~')
    os.environ['USER'] = os.getenv('USER', 'dan')

    
    # Load additional environment variables from .env file
    env_file = os.path.join(os.path.expanduser('~'), 'Nevil-picar-v2', '.env')
    if os.path.exists(env_file):
        with open(env_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    os.environ[key] = value
    
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
    
    # Note: Green field validation now runs synchronously in generate_launch_description()
    # before any nodes are started, preventing the "split personality Nevil" issue
    
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
    
    # Include the perception launch file # TODO: Uncomment this when perception is ready - paused
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
    
    # Add signal handling for proper CTRL+C support
    import signal
    import sys
    
    def signal_handler(sig, frame):
        #print("\n🛑 Shutdown signal received, cleaning up...")
        # Force kill all nevil processes
        import subprocess
        try:
            subprocess.run(['pkill', '-f', 'nevil'], check=False)
            subprocess.run(['pkill', '-f', 'speech_recognition_node'], check=False)
            subprocess.run(['pkill', '-f', 'speech_synthesis_node'], check=False)
            subprocess.run(['pkill', '-f', 'ai_interface_node'], check=False)
        except:
            pass
        # Remove signal handlers to prevent re-entry
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # CRITICAL: Execute green field validation SYNCHRONOUSLY before any nodes start
    print("🔍 Performing comprehensive green field validation...")
    
    # Check for existing problematic processes
    conflicts = 0
    cleanup_needed = False
    
    # Define all process patterns to check
    process_patterns = [
        ("nevil_bringup.*hardware_init", "hardware_init"),
        ("nevil_bringup.*system_monitor", "system_monitor"),
        ("nevil_bringup.*battery_monitor", "battery_monitor"),
        ("navigation_node", "navigation_node"),
        ("ai_interface_node", "ai_interface_node"),
        ("motion_control_node", "motion_control_node"),
        ("speech_recognition_node", "speech_recognition_node"),
        ("speech_synthesis_node", "speech_synthesis_node"),
        ("camera_vision", "camera_vision"),
        ("obstacle_detection", "obstacle_detection"),
        ("system_manager", "system_manager")
    ]
    
    # Check for conflicts
    for pattern, name in process_patterns:
        try:
            result = subprocess.run(['pgrep', '-f', pattern],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print(f"⚠️  Found existing {name} process")
                conflicts += 1
                cleanup_needed = True
        except Exception:
            pass  # pgrep not found or other error, continue
    
    # Perform cleanup if needed
    if cleanup_needed:
        print(f"🧹 Cleaning up {conflicts} conflicting processes...")
        
        # First pass: Standard cleanup
        for pattern, name in process_patterns:
            try:
                subprocess.run(['pkill', '-f', pattern],
                             capture_output=True, check=False)
            except Exception:
                pass  # Continue even if pkill fails
        
        # Wait for cleanup to complete
        import time
        time.sleep(2)
        
        # Second pass: More aggressive cleanup for stubborn processes
        stubborn_patterns = [
            "speech_recognition_node",
            "speech_synthesis_node",
            "ai_interface_node"
        ]
        
        for pattern in stubborn_patterns:
            try:
                # Get PIDs first
                result = subprocess.run(['pgrep', '-f', pattern],
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    pids = result.stdout.strip().split('\n')
                    print(f"🔨 Force killing stubborn {pattern} processes: {pids}")
                    for pid in pids:
                        if pid.strip():
                            subprocess.run(['kill', '-9', pid.strip()],
                                         capture_output=True, check=False)
            except Exception:
                pass
        
        # Final wait
        time.sleep(1)
        
        # Verify cleanup
        remaining = 0
        for pattern, name in process_patterns:
            try:
                result = subprocess.run(['pgrep', '-f', pattern],
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    remaining += 1
            except Exception:
                pass
        
        if remaining == 0:
            print("✅ Green field achieved - environment is clean")
        else:
            print(f"❌ Warning: {remaining} processes still running after cleanup")
            # Show what remains for debugging
            print("🔍 Remaining processes:")
            try:
                subprocess.run(['ps', 'aux'], capture_output=False, check=False)
            except Exception:
                pass
    else:
        print("✅ Green field confirmed - no conflicts detected")
    
    print("🚀 Ready to start new nodes")
    
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
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT','[{time}] {severity} [{name}] {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT','1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT','1'),
        SetEnvironmentVariable('PYTHONUNBUFFERED','1'),
        common_launch,
        enable_rt_arg,
        enable_voice_arg
    ])
    
    # Add the setup function
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld