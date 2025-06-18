from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Main launch file for the Nevil-picar v2.0 system
    
    This launch file includes the launch files from the individual packages
    to start the complete system.
    """
    # Launch arguments
    system_mode_arg = DeclareLaunchArgument(
        'system_mode',
        default_value='active',
        description='Initial system mode (standby, manual, autonomous)'
    )
    
    enable_detection_arg = DeclareLaunchArgument(
        'enable_detection',
        default_value='true',
        description='Enable object detection'
    )
    
    navigation_mode_arg = DeclareLaunchArgument(
        'navigation_mode',
        default_value='manual',
        description='Navigation mode (manual, autonomous, learning)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.2',
        description='Minimum distance for obstacle detection in meters'
    )
    
    # Include launch files from individual packages
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_core'),
                'launch',
                'nevil_core.launch.py'
            ])
        ]),
        launch_arguments={
            'system_mode': LaunchConfiguration('system_mode')
        }.items()
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_navigation'),
                'launch',
                'nevil_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'navigation_mode': LaunchConfiguration('navigation_mode'),
            'max_speed': LaunchConfiguration('max_speed'),
            'use_hardware_bridge': 'true'
        }.items()
    )
    
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_perception'),
                'launch',
                'nevil_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_detection': LaunchConfiguration('enable_detection'),
            'min_distance': LaunchConfiguration('min_distance')
        }.items()
    )
    
    # Real-time motor control launch with hardware bridge
    realtime_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nevil_realtime'),
                'launch',
                'nevil_realtime.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_rt': 'true',
            'hardware_backend': 'auto',
            'use_sim': 'false'
        }.items()
    )
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        system_mode_arg,
        enable_detection_arg,
        navigation_mode_arg,
        max_speed_arg,
        min_distance_arg,
        
        # Launch files
        core_launch,
        navigation_launch,
        perception_launch,
        realtime_launch
    ])