from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the Nevil-picar v2.0 perception system
    
    This launch file starts the perception nodes with appropriate parameters
    and priorities.
    """
    # Launch arguments
    enable_detection_arg = DeclareLaunchArgument(
        'enable_detection',
        default_value='true',
        description='Enable object detection'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.2',
        description='Minimum distance for obstacle detection in meters'
    )
    
    # Nodes
    camera_vision_node = Node(
        package='nevil_perception',
        executable='camera_vision.py',
        name='camera_vision',
        output='screen',
        parameters=[
            {'enable_detection': LaunchConfiguration('enable_detection')},
            {'detection_threshold': 0.5},
            {'camera_fps': 15}
        ]
    )
    
    obstacle_detection_node = Node(
        package='nevil_perception',
        executable='obstacle_detection.py',
        name='obstacle_detection',
        output='screen',
        parameters=[
            {'min_distance': LaunchConfiguration('min_distance')},
            {'safety_enabled': True},
            {'sensor_timeout': 0.5}
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        enable_detection_arg,
        min_distance_arg,
        camera_vision_node,
        obstacle_detection_node
    ])