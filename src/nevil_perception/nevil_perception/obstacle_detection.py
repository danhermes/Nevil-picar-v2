#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class ObstacleDetectionNode(Node):
    """
    Obstacle Detection Node for Nevil-picar v2.0
    
    This node is responsible for:
    - Processing ultrasonic sensor data
    - Detecting obstacles in the robot's path
    - Providing distance measurements
    - Triggering obstacle avoidance behaviors
    """
    
    def __init__(self):
        super().__init__('obstacle_detection_node')
        
        # Initialize parameters
        self.declare_parameter('min_distance', 0.2)  # meters
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('sensor_timeout', 0.5)  # seconds
        
        # Create subscribers
        self.ultrasonic_subscription = self.create_subscription(
            Range,
            '/ultrasonic_data',
            self.ultrasonic_callback,
            10
        )
        
        self.system_mode_subscription = self.create_subscription(
            String,
            '/system_mode',
            self.system_mode_callback,
            10
        )
        
        # Create publishers
        self.obstacle_detected_publisher = self.create_publisher(
            Bool,
            '/obstacle_detected',
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables
        self.current_range = 999.0  # Initialize to a large value
        self.last_range_time = self.get_clock().now()
        self.current_mode = 'standby'
        
        self.get_logger().info('Obstacle Detection Node initialized')
        
    def ultrasonic_callback(self, msg):
        """Process ultrasonic sensor data"""
        # Store the current range reading
        self.current_range = msg.range
        self.last_range_time = self.get_clock().now()
        
        # Check if obstacle is detected
        min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        obstacle_detected = self.current_range < min_distance
        
        # Publish obstacle detection status
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_detected_publisher.publish(obstacle_msg)
        
        # If safety is enabled and obstacle is detected, send stop command
        if self.get_parameter('safety_enabled').get_parameter_value().bool_value and obstacle_detected:
            self.get_logger().warn(f'Obstacle detected at {self.current_range:.2f} meters! Sending stop command.')
            self.send_stop_command()
    
    def system_mode_callback(self, msg):
        """Handle system mode changes"""
        self.current_mode = msg.data
        
        # If system is in standby mode, disable safety
        if self.current_mode == 'standby':
            self.set_parameters([rclpy.parameter.Parameter(
                'safety_enabled', 
                rclpy.parameter.Parameter.Type.BOOL, 
                False
            )])
            self.get_logger().info('System in standby mode, disabling obstacle safety')
        else:
            self.set_parameters([rclpy.parameter.Parameter(
                'safety_enabled', 
                rclpy.parameter.Parameter.Type.BOOL, 
                True
            )])
            self.get_logger().info(f'System in {self.current_mode} mode, enabling obstacle safety')
    
    def timer_callback(self):
        """Check for sensor timeout"""
        timeout = self.get_parameter('sensor_timeout').get_parameter_value().double_value
        now = self.get_clock().now()
        elapsed = (now - self.last_range_time).nanoseconds / 1e9  # Convert to seconds
        
        if elapsed > timeout:
            self.get_logger().error(f'Ultrasonic sensor timeout ({elapsed:.2f} seconds)! Sending stop command.')
            self.send_stop_command()
    
    def send_stop_command(self):
        """Emergency stop the robot"""
        cmd_vel = Twist()
        # All velocities set to zero
        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    
    obstacle_detection_node = ObstacleDetectionNode()
    
    try:
        rclpy.spin(obstacle_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_detection_node.send_stop_command()
        obstacle_detection_node.get_logger().info('Shutting down Obstacle Detection Node')
        obstacle_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()