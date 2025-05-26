#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class NavigationNode(Node):
    """
    Navigation Node for Nevil-picar v2.0
    
    This node is responsible for:
    - Path planning and execution
    - Integrating sensor data for navigation
    - Implementing navigation behaviors
    - Coordinating with obstacle avoidance
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # QoS profile for navigation messages
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize parameters
        self.declare_parameter('navigation_mode', 'manual')  # manual, autonomous, learning
        self.declare_parameter('max_speed', 0.5)  # m/s
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            qos_profile=nav_qos
        )
        
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            qos_profile=nav_qos
        )
        
        # Create subscribers
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.system_mode_subscription = self.create_subscription(
            String,
            '/system_mode',
            self.system_mode_callback,
            10
        )
        
        # Create timers
        self.timer = self.create_timer(0.1, self.navigation_loop)
        
        # State variables
        self.current_goal = None
        self.current_path = None
        self.system_mode = 'standby'
        self.navigation_active = False
        
        self.get_logger().info('Navigation Node initialized')
        
    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.current_goal = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
        # Here we would implement path planning
        # For now, just set navigation active
        if self.system_mode != 'standby':
            self.navigation_active = True
            self.get_logger().info('Starting navigation to goal')
        else:
            self.get_logger().warn('Cannot navigate while in standby mode')
    
    def system_mode_callback(self, msg):
        """Handle system mode changes"""
        self.system_mode = msg.data
        
        if self.system_mode == 'standby':
            self.navigation_active = False
            self.stop_robot()
            self.get_logger().info('Navigation stopped due to standby mode')
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.current_goal is None:
            return
            
        # Here we would implement path following
        # For now, just send a simple command
        cmd = Twist()
        
        # Simple example: move forward at half speed
        nav_mode = self.get_parameter('navigation_mode').get_parameter_value().string_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
        if nav_mode == 'autonomous' and self.navigation_active:
            cmd.linear.x = max_speed / 2.0
            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().debug(f'Sending cmd_vel: linear.x={cmd.linear.x}')
    
    def stop_robot(self):
        """Emergency stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    
    navigation_node = NavigationNode()
    
    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.stop_robot()
        navigation_node.get_logger().info('Shutting down Navigation Node')
        navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()