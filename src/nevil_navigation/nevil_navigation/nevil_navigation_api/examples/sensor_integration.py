#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Image
from std_msgs.msg import Bool

from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI
from nevil_navigation.nevil_navigation_api.rt_integration import RTContext

class SensorIntegrationExample(Node):
    """
    Example node demonstrating sensor integration with the Nevil Navigation API.
    
    This node shows how to use the API to integrate with various sensors,
    including ultrasonic sensors, cameras, and IMUs, and how to use this
    sensor data to control the robot's behavior.
    """
    
    def __init__(self):
        """Initialize the sensor integration example node."""
        super().__init__('sensor_integration_example')
        
        # Create the API instance
        self.api = NevilNavigationAPI(self)
        
        # Create QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for sensor data
        self.ultrasonic_sub = self.create_subscription(
            Range,
            '/ultrasonic_data',
            self.ultrasonic_callback,
            qos_profile=self.sensor_qos
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos_profile=self.sensor_qos
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create timers
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        # Initialize state variables
        self.current_distance = float('inf')
        self.has_camera_data = False
        self.obstacle_detected = False
        self.running = True
        
        # Create a real-time context for sensor processing
        self.rt_context = RTContext(self)
        
        self.get_logger().info('Sensor integration example node initialized')
    
    def ultrasonic_callback(self, msg):
        """
        Process ultrasonic sensor data.
        
        This callback runs in a real-time context to ensure timely
        processing of obstacle detection data.
        """
        # Use the real-time context to process the data
        self.rt_context.run_in_rt_context(self._process_ultrasonic_data, msg)
    
    def _process_ultrasonic_data(self, msg):
        """Process ultrasonic sensor data in a real-time context."""
        # Store the distance
        self.current_distance = msg.range
        
        # Check for obstacles
        if msg.range < 0.3:  # 30cm
            if not self.obstacle_detected:
                self.get_logger().warn(f'Obstacle detected at {msg.range:.2f}m')
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info(f'Obstacle cleared, distance: {msg.range:.2f}m')
                self.obstacle_detected = False
    
    def camera_callback(self, msg):
        """Process camera data."""
        # In a real implementation, this would process the image data
        # For now, just set a flag indicating that we have camera data
        self.has_camera_data = True
    
    def control_callback(self):
        """
        Main control loop.
        
        This callback integrates sensor data to control the robot's behavior.
        """
        if not self.running:
            return
        
        # Check for obstacles
        if self.obstacle_detected:
            # Stop the robot
            self.api.stop()
            
            # Check if we can go around the obstacle
            self.get_logger().info('Checking for a path around the obstacle')
            
            # Check left
            left_clear = self.check_direction(-math.pi/4)  # -45 degrees
            
            # Check right
            right_clear = self.check_direction(math.pi/4)  # 45 degrees
            
            if left_clear and right_clear:
                # Both directions are clear, choose one randomly
                import random
                if random.random() < 0.5:
                    self.get_logger().info('Both directions clear, turning left')
                    self.api.turn_left()
                else:
                    self.get_logger().info('Both directions clear, turning right')
                    self.api.turn_right()
            elif left_clear:
                # Left is clear, turn left
                self.get_logger().info('Left is clear, turning left')
                self.api.turn_left()
            elif right_clear:
                # Right is clear, turn right
                self.get_logger().info('Right is clear, turning right')
                self.api.turn_right()
            else:
                # Both directions are blocked, back up
                self.get_logger().info('Both directions blocked, backing up')
                self.api.move_backward_this_way(30, 0.3)
        else:
            # No obstacles, move forward
            cmd = Twist()
            cmd.linear.x = 0.2  # 20% speed
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
    
    def check_direction(self, direction):
        """
        Check if a direction is clear of obstacles.
        
        Args:
            direction: Direction to check in radians
            
        Returns:
            True if the direction is clear, False otherwise
        """
        # Call the CheckObstacle service
        obstacle_detected, distance, _ = self.api.check_obstacle(direction, 0.5)
        
        # Log the result
        if obstacle_detected:
            self.get_logger().info(f'Obstacle detected at {distance:.2f}m in direction {math.degrees(direction):.1f} degrees')
            return False
        else:
            self.get_logger().info(f'No obstacle detected in direction {math.degrees(direction):.1f} degrees')
            return True
    
    def run_demo(self):
        """Run the sensor integration demonstration."""
        self.get_logger().info('Starting sensor integration demonstration')
        
        try:
            # Wait for sensor data
            self.get_logger().info('Waiting for sensor data...')
            
            start_time = time.time()
            while time.time() - start_time < 5.0:
                if self.current_distance < float('inf') and self.has_camera_data:
                    break
                time.sleep(0.1)
            
            if self.current_distance < float('inf'):
                self.get_logger().info(f'Ultrasonic sensor data received: {self.current_distance:.2f}m')
            else:
                self.get_logger().warn('No ultrasonic sensor data received')
            
            if self.has_camera_data:
                self.get_logger().info('Camera data received')
            else:
                self.get_logger().warn('No camera data received')
            
            # Run the demo for 60 seconds
            self.get_logger().info('Running sensor-based navigation for 60 seconds')
            
            start_time = time.time()
            while time.time() - start_time < 60.0:
                # The control_callback will handle the navigation
                time.sleep(0.1)
            
            # Stop the robot
            self.api.stop()
            
            self.get_logger().info('Sensor integration demonstration completed')
        
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot
            self.api.stop()
            self.running = False


def main():
    """Main function."""
    # Initialize ROS2
    rclpy.init()
    
    # Create the example node
    node = SensorIntegrationExample()
    
    try:
        # Run the demo
        node.run_demo()
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()