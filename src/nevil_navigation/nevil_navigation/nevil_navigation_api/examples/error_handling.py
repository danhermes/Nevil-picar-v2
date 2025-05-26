#!/usr/bin/env python3

import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from nevil_interfaces.action import NavigateToPoint, PerformBehavior
from nevil_interfaces.msg import SystemStatus

from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI

class ErrorHandlingExample(Node):
    """
    Example node demonstrating error handling and recovery with the Nevil Navigation API.
    
    This node shows how to handle various error conditions that may occur during
    navigation, such as obstacles, timeouts, and hardware failures, and how to
    recover from these errors.
    """
    
    def __init__(self):
        """Initialize the error handling example node."""
        super().__init__('error_handling_example')
        
        # Create the API instance
        self.api = NevilNavigationAPI(self)
        
        # Create subscribers
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            self.system_status_callback,
            10
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Initialize state variables
        self.system_status = None
        self.error_count = 0
        self.recovery_in_progress = False
        self.navigation_active = False
        self.current_goal = None
        
        # Create a lock for thread safety
        self.lock = threading.Lock()
        
        self.get_logger().info('Error handling example node initialized')
    
    def system_status_callback(self, msg):
        """Process system status updates."""
        with self.lock:
            self.system_status = msg
            
            # Check for errors
            if msg.has_errors and not self.recovery_in_progress:
                self.get_logger().warn(f'System errors detected: {msg.error_codes}')
                self.handle_errors(msg.error_codes, msg.error_messages)
    
    def handle_errors(self, error_codes, error_messages):
        """
        Handle system errors.
        
        Args:
            error_codes: List of error codes
            error_messages: List of error messages
        """
        with self.lock:
            self.error_count += 1
            self.recovery_in_progress = True
        
        self.get_logger().info(f'Starting error recovery (attempt {self.error_count})')
        
        try:
            # Stop the robot first
            self.api.stop()
            
            # Handle different types of errors
            for i, code in enumerate(error_codes):
                message = error_messages[i] if i < len(error_messages) else "Unknown error"
                
                if code == 'OBSTACLE_DETECTED':
                    self.get_logger().info(f'Handling obstacle: {message}')
                    self.recover_from_obstacle()
                elif code == 'MOTOR_FAILURE':
                    self.get_logger().info(f'Handling motor failure: {message}')
                    self.recover_from_motor_failure()
                elif code == 'SENSOR_FAILURE':
                    self.get_logger().info(f'Handling sensor failure: {message}')
                    self.recover_from_sensor_failure()
                elif code == 'NAVIGATION_TIMEOUT':
                    self.get_logger().info(f'Handling navigation timeout: {message}')
                    self.recover_from_navigation_timeout()
                elif code == 'EMERGENCY_STOP':
                    self.get_logger().info(f'Handling emergency stop: {message}')
                    self.recover_from_emergency_stop()
                else:
                    self.get_logger().info(f'Handling unknown error: {code} - {message}')
                    self.recover_from_unknown_error()
            
            self.get_logger().info('Error recovery completed')
        except Exception as e:
            self.get_logger().error(f'Error during recovery: {e}')
        finally:
            with self.lock:
                self.recovery_in_progress = False
    
    def recover_from_obstacle(self):
        """Recover from an obstacle detection error."""
        self.get_logger().info('Attempting to recover from obstacle...')
        
        # Back up a bit
        self.api.move_backward_this_way(20, 0.3)
        time.sleep(1.0)
        
        # Try to find a clear path
        left_clear = self.check_direction(-math.pi/4)  # -45 degrees
        right_clear = self.check_direction(math.pi/4)  # 45 degrees
        
        if left_clear:
            self.get_logger().info('Left is clear, turning left')
            self.api.turn_left()
        elif right_clear:
            self.get_logger().info('Right is clear, turning right')
            self.api.turn_right()
        else:
            self.get_logger().info('Both directions blocked, backing up more')
            self.api.move_backward_this_way(30, 0.3)
        
        self.get_logger().info('Obstacle recovery completed')
    
    def recover_from_motor_failure(self):
        """Recover from a motor failure error."""
        self.get_logger().info('Attempting to recover from motor failure...')
        
        # Stop all motors
        self.api.stop()
        time.sleep(1.0)
        
        # Try to reset the motors by sending a zero command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(1.0)
        
        # Try a small movement to test if the motors are working
        cmd.linear.x = 0.1
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
        
        # Stop again
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('Motor recovery completed')
    
    def recover_from_sensor_failure(self):
        """Recover from a sensor failure error."""
        self.get_logger().info('Attempting to recover from sensor failure...')
        
        # Stop the robot
        self.api.stop()
        time.sleep(1.0)
        
        # In a real implementation, this would attempt to reset the sensors
        # or switch to alternative sensors
        
        self.get_logger().info('Sensor recovery completed')
    
    def recover_from_navigation_timeout(self):
        """Recover from a navigation timeout error."""
        self.get_logger().info('Attempting to recover from navigation timeout...')
        
        # Stop the robot
        self.api.stop()
        time.sleep(1.0)
        
        # If we have a current goal, try to replan
        if self.current_goal is not None:
            self.get_logger().info('Replanning path to goal')
            
            # In a real implementation, this would replan the path
            # For now, just retry the navigation
            self.navigate_to_goal(self.current_goal)
        
        self.get_logger().info('Navigation timeout recovery completed')
    
    def recover_from_emergency_stop(self):
        """Recover from an emergency stop error."""
        self.get_logger().info('Attempting to recover from emergency stop...')
        
        # Wait for the emergency condition to clear
        time.sleep(3.0)
        
        # Check if it's safe to continue
        if self.api.get_distance() > 0.5:  # 50cm
            self.get_logger().info('Emergency condition cleared, resuming operation')
        else:
            self.get_logger().warn('Emergency condition still present, cannot resume')
        
        self.get_logger().info('Emergency stop recovery completed')
    
    def recover_from_unknown_error(self):
        """Recover from an unknown error."""
        self.get_logger().info('Attempting to recover from unknown error...')
        
        # Stop the robot
        self.api.stop()
        time.sleep(1.0)
        
        # Try a basic reset sequence
        self.api.set_dir_servo_angle(0)
        self.api.set_cam_pan_angle(0)
        self.api.set_cam_tilt_angle(0)
        
        self.get_logger().info('Unknown error recovery completed')
    
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
    
    def navigate_to_goal(self, goal_pose):
        """
        Navigate to a goal pose with error handling.
        
        Args:
            goal_pose: The goal pose to navigate to
            
        Returns:
            True if navigation was successful, False otherwise
        """
        with self.lock:
            self.navigation_active = True
            self.current_goal = goal_pose
        
        self.get_logger().info(f'Navigating to goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}')
        
        try:
            # Create the goal message
            goal_msg = NavigateToPoint.Goal()
            goal_msg.target_pose = goal_pose
            goal_msg.linear_velocity = 0.2
            goal_msg.angular_velocity = 0.5
            goal_msg.avoid_obstacles = True
            goal_msg.goal_tolerance = 0.05
            
            # Send the goal
            future = self.api.navigate_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            # Wait for the result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result().result
            
            if result.success:
                self.get_logger().info('Goal reached successfully')
                return True
            else:
                self.get_logger().warn(f'Failed to reach goal: {result.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error during navigation: {e}')
            return False
        finally:
            with self.lock:
                self.navigation_active = False
    
    def run_demo(self):
        """Run the error handling demonstration."""
        self.get_logger().info('Starting error handling demonstration')
        
        try:
            # Wait for system status
            self.get_logger().info('Waiting for system status...')
            
            start_time = time.time()
            while time.time() - start_time < 5.0:
                if self.system_status is not None:
                    break
                time.sleep(0.1)
            
            if self.system_status is not None:
                self.get_logger().info('System status received')
            else:
                self.get_logger().warn('No system status received')
            
            # Simulate different error scenarios
            self.get_logger().info('Simulating error scenarios')
            
            # Scenario 1: Obstacle detection
            self.get_logger().info('Scenario 1: Obstacle detection')
            self.simulate_error(['OBSTACLE_DETECTED'], ['Obstacle detected at 0.15m'])
            time.sleep(5.0)
            
            # Scenario 2: Motor failure
            self.get_logger().info('Scenario 2: Motor failure')
            self.simulate_error(['MOTOR_FAILURE'], ['Left motor encoder failure'])
            time.sleep(5.0)
            
            # Scenario 3: Multiple errors
            self.get_logger().info('Scenario 3: Multiple errors')
            self.simulate_error(
                ['OBSTACLE_DETECTED', 'SENSOR_FAILURE'],
                ['Obstacle detected at 0.10m', 'Ultrasonic sensor timeout']
            )
            time.sleep(5.0)
            
            # Scenario 4: Navigation with error recovery
            self.get_logger().info('Scenario 4: Navigation with error recovery')
            
            # Create a goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = 1.0
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # Start navigation
            navigation_thread = threading.Thread(
                target=self.navigate_to_goal,
                args=(goal_pose,)
            )
            navigation_thread.start()
            
            # Wait a bit, then simulate an error
            time.sleep(2.0)
            self.simulate_error(['NAVIGATION_TIMEOUT'], ['Path planning timeout'])
            
            # Wait for navigation to complete
            navigation_thread.join()
            
            self.get_logger().info('Error handling demonstration completed')
        
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot
            self.api.stop()
    
    def simulate_error(self, error_codes, error_messages):
        """
        Simulate a system error.
        
        Args:
            error_codes: List of error codes
            error_messages: List of error messages
        """
        # Create a system status message with errors
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mode = 'autonomous'
        status.system_ok = False
        status.has_errors = True
        status.error_codes = error_codes
        status.error_messages = error_messages
        
        # Call the system status callback directly
        self.system_status_callback(status)


def main():
    """Main function."""
    # Initialize ROS2
    rclpy.init()
    
    # Create the example node
    node = ErrorHandlingExample()
    
    # Create an executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.daemon = True
    executor_thread.start()
    
    try:
        # Run the demo
        node.run_demo()
    finally:
        # Clean up
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()