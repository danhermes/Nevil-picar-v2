#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI

def main():
    """
    Example script demonstrating basic movement commands with the Nevil Navigation API.
    
    This script shows how to use the API to perform basic movement operations
    such as moving forward, backward, turning, and stopping.
    """
    # Initialize ROS2
    rclpy.init()
    
    # Create a node
    node = rclpy.create_node('basic_movement_example')
    
    # Create the API instance
    api = NevilNavigationAPI(node)
    
    try:
        # Wait for services and action servers to be available
        node.get_logger().info('Waiting for services and action servers...')
        time.sleep(2.0)
        
        # Perform basic movements
        node.get_logger().info('Starting basic movement demonstration')
        
        # Move forward 30cm at 50% speed
        node.get_logger().info('Moving forward 30cm at 50% speed')
        api.move_forward_this_way(30, 0.5)
        time.sleep(1.0)
        
        # Move backward 20cm at 30% speed
        node.get_logger().info('Moving backward 20cm at 30% speed')
        api.move_backward_this_way(20, 0.3)
        time.sleep(1.0)
        
        # Turn left
        node.get_logger().info('Turning left')
        api.turn_left()
        time.sleep(1.0)
        
        # Turn right
        node.get_logger().info('Turning right')
        api.turn_right()
        time.sleep(1.0)
        
        # Set direction servo angle
        node.get_logger().info('Setting direction servo angle to 30 degrees')
        api.set_dir_servo_angle(30)
        time.sleep(1.0)
        
        # Reset direction servo angle
        node.get_logger().info('Resetting direction servo angle to 0 degrees')
        api.set_dir_servo_angle(0)
        time.sleep(1.0)
        
        # Stop
        node.get_logger().info('Stopping')
        api.stop()
        
        node.get_logger().info('Basic movement demonstration completed')
    
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        # Clean up
        api.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()