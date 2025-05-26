#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI

def create_square_path(side_length=0.5, start_x=0.0, start_y=0.0):
    """
    Create a square path with the given side length.
    
    Args:
        side_length: Length of each side of the square in meters
        start_x: X coordinate of the starting point
        start_y: Y coordinate of the starting point
        
    Returns:
        A list of PoseStamped messages representing the path
    """
    path = []
    
    # Define the four corners of the square
    corners = [
        (start_x, start_y),
        (start_x + side_length, start_y),
        (start_x + side_length, start_y + side_length),
        (start_x, start_y + side_length),
        (start_x, start_y)  # Return to the starting point
    ]
    
    # Create a PoseStamped for each corner
    for i, (x, y) in enumerate(corners):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Calculate orientation (yaw) based on the direction to the next corner
        if i < len(corners) - 1:
            next_x, next_y = corners[i + 1]
            dx = next_x - x
            dy = next_y - y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0
        
        # Convert yaw to quaternion (simplified, only setting w and z for 2D rotation)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        path.append(pose)
    
    return path

def create_figure_eight_path(radius=0.5, center_x=0.0, center_y=0.0, num_points=20):
    """
    Create a figure-eight path with the given radius.
    
    Args:
        radius: Radius of each circle in the figure-eight in meters
        center_x: X coordinate of the center of the figure-eight
        center_y: Y coordinate of the center of the figure-eight
        num_points: Number of points to generate for each circle
        
    Returns:
        A list of PoseStamped messages representing the path
    """
    path = []
    
    # Generate points for the first circle
    for i in range(num_points):
        angle = 2.0 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Calculate orientation (yaw) based on the tangent to the circle
        yaw = angle + math.pi / 2.0
        
        # Convert yaw to quaternion (simplified, only setting w and z for 2D rotation)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        path.append(pose)
    
    # Generate points for the second circle
    for i in range(num_points):
        angle = 2.0 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle + math.pi)
        y = center_y - radius * math.sin(angle)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Calculate orientation (yaw) based on the tangent to the circle
        yaw = angle + math.pi / 2.0
        
        # Convert yaw to quaternion (simplified, only setting w and z for 2D rotation)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        path.append(pose)
    
    return path

def main():
    """
    Example script demonstrating complex navigation sequences with the Nevil Navigation API.
    
    This script shows how to use the API to perform complex navigation sequences
    such as following a square path and a figure-eight path.
    """
    # Initialize ROS2
    rclpy.init()
    
    # Create a node
    node = rclpy.create_node('complex_navigation_example')
    
    # Create the API instance
    api = NevilNavigationAPI(node)
    
    # Create a publisher for the path visualization
    path_publisher = node.create_publisher(
        Path,
        '/planned_path',
        10
    )
    
    try:
        # Wait for services and action servers to be available
        node.get_logger().info('Waiting for services and action servers...')
        time.sleep(2.0)
        
        # Perform complex navigation sequences
        node.get_logger().info('Starting complex navigation demonstration')
        
        # Square path
        node.get_logger().info('Executing square path')
        square_path = create_square_path(side_length=0.5)
        
        # Publish the path for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = node.get_clock().now().to_msg()
        path_msg.poses = square_path
        path_publisher.publish(path_msg)
        
        # Follow the square path
        for i, pose in enumerate(square_path):
            node.get_logger().info(f'Moving to waypoint {i+1}/{len(square_path)}')
            
            # Convert the pose to a NavigateToPoint goal
            goal_msg = api.navigate_client.get_goal_type()()
            goal_msg.target_pose = pose
            goal_msg.linear_velocity = 0.2
            goal_msg.angular_velocity = 0.5
            goal_msg.avoid_obstacles = True
            goal_msg.goal_tolerance = 0.05
            
            # Send the goal
            future = api.navigate_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                node.get_logger().error('Goal rejected')
                continue
            
            # Wait for the result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            
            result = result_future.result().result
            
            if result.success:
                node.get_logger().info(f'Waypoint {i+1} reached successfully')
            else:
                node.get_logger().warn(f'Failed to reach waypoint {i+1}: {result.message}')
        
        # Figure-eight path
        node.get_logger().info('Executing figure-eight path')
        figure_eight_path = create_figure_eight_path(radius=0.3)
        
        # Publish the path for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = node.get_clock().now().to_msg()
        path_msg.poses = figure_eight_path
        path_publisher.publish(path_msg)
        
        # Follow the figure-eight path
        for i, pose in enumerate(figure_eight_path):
            node.get_logger().info(f'Moving to waypoint {i+1}/{len(figure_eight_path)}')
            
            # Convert the pose to a NavigateToPoint goal
            goal_msg = api.navigate_client.get_goal_type()()
            goal_msg.target_pose = pose
            goal_msg.linear_velocity = 0.2
            goal_msg.angular_velocity = 0.5
            goal_msg.avoid_obstacles = True
            goal_msg.goal_tolerance = 0.05
            
            # Send the goal
            future = api.navigate_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                node.get_logger().error('Goal rejected')
                continue
            
            # Wait for the result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            
            result = result_future.result().result
            
            if result.success:
                node.get_logger().info(f'Waypoint {i+1} reached successfully')
            else:
                node.get_logger().warn(f'Failed to reach waypoint {i+1}: {result.message}')
        
        node.get_logger().info('Complex navigation demonstration completed')
    
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