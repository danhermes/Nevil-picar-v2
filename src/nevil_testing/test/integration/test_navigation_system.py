#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nevil_interfaces.action import NavigateToPoint
from nevil_interfaces.srv import CheckObstacle
import math
import yaml
import os

from nevil_testing.simulation_test_base import SimulationTestBase

class TestNavigationSystem(SimulationTestBase):
    """
    Integration tests for the navigation system in Nevil-picar v2.0.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Set the simulation environment to empty
        self.set_simulation_environment('empty')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
    
    def test_navigate_to_point_action(self):
        """Test the navigate_to_point action."""
        # Create an action client
        client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to a point 1 meter ahead
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 1.0
        goal_msg.target_pose.pose.position.y = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.1
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=30.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.result_code, 'GOAL_REACHED')
        
        # Verify the final pose is close to the target
        final_pose = result.result.final_pose.pose
        distance = math.sqrt((final_pose.position.x - 1.0)**2 + 
                             (final_pose.position.y - 0.0)**2)
        
        self.assertLessEqual(distance, goal_msg.goal_tolerance)
    
    def test_obstacle_avoidance(self):
        """Test obstacle avoidance during navigation."""
        # Set the simulation environment to obstacle_course
        self.set_simulation_environment('obstacle_course')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
        
        # Create an action client
        client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to a point that requires obstacle avoidance
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 2.0
        goal_msg.target_pose.pose.position.y = 2.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.2
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal with obstacles')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=60.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.result_code, 'GOAL_REACHED')
        
        # Verify the final pose is close to the target
        final_pose = result.result.final_pose.pose
        distance = math.sqrt((final_pose.position.x - 2.0)**2 + 
                             (final_pose.position.y - 2.0)**2)
        
        self.assertLessEqual(distance, goal_msg.goal_tolerance)
    
    def test_check_obstacle_service(self):
        """Test the check_obstacle service."""
        # Set the simulation environment to obstacle_course
        self.set_simulation_environment('obstacle_course')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
        
        # Create a client for the check_obstacle service
        client = self.create_client(CheckObstacle, '/check_obstacle')
        
        # Create a request to check for obstacles in front
        request = CheckObstacle.Request()
        request.direction = 0.0  # Forward
        request.max_distance = 2.0
        request.mode = 'single'
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertIsNotNone(response)
        
        # Create a request to check for obstacles to the left
        request.direction = math.pi / 2  # Left
        
        # Send the request and wait for the response
        response_left = self.wait_for_service_response(client, request)
        
        # Create a request to check for obstacles to the right
        request.direction = -math.pi / 2  # Right
        
        # Send the request and wait for the response
        response_right = self.wait_for_service_response(client, request)
        
        # Verify that at least one of the directions has an obstacle
        self.assertTrue(response.obstacle_detected or 
                       response_left.obstacle_detected or 
                       response_right.obstacle_detected)
    
    def test_navigation_with_continuous_obstacle_checking(self):
        """Test navigation with continuous obstacle checking."""
        # Set the simulation environment to obstacle_course
        self.set_simulation_environment('obstacle_course')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
        
        # Create a client for the check_obstacle service
        client = self.create_client(CheckObstacle, '/check_obstacle')
        
        # Create a request for continuous obstacle checking
        request = CheckObstacle.Request()
        request.direction = 0.0  # Forward
        request.max_distance = 1.0
        request.mode = 'continuous'
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertIsNotNone(response)
        
        # Move the robot forward
        self.move_robot(0.2, 0.0, 2.0)
        
        # Create a request to stop continuous checking
        request.mode = 'stop'
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertIsNotNone(response)
    
    def test_navigation_api(self):
        """Test the navigation API."""
        # Import the navigation API
        import sys
        import os
        
        # Add the nevil_navigation package to the Python path
        from ament_index_python.packages import get_package_share_directory
        nevil_navigation_dir = get_package_share_directory('nevil_navigation')
        sys.path.append(os.path.join(nevil_navigation_dir, 'lib', 'python3', 'dist-packages'))
        
        # Import the navigation API
        from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI
        
        # Create a navigation API instance
        nav_api = NevilNavigationAPI(node=self.node)
        
        # Initialize the API
        nav_api.initialize()
        
        # Set the simulation environment to empty
        self.set_simulation_environment('empty')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
        
        # Move forward 0.5 meters
        self.node.get_logger().info('Moving forward 0.5 meters using API')
        result = nav_api.move_forward(0.5, 0.2)
        
        # Verify the result
        self.assertTrue(result.success)
        
        # Turn left 90 degrees
        self.node.get_logger().info('Turning left 90 degrees using API')
        result = nav_api.turn_left(90.0, 0.5)
        
        # Verify the result
        self.assertTrue(result.success)
        
        # Move to a specific point
        self.node.get_logger().info('Moving to point (1.0, 1.0) using API')
        result = nav_api.move_to_point(1.0, 1.0, 0.2, 0.5)
        
        # Verify the result
        self.assertTrue(result.success)
        
        # Clean up
        nav_api.shutdown()
    
    def test_navigation_with_real_time_executor(self):
        """Test navigation with the real-time executor."""
        # Set the simulation environment to empty
        self.set_simulation_environment('empty')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
        
        # Create a client for the rt_executor service
        from nevil_realtime.srv import SetRtPriority
        
        client = self.create_client(SetRtPriority, '/rt_executor/set_priority')
        
        # Create a request to set high priority for navigation
        request = SetRtPriority.Request()
        request.node_name = 'navigation_node'
        request.priority = 90  # High priority
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)
        
        # Create an action client
        nav_client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to a point 1 meter ahead
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 1.0
        goal_msg.target_pose.pose.position.y = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.1
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal with RT executor')
        result = self.wait_for_action_result(nav_client, goal_msg, timeout_sec=30.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        
        # Reset the priority
        request.priority = 0  # Default priority
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)

if __name__ == '__main__':
    unittest.main()