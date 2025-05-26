#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nevil_interfaces.action import NavigateToPoint, PerformBehavior
from nevil_interfaces.srv import CheckObstacle
from nevil_interfaces_ai.action import ProcessDialog
from nevil_interfaces_ai.msg import TextCommand, TextResponse
import math
import yaml
import os
import json

from nevil_testing.simulation_test_base import SimulationTestBase

class TestEndToEndNavigation(SimulationTestBase):
    """
    System tests for end-to-end navigation in Nevil-picar v2.0.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a subscription to text responses
        self.text_response = None
        self.text_response_sub = self.create_subscription(
            TextResponse,
            '/text_response',
            self.text_response_callback,
            10
        )
        
        # Create a publisher for text commands
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/text_command',
            10
        )
        
        # Set the simulation environment to maze
        self.set_simulation_environment('maze')
        
        # Wait for the environment to be loaded
        time.sleep(1.0)
    
    def text_response_callback(self, msg):
        """Callback for text response messages."""
        self.text_response = msg
    
    def send_text_command(self, text, command_type='navigation', priority=0, context_id=''):
        """Send a text command."""
        msg = TextCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.command_text = text
        msg.source = 'user'
        msg.command_type = command_type
        msg.priority = priority
        msg.command_id = f'test_{time.time()}'
        msg.context_id = context_id
        
        self.text_command_pub.publish(msg)
    
    def test_navigate_to_goal_with_obstacles(self):
        """Test navigating to a goal with obstacles."""
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
        goal_msg.target_pose.pose.position.x = 3.0
        goal_msg.target_pose.pose.position.y = 3.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.2
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal with obstacles')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=120.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.result_code, 'GOAL_REACHED')
        
        # Verify the final pose is close to the target
        final_pose = result.result.final_pose.pose
        distance = math.sqrt((final_pose.position.x - 3.0)**2 + 
                             (final_pose.position.y - 3.0)**2)
        
        self.assertLessEqual(distance, goal_msg.goal_tolerance)
    
    def test_voice_command_navigation(self):
        """Test navigation via voice commands."""
        # Create an action client for the process dialog action
        client = self.create_action_client(
            ProcessDialog,
            '/process_dialog'
        )
        
        # Create a goal to process a navigation command
        goal_msg = ProcessDialog.Goal()
        goal_msg.initial_utterance = "Move forward two meters"
        goal_msg.context_id = ''
        goal_msg.dialog_mode = 'command'
        goal_msg.timeout = 60.0
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending voice command for navigation')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=120.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.final_state, 'completed')
        
        # Verify that the robot moved forward
        self.assertGreater(self.get_robot_pose().position.x, 1.5)
    
    def test_text_command_navigation(self):
        """Test navigation via text commands."""
        # Reset text response
        self.text_response = None
        
        # Get initial position
        initial_pose = self.get_robot_pose()
        
        # Send a text command to move forward
        self.send_text_command("Move forward one meter")
        
        # Wait for the command to be processed and executed
        time.sleep(10.0)
        
        # Verify that the robot moved forward
        current_pose = self.get_robot_pose()
        distance_moved = math.sqrt((current_pose.position.x - initial_pose.position.x)**2 + 
                                  (current_pose.position.y - initial_pose.position.y)**2)
        
        self.assertGreater(distance_moved, 0.8)  # Allow for some tolerance
        
        # Verify that we received a text response
        self.assertIsNotNone(self.text_response)
    
    def test_complex_navigation_sequence(self):
        """Test a complex navigation sequence."""
        # Create a context ID for this conversation
        context_id = f'test_context_{time.time()}'
        
        # Reset text response
        self.text_response = None
        
        # Get initial position
        initial_pose = self.get_robot_pose()
        
        # Send a text command to move forward
        self.send_text_command("Move forward one meter", context_id=context_id)
        
        # Wait for the command to be processed and executed
        time.sleep(10.0)
        
        # Verify that we received a text response
        self.assertIsNotNone(self.text_response)
        
        # Reset text response
        self.text_response = None
        
        # Send a text command to turn left
        self.send_text_command("Turn left 90 degrees", context_id=context_id)
        
        # Wait for the command to be processed and executed
        time.sleep(10.0)
        
        # Verify that we received a text response
        self.assertIsNotNone(self.text_response)
        
        # Reset text response
        self.text_response = None
        
        # Send a text command to move forward again
        self.send_text_command("Move forward one meter", context_id=context_id)
        
        # Wait for the command to be processed and executed
        time.sleep(10.0)
        
        # Verify that we received a text response
        self.assertIsNotNone(self.text_response)
        
        # Verify that the robot moved in an L-shaped pattern
        current_pose = self.get_robot_pose()
        
        # The robot should have moved approximately 1 meter in X and 1 meter in Y
        self.assertGreater(current_pose.position.x - initial_pose.position.x, 0.8)
        self.assertGreater(current_pose.position.y - initial_pose.position.y, 0.8)
    
    def test_navigation_with_behaviors(self):
        """Test navigation with expressive behaviors."""
        # Create an action client for the perform behavior action
        behavior_client = self.create_action_client(
            PerformBehavior,
            '/perform_behavior'
        )
        
        # Create a goal to perform a behavior
        behavior_goal = PerformBehavior.Goal()
        behavior_goal.behavior_id = 'wave'
        behavior_goal.behavior_name = 'Wave'
        behavior_goal.behavior_category = 'expression'
        behavior_goal.duration = 2.0
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Performing wave behavior')
        behavior_result = self.wait_for_action_result(behavior_client, behavior_goal, timeout_sec=10.0)
        
        # Verify the behavior result
        self.assertTrue(behavior_result.result.success)
        
        # Create an action client for navigation
        nav_client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to a point
        nav_goal = NavigateToPoint.Goal()
        nav_goal.target_pose = PoseStamped()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        nav_goal.target_pose.pose.position.x = 1.0
        nav_goal.target_pose.pose.position.y = 1.0
        nav_goal.target_pose.pose.orientation.w = 1.0
        nav_goal.linear_velocity = 0.2
        nav_goal.angular_velocity = 0.5
        nav_goal.avoid_obstacles = True
        nav_goal.goal_tolerance = 0.2
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal after behavior')
        nav_result = self.wait_for_action_result(nav_client, nav_goal, timeout_sec=60.0)
        
        # Verify the navigation result
        self.assertTrue(nav_result.result.success)
        
        # Perform another behavior after reaching the goal
        behavior_goal.behavior_id = 'nod'
        behavior_goal.behavior_name = 'Nod'
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Performing nod behavior after navigation')
        behavior_result = self.wait_for_action_result(behavior_client, behavior_goal, timeout_sec=10.0)
        
        # Verify the behavior result
        self.assertTrue(behavior_result.result.success)
    
    def test_error_recovery(self):
        """Test error recovery during navigation."""
        # Create an action client
        client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to an unreachable point
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 100.0  # Unreachable
        goal_msg.target_pose.pose.position.y = 100.0  # Unreachable
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.2
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal to unreachable point')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=30.0)
        
        # Verify the result indicates failure
        self.assertFalse(result.result.success)
        self.assertIn(result.result.result_code, ['GOAL_UNREACHABLE', 'PLANNING_FAILED', 'ABORTED'])
        
        # Now try to navigate to a reachable point
        goal_msg.target_pose.pose.position.x = 1.0
        goal_msg.target_pose.pose.position.y = 1.0
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal to reachable point after failure')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=60.0)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.result_code, 'GOAL_REACHED')
    
    def test_performance_under_load(self):
        """Test navigation performance under load."""
        # Create multiple publishers to simulate system load
        pubs = []
        for i in range(10):
            pub = self.create_publisher(String, f'/load_test_{i}', 10)
            pubs.append(pub)
        
        # Create a message to publish
        from std_msgs.msg import String
        msg = String()
        msg.data = 'x' * 1000  # 1KB of data
        
        # Start publishing messages to simulate load
        def publish_load():
            for pub in pubs:
                pub.publish(msg)
        
        # Create a timer to publish load
        timer = self.node.create_timer(0.01, publish_load)  # 100Hz
        
        # Create an action client
        client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create a goal to navigate to a point
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = 1.0
        goal_msg.target_pose.pose.position.y = 1.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.2
        
        # Record start time
        start_time = time.time()
        
        # Send the goal and wait for the result
        self.node.get_logger().info('Sending navigation goal under load')
        result = self.wait_for_action_result(client, goal_msg, timeout_sec=60.0)
        
        # Record end time
        end_time = time.time()
        
        # Stop the timer
        self.node.destroy_timer(timer)
        
        # Verify the result
        self.assertTrue(result.result.success)
        
        # Verify the navigation completed within a reasonable time
        navigation_time = end_time - start_time
        self.node.get_logger().info(f'Navigation completed in {navigation_time:.2f} seconds')
        
        # The time should be reasonable (adjust based on expected performance)
        self.assertLessEqual(navigation_time, 30.0)

if __name__ == '__main__':
    unittest.main()