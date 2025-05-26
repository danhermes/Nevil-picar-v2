#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nevil_interfaces.action import PerformBehavior

from nevil_testing.test_base import NevilTestBase

class TestMotionControlNode(NevilTestBase):
    """
    Unit tests for the motion_control_node in the nevil_navigation package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscription to odometry
        self.odom = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Wait for the motion control node to be ready
        self.wait_for_motion_control()
    
    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.odom = msg
    
    def wait_for_motion_control(self, timeout_sec=5.0):
        """Wait for the motion control node to be ready."""
        # Wait for odometry
        if not self.spin_until(lambda: self.odom is not None, timeout_sec):
            self.fail('Timed out waiting for odometry')
    
    def test_cmd_vel_subscription(self):
        """Test that the motion control node subscribes to cmd_vel."""
        # Create a Twist message
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        
        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
        
        # Wait for the odometry to change
        initial_odom = self.odom
        
        def check_odom_changed():
            return (self.odom is not None and 
                    self.odom.twist.twist.linear.x > 0.0)
        
        if not self.spin_until(check_odom_changed, 1.0):
            self.fail('Timed out waiting for odometry to change')
        
        # Verify the odometry changed
        self.assertGreater(self.odom.twist.twist.linear.x, 0.0)
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_zero_velocity(self):
        """Test that the robot stops when commanded to."""
        # First, move the robot
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        
        # Then stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Wait for the robot to stop
        def check_robot_stopped():
            return (self.odom is not None and 
                    abs(self.odom.twist.twist.linear.x) < 0.01 and
                    abs(self.odom.twist.twist.angular.z) < 0.01)
        
        if not self.spin_until(check_robot_stopped, 1.0):
            self.fail('Timed out waiting for robot to stop')
        
        # Verify the robot stopped
        self.assertLess(abs(self.odom.twist.twist.linear.x), 0.01)
        self.assertLess(abs(self.odom.twist.twist.angular.z), 0.01)
    
    def test_rotation(self):
        """Test that the robot rotates when commanded to."""
        # Create a Twist message for rotation
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2
        
        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
        
        # Wait for the odometry to change
        def check_odom_rotation():
            return (self.odom is not None and 
                    abs(self.odom.twist.twist.angular.z) > 0.0)
        
        if not self.spin_until(check_odom_rotation, 1.0):
            self.fail('Timed out waiting for odometry rotation to change')
        
        # Verify the odometry changed
        self.assertGreater(abs(self.odom.twist.twist.angular.z), 0.0)
        
        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_velocity_limits(self):
        """Test that the motion control node enforces velocity limits."""
        # Create a Twist message with excessive velocity
        twist = Twist()
        twist.linear.x = 10.0  # Excessive linear velocity
        twist.angular.z = 10.0  # Excessive angular velocity
        
        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        
        # Verify the velocity is limited
        self.assertLess(abs(self.odom.twist.twist.linear.x), 1.0)  # Assuming max linear velocity is less than 1.0
        self.assertLess(abs(self.odom.twist.twist.angular.z), 2.0)  # Assuming max angular velocity is less than 2.0
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_perform_behavior_action(self):
        """Test the perform_behavior action."""
        # Create an action client
        client = self.create_action_client(
            PerformBehavior,
            '/perform_behavior'
        )
        
        # Create a goal
        goal_msg = PerformBehavior.Goal()
        goal_msg.behavior_id = 'test_behavior'
        goal_msg.behavior_name = 'Test Behavior'
        goal_msg.behavior_category = 'test'
        goal_msg.duration = 2.0
        
        # Send the goal and wait for the result
        result = self.wait_for_action_result(client, goal_msg)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertGreaterEqual(result.result.actual_duration, 0.0)
    
    def test_emergency_stop(self):
        """Test the emergency stop functionality."""
        # First, move the robot
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        
        # Create a publisher to the emergency stop topic
        estop_pub = self.create_publisher(String, '/system/emergency_stop', 10)
        
        # Publish an emergency stop message
        from std_msgs.msg import String
        estop_msg = String()
        estop_msg.data = 'STOP'
        estop_pub.publish(estop_msg)
        
        # Wait for the robot to stop
        def check_robot_stopped():
            return (self.odom is not None and 
                    abs(self.odom.twist.twist.linear.x) < 0.01 and
                    abs(self.odom.twist.twist.angular.z) < 0.01)
        
        if not self.spin_until(check_robot_stopped, 1.0):
            self.fail('Timed out waiting for robot to stop after emergency stop')
        
        # Verify the robot stopped
        self.assertLess(abs(self.odom.twist.twist.linear.x), 0.01)
        self.assertLess(abs(self.odom.twist.twist.angular.z), 0.01)
        
        # Try to move the robot again (should not move due to emergency stop)
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
        
        # Verify the robot did not move
        self.assertLess(abs(self.odom.twist.twist.linear.x), 0.01)
        
        # Clear the emergency stop
        estop_msg.data = 'CLEAR'
        estop_pub.publish(estop_msg)
        time.sleep(0.1)
        
        # Now the robot should move
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        
        # Wait for the robot to move
        def check_robot_moving():
            return (self.odom is not None and 
                    self.odom.twist.twist.linear.x > 0.0)
        
        if not self.spin_until(check_robot_moving, 1.0):
            self.fail('Timed out waiting for robot to move after clearing emergency stop')
        
        # Verify the robot is moving
        self.assertGreater(self.odom.twist.twist.linear.x, 0.0)
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)

if __name__ == '__main__':
    unittest.main()