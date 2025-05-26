#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml
import os

from nevil_testing.test_base import NevilTestBase

class TestSimulationNode(NevilTestBase):
    """
    Unit tests for the simulation_node in the nevil_simulation package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriptions to simulation outputs
        self.odom = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan = None
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.image = None
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.sim_state = None
        self.sim_state_sub = self.create_subscription(
            String,
            '/simulation/state',
            self.sim_state_callback,
            10
        )
        
        # Wait for the simulation node to be ready
        self.wait_for_simulation()
    
    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.odom = msg
    
    def scan_callback(self, msg):
        """Callback for laser scan messages."""
        self.scan = msg
    
    def image_callback(self, msg):
        """Callback for image messages."""
        self.image = msg
    
    def sim_state_callback(self, msg):
        """Callback for simulation state messages."""
        self.sim_state = msg
    
    def wait_for_simulation(self, timeout_sec=5.0):
        """Wait for the simulation node to be ready."""
        # Wait for odometry, laser scan, and image
        if not self.spin_until(lambda: self.odom is not None, timeout_sec):
            self.fail('Timed out waiting for odometry')
        
        if not self.spin_until(lambda: self.scan is not None, timeout_sec):
            self.fail('Timed out waiting for laser scan')
        
        if not self.spin_until(lambda: self.image is not None, timeout_sec):
            self.fail('Timed out waiting for image')
        
        if not self.spin_until(lambda: self.sim_state is not None, timeout_sec):
            self.fail('Timed out waiting for simulation state')
    
    def test_simulation_state(self):
        """Test that the simulation node publishes state."""
        # Verify that we received a simulation state message
        self.assertIsNotNone(self.sim_state)
        
        # Verify the simulation state message
        self.assertIn(self.sim_state.data, ['RUNNING', 'PAUSED', 'INITIALIZING'])
    
    def test_cmd_vel_to_odom(self):
        """Test that cmd_vel commands affect odometry."""
        # Get initial odometry
        initial_odom = self.odom
        
        # Send a command to move forward
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        
        # Publish the command
        self.cmd_vel_pub.publish(twist)
        
        # Wait for odometry to change
        def check_odom_changed():
            return (self.odom is not None and 
                    self.odom.pose.pose.position.x != initial_odom.pose.pose.position.x)
        
        if not self.spin_until(check_odom_changed, 1.0):
            self.fail('Timed out waiting for odometry to change')
        
        # Verify odometry changed
        self.assertNotEqual(self.odom.pose.pose.position.x, initial_odom.pose.pose.position.x)
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_cmd_vel_to_rotation(self):
        """Test that cmd_vel commands affect rotation."""
        # Get initial odometry
        initial_odom = self.odom
        
        # Send a command to rotate
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.1
        
        # Publish the command
        self.cmd_vel_pub.publish(twist)
        
        # Wait for odometry to change
        def check_odom_rotation_changed():
            return (self.odom is not None and 
                    self.odom.pose.pose.orientation.z != initial_odom.pose.pose.orientation.z)
        
        if not self.spin_until(check_odom_rotation_changed, 1.0):
            self.fail('Timed out waiting for odometry rotation to change')
        
        # Verify odometry rotation changed
        self.assertNotEqual(self.odom.pose.pose.orientation.z, initial_odom.pose.pose.orientation.z)
        
        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_laser_scan(self):
        """Test that the laser scan data is valid."""
        # Verify that we received a laser scan message
        self.assertIsNotNone(self.scan)
        
        # Verify the laser scan message
        self.assertGreater(len(self.scan.ranges), 0)
        self.assertGreaterEqual(self.scan.range_min, 0.0)
        self.assertGreater(self.scan.range_max, self.scan.range_min)
        
        # Verify that all ranges are valid
        for r in self.scan.ranges:
            self.assertGreaterEqual(r, self.scan.range_min)
            self.assertLessEqual(r, self.scan.range_max)
    
    def test_camera_image(self):
        """Test that the camera image is valid."""
        # Verify that we received an image message
        self.assertIsNotNone(self.image)
        
        # Verify the image message
        self.assertGreater(self.image.height, 0)
        self.assertGreater(self.image.width, 0)
        self.assertGreater(len(self.image.data), 0)
    
    def test_set_environment(self):
        """Test setting the simulation environment."""
        # Create a client for the set_environment service
        from nevil_simulation.srv import SetEnvironment
        
        client = self.create_client(SetEnvironment, '/set_environment')
        
        # Create a request
        request = SetEnvironment.Request()
        request.environment_name = 'empty'
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)
        
        # Wait for the environment to be loaded
        time.sleep(0.5)
        
        # Verify the simulation state
        self.assertIsNotNone(self.sim_state)
        self.assertEqual(self.sim_state.data, 'RUNNING')
    
    def test_reset_simulation(self):
        """Test resetting the simulation."""
        # First, move the robot
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # Get the current position
        moved_position = self.odom.pose.pose.position
        
        # Create a client for the reset_simulation service
        from std_srvs.srv import Empty
        
        client = self.create_client(Empty, '/reset_simulation')
        
        # Create a request
        request = Empty.Request()
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Wait for the simulation to reset
        time.sleep(0.5)
        
        # Verify the robot position was reset
        self.assertNotEqual(self.odom.pose.pose.position.x, moved_position.x)
        self.assertNotEqual(self.odom.pose.pose.position.y, moved_position.y)
    
    def test_pause_resume_simulation(self):
        """Test pausing and resuming the simulation."""
        # Create a client for the pause_simulation service
        from std_srvs.srv import Empty
        
        pause_client = self.create_client(Empty, '/pause_simulation')
        
        # Create a request
        pause_request = Empty.Request()
        
        # Send the request and wait for the response
        pause_response = self.wait_for_service_response(pause_client, pause_request)
        
        # Wait for the simulation to pause
        time.sleep(0.5)
        
        # Verify the simulation state
        self.assertIsNotNone(self.sim_state)
        self.assertEqual(self.sim_state.data, 'PAUSED')
        
        # Try to move the robot (should not move when paused)
        initial_position = self.odom.pose.pose.position
        
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # Verify the robot did not move
        self.assertEqual(self.odom.pose.pose.position.x, initial_position.x)
        self.assertEqual(self.odom.pose.pose.position.y, initial_position.y)
        
        # Create a client for the resume_simulation service
        resume_client = self.create_client(Empty, '/resume_simulation')
        
        # Create a request
        resume_request = Empty.Request()
        
        # Send the request and wait for the response
        resume_response = self.wait_for_service_response(resume_client, resume_request)
        
        # Wait for the simulation to resume
        time.sleep(0.5)
        
        # Verify the simulation state
        self.assertIsNotNone(self.sim_state)
        self.assertEqual(self.sim_state.data, 'RUNNING')
        
        # Now the robot should move
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # Verify the robot moved
        self.assertNotEqual(self.odom.pose.pose.position.x, initial_position.x)
        
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)
    
    def test_get_simulation_parameters(self):
        """Test getting simulation parameters."""
        # Create a client for the get_simulation_parameters service
        from nevil_simulation.srv import GetSimulationParameters
        
        client = self.create_client(GetSimulationParameters, '/get_simulation_parameters')
        
        # Create a request
        request = GetSimulationParameters.Request()
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)
        self.assertIsNotNone(response.parameters)
        
        # Parse the parameters
        params = yaml.safe_load(response.parameters)
        
        # Verify the parameters
        self.assertIn('physics', params)
        self.assertIn('sensors', params)
        self.assertIn('robot', params)

if __name__ == '__main__':
    unittest.main()