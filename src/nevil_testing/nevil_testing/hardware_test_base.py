#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

from nevil_testing.test_base import NevilTestBase

class HardwareTestBase(NevilTestBase):
    """
    Base class for Nevil-picar v2.0 hardware tests.
    Provides functionality for testing with the physical hardware.
    """
    
    def setUp(self):
        """Set up the hardware test."""
        # Call the parent setUp
        super().setUp()
        
        # Create publishers and subscribers for hardware interaction
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscription to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create a subscription to ultrasonic sensor data
        self.ultrasonic_sub = self.create_subscription(
            LaserScan,  # Using LaserScan for compatibility with simulation
            '/ultrasonic_data',
            self.ultrasonic_callback,
            10
        )
        
        # Create a subscription to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize state variables
        self.current_odom = None
        self.current_ultrasonic = None
        self.current_image = None
        
        # Wait for hardware to be ready
        self.wait_for_hardware()
    
    def tearDown(self):
        """Tear down the hardware test."""
        # Stop the robot
        self.stop_robot()
        
        # Call the parent tearDown
        super().tearDown()
    
    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.current_odom = msg
    
    def ultrasonic_callback(self, msg):
        """Callback for ultrasonic sensor messages."""
        self.current_ultrasonic = msg
    
    def image_callback(self, msg):
        """Callback for camera image messages."""
        self.current_image = msg
    
    def wait_for_hardware(self, timeout_sec=5.0):
        """Wait for the hardware to be ready."""
        # Wait for odometry, ultrasonic sensor, and camera image
        self.node.get_logger().info('Waiting for hardware to be ready...')
        
        # Wait for odometry
        if not self.spin_until(lambda: self.current_odom is not None, timeout_sec):
            self.fail('Timed out waiting for odometry')
        
        # Wait for ultrasonic sensor
        if not self.spin_until(lambda: self.current_ultrasonic is not None, timeout_sec):
            self.fail('Timed out waiting for ultrasonic sensor')
        
        # Wait for camera image
        if not self.spin_until(lambda: self.current_image is not None, timeout_sec):
            self.fail('Timed out waiting for camera image')
        
        self.node.get_logger().info('Hardware is ready')
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.1)  # Give time for the command to be processed
    
    def move_robot(self, linear_x, angular_z, duration_sec=1.0):
        """Move the robot with the given velocity for the given duration."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        # Publish the velocity command
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        self.stop_robot()
    
    def get_robot_pose(self):
        """Get the current robot pose from odometry."""
        if self.current_odom is None:
            self.fail('No odometry data available')
        
        return self.current_odom.pose.pose
    
    def get_obstacle_distance(self):
        """Get the current obstacle distance from ultrasonic sensor."""
        if self.current_ultrasonic is None:
            self.fail('No ultrasonic sensor data available')
        
        # Find the minimum distance in the ranges
        return min(self.current_ultrasonic.ranges)
    
    def get_camera_image(self):
        """Get the current camera image."""
        if self.current_image is None:
            self.fail('No camera image available')
        
        return self.current_image
    
    def navigate_to_point(self, x, y, theta=0.0, timeout_sec=30.0):
        """Navigate to a point using the navigation action."""
        from nevil_interfaces.action import NavigateToPoint
        
        # Create the action client
        client = self.create_action_client(
            NavigateToPoint,
            '/navigate_to_point'
        )
        
        # Create the goal
        goal_msg = NavigateToPoint.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.orientation.z = theta
        goal_msg.linear_velocity = 0.2
        goal_msg.angular_velocity = 0.5
        goal_msg.avoid_obstacles = True
        goal_msg.goal_tolerance = 0.1
        
        # Send the goal and wait for the result
        result = self.wait_for_action_result(client, goal_msg, timeout_sec)
        
        return result
    
    def perform_behavior(self, behavior_id, behavior_name, behavior_category, params=None, duration=5.0, timeout_sec=30.0):
        """Perform a behavior using the behavior action."""
        from nevil_interfaces.action import PerformBehavior
        
        # Create the action client
        client = self.create_action_client(
            PerformBehavior,
            '/perform_behavior'
        )
        
        # Create the goal
        goal_msg = PerformBehavior.Goal()
        goal_msg.behavior_id = behavior_id
        goal_msg.behavior_name = behavior_name
        goal_msg.behavior_category = behavior_category
        goal_msg.duration = duration
        
        if params:
            goal_msg.param_names = list(params.keys())
            goal_msg.param_values = list(map(str, params.values()))
        
        # Send the goal and wait for the result
        result = self.wait_for_action_result(client, goal_msg, timeout_sec)
        
        return result
    
    def check_obstacle(self, direction=0.0, max_distance=1.0, mode='single'):
        """Check for obstacles using the obstacle check service."""
        from nevil_interfaces.srv import CheckObstacle
        
        # Create the client
        client = self.create_client(CheckObstacle, '/check_obstacle')
        
        # Create the request
        request = CheckObstacle.Request()
        request.direction = direction
        request.max_distance = max_distance
        request.mode = mode
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        return response
    
    def process_dialog(self, utterance, context_id='', dialog_mode='command', timeout=10.0, timeout_sec=30.0):
        """Process a dialog using the dialog action."""
        from nevil_interfaces_ai.action import ProcessDialog
        
        # Create the action client
        client = self.create_action_client(
            ProcessDialog,
            '/process_dialog'
        )
        
        # Create the goal
        goal_msg = ProcessDialog.Goal()
        goal_msg.initial_utterance = utterance
        goal_msg.context_id = context_id
        goal_msg.dialog_mode = dialog_mode
        goal_msg.timeout = timeout
        
        # Send the goal and wait for the result
        result = self.wait_for_action_result(client, goal_msg, timeout_sec)
        
        return result
    
    def check_battery_level(self):
        """Check the battery level."""
        from std_msgs.msg import Float32
        
        # Create a subscription to battery level
        battery_level = None
        
        def callback(msg):
            nonlocal battery_level
            battery_level = msg.data
        
        battery_sub = self.create_subscription(
            Float32,
            '/battery_level',
            callback,
            10
        )
        
        # Wait for battery level
        if not self.spin_until(lambda: battery_level is not None, 1.0):
            self.fail('Timed out waiting for battery level')
        
        # Clean up
        self.node.destroy_subscription(battery_sub)
        
        return battery_level
    
    def check_system_status(self):
        """Check the system status."""
        from nevil_interfaces.msg import SystemStatus
        
        # Create a subscription to system status
        system_status = None
        
        def callback(msg):
            nonlocal system_status
            system_status = msg
        
        status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            callback,
            10
        )
        
        # Wait for system status
        if not self.spin_until(lambda: system_status is not None, 1.0):
            self.fail('Timed out waiting for system status')
        
        # Clean up
        self.node.destroy_subscription(status_sub)
        
        return system_status