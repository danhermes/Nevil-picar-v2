#!/usr/bin/env python3

import asyncio
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import RealtimeCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from nevil_interfaces.action import NavigateToPoint, PerformBehavior
from nevil_interfaces.srv import CheckObstacle
from nevil_interfaces.msg import BehaviorStatus, SystemStatus

class AsyncNevilNavigationAPI:
    """
    Asynchronous version of the Nevil Navigation API.
    
    This class provides the same functionality as NevilNavigationAPI,
    but with async/await support for non-blocking operation.
    """
    
    def __init__(self, node=None):
        """
        Initialize the Async Nevil Navigation API.
        
        Args:
            node: An existing ROS2 node to use. If None, a new node will be created.
        """
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create or use an existing node
        if node is None:
            self.node = rclpy.create_node('async_nevil_navigation_api')
            self.should_spin = True
        else:
            self.node = node
            self.should_spin = False
        
        # Create callback groups
        self.rt_callback_group = RealtimeCallbackGroup()
        self.regular_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create QoS profiles
        self.rt_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=self.rt_qos
        )
        
        # Create subscribers
        self.ultrasonic_sub = self.node.create_subscription(
            Range,
            '/ultrasonic_data',
            self._ultrasonic_callback,
            qos_profile=self.rt_qos,
            callback_group=self.rt_callback_group
        )
        
        self.system_status_sub = self.node.create_subscription(
            SystemStatus,
            '/system_status',
            self._system_status_callback,
            10,
            callback_group=self.regular_callback_group
        )
        
        # Create service clients
        self.check_obstacle_client = self.node.create_client(
            CheckObstacle,
            '/check_obstacle',
            callback_group=self.regular_callback_group
        )
        
        # Create action clients
        self.navigate_client = ActionClient(
            self.node,
            NavigateToPoint,
            '/navigate_to_point',
            callback_group=self.regular_callback_group
        )
        
        self.behavior_client = ActionClient(
            self.node,
            PerformBehavior,
            '/perform_behavior',
            callback_group=self.regular_callback_group
        )
        
        # Initialize state variables
        self.current_distance = float('inf')
        self.safe_distance = 0.5  # 50cm
        self.danger_distance = 0.2  # 20cm
        self.system_status = None
        self.emergency_stop = False
        
        # Constants for movement calculations
        self.SPEED_TO_CM_PER_SEC = 0.7  # Calibration factor
        
        # Start the executor in a separate thread if needed
        if self.should_spin:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.spin_thread = threading.Thread(target=self._spin)
            self.spin_thread.daemon = True
            self.spin_thread.start()
        
        self.node.get_logger().info('Async Nevil Navigation API initialized')
    
    def _spin(self):
        """Spin the executor in a separate thread."""
        try:
            self.executor.spin()
        except Exception as e:
            self.node.get_logger().error(f'Error in executor: {e}')
    
    def _ultrasonic_callback(self, msg):
        """Callback for ultrasonic sensor data."""
        self.current_distance = msg.range
    
    def _system_status_callback(self, msg):
        """Callback for system status updates."""
        self.system_status = msg
        
        # Check for emergency stop
        if msg.has_errors and 'EMERGENCY_STOP' in msg.error_codes:
            self.emergency_stop = True
        else:
            self.emergency_stop = False
    
    def get_distance(self):
        """Get the current distance from the ultrasonic sensor."""
        return self.current_distance
    
    async def check_obstacle(self, direction=0.0, max_distance=1.0):
        """
        Check for obstacles in the specified direction.
        
        Args:
            direction: Direction to check (in radians, 0 = forward)
            max_distance: Maximum distance to check (in meters)
            
        Returns:
            A tuple of (obstacle_detected, distance, direction)
        """
        # Create the request
        request = CheckObstacle.Request()
        request.direction = direction
        request.max_distance = max_distance
        request.mode = 'single'
        
        # Call the service
        future = self.check_obstacle_client.call_async(request)
        
        # Wait for the result
        while not future.done():
            await asyncio.sleep(0.01)
        
        # Process the result
        if future.result() is not None:
            response = future.result()
            return (
                response.obstacle_detected,
                response.distance,
                response.obstacle_direction
            )
        else:
            self.node.get_logger().error('Failed to call CheckObstacle service')
            return (False, float('inf'), 0.0)
    
    async def move_forward_this_way(self, distance_cm, speed=None, check_obstacles=True):
        """
        Move forward a specific distance at given speed.
        
        Args:
            distance_cm: Distance to move in centimeters
            speed: Speed to move at (0.0 to 1.0)
            check_obstacles: Whether to check for obstacles during movement
            
        Returns:
            True if movement completed successfully, False otherwise
        """
        # Convert cm to meters
        distance_m = distance_cm / 100.0
        
        # Default speed if not specified
        if speed is None:
            speed = 0.5  # Default to 50% speed
        
        # Clamp speed to valid range
        speed = max(0.0, min(1.0, speed))
        
        # Calculate linear velocity in m/s
        linear_velocity = speed * 0.5  # Max speed is 0.5 m/s
        
        if check_obstacles:
            # Use the NavigateToPoint action with obstacle avoidance
            goal_msg = NavigateToPoint.Goal()
            
            # Create a target pose at the specified distance in front of the robot
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = 'base_link'
            goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = distance_m
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.position.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0
            
            goal_msg.linear_velocity = linear_velocity
            goal_msg.angular_velocity = 0.0
            goal_msg.avoid_obstacles = True
            goal_msg.goal_tolerance = 0.05  # 5cm tolerance
            
            # Send the goal
            self.node.get_logger().info(f'Moving forward {distance_cm}cm at speed {speed}')
            
            # Wait for the action server
            server_ready = await self._wait_for_action_server(self.navigate_client, timeout=1.0)
            if not server_ready:
                self.node.get_logger().error('Navigate action server not available')
                return False
            
            # Send the goal
            send_goal_future = self.navigate_client.send_goal_async(goal_msg)
            
            # Wait for the goal to be accepted
            while not send_goal_future.done():
                await asyncio.sleep(0.01)
            
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.node.get_logger().error('Navigate goal rejected')
                return False
            
            # Wait for the result
            get_result_future = goal_handle.get_result_async()
            
            while not get_result_future.done():
                await asyncio.sleep(0.01)
            
            result = get_result_future.result().result
            
            if result.success:
                self.node.get_logger().info('Movement completed successfully')
                return True
            else:
                self.node.get_logger().warn(f'Movement failed: {result.message}')
                return False
        else:
            # Direct control using cmd_vel for simpler movements
            self.node.get_logger().info(f'Moving forward {distance_cm}cm at speed {speed}')
            
            # Calculate move time based on distance and speed
            move_time = distance_m / linear_velocity
            
            # Create and send the command
            cmd = Twist()
            cmd.linear.x = linear_velocity
            cmd.angular.z = 0.0
            
            # Start time
            start_time = time.time()
            
            # Send commands until we've moved the desired distance
            while time.time() - start_time < move_time:
                # Check for emergency stop
                if self.emergency_stop:
                    self.node.get_logger().warn('Emergency stop triggered')
                    await self.stop()
                    return False
                
                # Check for obstacles if requested
                if check_obstacles and self.current_distance < self.danger_distance:
                    self.node.get_logger().warn(f'Obstacle detected at {self.current_distance:.2f}m')
                    await self.stop()
                    return False
                
                # Send the command
                self.cmd_vel_pub.publish(cmd)
                await asyncio.sleep(0.1)
            
            # Stop the robot
            await self.stop()
            
            self.node.get_logger().info('Movement completed')
            return True
    
    async def move_backward_this_way(self, distance_cm, speed=None):
        """
        Move backward a specific distance at given speed.
        
        Args:
            distance_cm: Distance to move in centimeters
            speed: Speed to move at (0.0 to 1.0)
            
        Returns:
            True if movement completed successfully, False otherwise
        """
        # Convert cm to meters
        distance_m = distance_cm / 100.0
        
        # Default speed if not specified
        if speed is None:
            speed = 0.5  # Default to 50% speed
        
        # Clamp speed to valid range
        speed = max(0.0, min(1.0, speed))
        
        # Calculate linear velocity in m/s (negative for backward)
        linear_velocity = -speed * 0.5  # Max speed is 0.5 m/s
        
        # Direct control using cmd_vel for backward movement
        self.node.get_logger().info(f'Moving backward {distance_cm}cm at speed {speed}')
        
        # Calculate move time based on distance and speed
        move_time = distance_m / abs(linear_velocity)
        
        # Create and send the command
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = 0.0
        
        # Start time
        start_time = time.time()
        
        # Send commands until we've moved the desired distance
        while time.time() - start_time < move_time:
            # Check for emergency stop
            if self.emergency_stop:
                self.node.get_logger().warn('Emergency stop triggered')
                await self.stop()
                return False
            
            # Send the command
            self.cmd_vel_pub.publish(cmd)
            await asyncio.sleep(0.1)
        
        # Stop the robot
        await self.stop()
        
        self.node.get_logger().info('Movement completed')
        return True
    
    async def turn_left(self):
        """
        Turn the robot left.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.node.get_logger().info('Starting left turn sequence')
        
        # Set wheel angle to -30 degrees
        await self.set_dir_servo_angle(-30)
        
        # Move forward first segment
        if not await self.move_forward_this_way(20):
            return False
        
        # Straighten wheels
        await self.set_dir_servo_angle(0)
        
        # Move forward second segment
        if not await self.move_forward_this_way(20):
            return False
        
        self.node.get_logger().info('Left turn complete')
        return True
    
    async def turn_right(self):
        """
        Turn the robot right.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.node.get_logger().info('Starting right turn sequence')
        
        # Set wheel angle to 30 degrees
        await self.set_dir_servo_angle(30)
        
        # Move forward first segment
        if not await self.move_forward_this_way(20):
            return False
        
        # Straighten wheels
        await self.set_dir_servo_angle(0)
        
        # Move forward second segment
        if not await self.move_forward_this_way(20):
            return False
        
        self.node.get_logger().info('Right turn complete')
        return True
    
    async def stop(self):
        """
        Stop the robot.
        
        Returns:
            True if stop command was sent successfully
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        return True
    
    async def set_dir_servo_angle(self, angle):
        """
        Set the direction servo angle.
        
        Args:
            angle: Angle in degrees
            
        Returns:
            True if command was sent successfully
        """
        # Convert angle to radians
        angle_rad = math.radians(angle)
        
        # Create and send the command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angle_rad
        self.cmd_vel_pub.publish(cmd)
        return True
    
    async def set_cam_pan_angle(self, angle):
        """
        Set the camera pan angle.
        
        Args:
            angle: Angle in degrees
            
        Returns:
            True if command was sent successfully
        """
        # This would be implemented using a camera control service or topic
        self.node.get_logger().info(f'Setting camera pan angle to {angle} degrees')
        return True
    
    async def set_cam_tilt_angle(self, angle):
        """
        Set the camera tilt angle.
        
        Args:
            angle: Angle in degrees
            
        Returns:
            True if command was sent successfully
        """
        # This would be implemented using a camera control service or topic
        self.node.get_logger().info(f'Setting camera tilt angle to {angle} degrees')
        return True
    
    async def perform_behavior(self, behavior_name, duration=5.0, **params):
        """
        Perform a predefined behavior.
        
        Args:
            behavior_name: Name of the behavior to perform
            duration: Maximum duration of the behavior in seconds
            **params: Additional parameters for the behavior
            
        Returns:
            True if behavior completed successfully, False otherwise
        """
        # Create the goal message
        goal_msg = PerformBehavior.Goal()
        goal_msg.behavior_id = behavior_name.lower().replace(' ', '_')
        goal_msg.behavior_name = behavior_name
        goal_msg.behavior_category = 'expression'
        goal_msg.duration = duration
        
        # Add parameters
        param_names = []
        param_values = []
        for key, value in params.items():
            param_names.append(key)
            param_values.append(str(value))
        
        goal_msg.param_names = param_names
        goal_msg.param_values = param_values
        
        # Wait for the action server
        server_ready = await self._wait_for_action_server(self.behavior_client, timeout=1.0)
        if not server_ready:
            self.node.get_logger().error('Behavior action server not available')
            return False
        
        # Send the goal
        self.node.get_logger().info(f'Performing behavior: {behavior_name}')
        send_goal_future = self.behavior_client.send_goal_async(goal_msg)
        
        # Wait for the goal to be accepted
        while not send_goal_future.done():
            await asyncio.sleep(0.01)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().error('Behavior goal rejected')
            return False
        
        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        
        while not get_result_future.done():
            await asyncio.sleep(0.01)
        
        result = get_result_future.result().result
        
        if result.success:
            self.node.get_logger().info(f'Behavior completed successfully in {result.actual_duration:.2f}s')
            return True
        else:
            self.node.get_logger().warn(f'Behavior failed: {result.message}')
            return False
    
    # Helper method to wait for action server
    async def _wait_for_action_server(self, action_client, timeout=1.0):
        """
        Wait for an action server to become available.
        
        Args:
            action_client: The action client to wait for
            timeout: Timeout in seconds
            
        Returns:
            True if the server is available, False otherwise
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if action_client.server_is_ready():
                return True
            await asyncio.sleep(0.1)
        return False
    
    # Implement the original action_helper.py behaviors using the perform_behavior method
    
    async def wave_hands(self):
        """Wave the robot's hands (steering servos)."""
        return await self.perform_behavior('wave_hands')
    
    async def resist(self):
        """Make the robot resist (move servos back and forth)."""
        return await self.perform_behavior('resist')
    
    async def act_cute(self):
        """Make the robot act cute."""
        return await self.perform_behavior('act_cute')
    
    async def rub_hands(self):
        """Make the robot rub hands (steering servos)."""
        return await self.perform_behavior('rub_hands')
    
    async def think(self):
        """Make the robot think (move camera and steering)."""
        return await self.perform_behavior('think')
    
    async def keep_think(self):
        """Make the robot keep thinking (continuous movement)."""
        return await self.perform_behavior('keep_think')
    
    async def shake_head(self):
        """Make the robot shake its head (camera pan)."""
        return await self.perform_behavior('shake_head')
    
    async def nod(self):
        """Make the robot nod (camera tilt)."""
        return await self.perform_behavior('nod')
    
    async def depressed(self):
        """Make the robot look depressed."""
        return await self.perform_behavior('depressed')
    
    async def twist_body(self):
        """Make the robot twist its body."""
        return await self.perform_behavior('twist_body')
    
    async def celebrate(self):
        """Make the robot celebrate."""
        return await self.perform_behavior('celebrate')
    
    async def honk(self):
        """Make the robot honk."""
        return await self.perform_behavior('honk')
    
    async def start_engine(self):
        """Make the robot start its engine sound."""
        return await self.perform_behavior('start_engine')
    
    def shutdown(self):
        """Shutdown the API and clean up resources."""
        if self.should_spin:
            self.executor.shutdown()
            self.spin_thread.join()
        
        self.node.get_logger().info('Async Nevil Navigation API shutdown')