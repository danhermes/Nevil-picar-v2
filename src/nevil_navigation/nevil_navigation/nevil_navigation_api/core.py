#!/usr/bin/env python3

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

class NevilNavigationAPI:
    """
    ROS2 adaptation of the original action_helper.py API from Nevil v1.0.
    
    This class provides a compatibility layer that allows existing code
    to work with the new ROS2 and PREEMPT-RT architecture with minimal changes.
    
    It implements the same function signatures as the original action_helper.py
    where possible, but adapts the implementation to use ROS2 actions, services,
    and topics, and integrates with the real-time components.
    """
    
    def __init__(self, node=None):
        """
        Initialize the Nevil Navigation API.
        
        Args:
            node: An existing ROS2 node to use. If None, a new node will be created.
        """
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create or use an existing node
        if node is None:
            self.node = rclpy.create_node('nevil_navigation_api')
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
            depth=1
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
        
        # Wait for services and action servers
        self.node.get_logger().info('Waiting for services and action servers...')
        
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
        
        self.node.get_logger().info('Nevil Navigation API initialized')
    
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
    
    def check_obstacle(self, direction=0.0, max_distance=1.0):
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
        rclpy.spin_until_future_complete(self.node, future)
        
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
    
    def with_obstacle_check(self, func):
        """
        Decorator to add obstacle checking to movement functions.
        
        This is a compatibility wrapper for the original decorator in action_helper.py.
        It uses the CheckObstacle service to implement the same functionality.
        """
        def wrapper(car, *args, **kwargs):
            def check_distance():
                distance = car.get_distance()
                if distance >= car.safe_distance:
                    return "safe"
                elif distance >= car.danger_distance:
                    car.set_dir_servo_angle(30)
                    return "caution"
                else:
                    car.set_dir_servo_angle(-30)
                    self.move_backward_this_way(car, 10, car.speed)
                    time.sleep(0.5)
                    return "danger"
            
            return func(car, *args, check_distance=check_distance, **kwargs)
        
        return wrapper
    
    def move_forward_this_way(self, distance_cm, speed=None, check_obstacles=True):
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
            if not self.navigate_client.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().error('Navigate action server not available')
                return False
            
            # Send the goal
            send_goal_future = self.navigate_client.send_goal_async(goal_msg)
            
            # Wait for the goal to be accepted
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.node.get_logger().error('Navigate goal rejected')
                return False
            
            # Wait for the result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future)
            
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
                    self.stop()
                    return False
                
                # Check for obstacles if requested
                if check_obstacles and self.current_distance < self.danger_distance:
                    self.node.get_logger().warn(f'Obstacle detected at {self.current_distance:.2f}m')
                    self.stop()
                    return False
                
                # Send the command
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
            
            # Stop the robot
            self.stop()
            
            self.node.get_logger().info('Movement completed')
            return True
    
    def move_backward_this_way(self, distance_cm, speed=None):
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
                self.stop()
                return False
            
            # Send the command
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop the robot
        self.stop()
        
        self.node.get_logger().info('Movement completed')
        return True
    
    def turn_left(self):
        """
        Turn the robot left.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.node.get_logger().info('Starting left turn sequence')
        
        # Set wheel angle to -30 degrees
        self.set_dir_servo_angle(-30)
        
        # Move forward first segment
        if not self.move_forward_this_way(20):
            return False
        
        # Straighten wheels
        self.set_dir_servo_angle(0)
        
        # Move forward second segment
        if not self.move_forward_this_way(20):
            return False
        
        self.node.get_logger().info('Left turn complete')
        return True
    
    def turn_right(self):
        """
        Turn the robot right.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.node.get_logger().info('Starting right turn sequence')
        
        # Set wheel angle to 30 degrees
        self.set_dir_servo_angle(30)
        
        # Move forward first segment
        if not self.move_forward_this_way(20):
            return False
        
        # Straighten wheels
        self.set_dir_servo_angle(0)
        
        # Move forward second segment
        if not self.move_forward_this_way(20):
            return False
        
        self.node.get_logger().info('Right turn complete')
        return True
    
    def stop(self):
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
    
    def set_dir_servo_angle(self, angle):
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
    
    def set_cam_pan_angle(self, angle):
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
    
    def set_cam_tilt_angle(self, angle):
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
    
    def perform_behavior(self, behavior_name, duration=5.0, **params):
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
        if not self.behavior_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('Behavior action server not available')
            return False
        
        # Send the goal
        self.node.get_logger().info(f'Performing behavior: {behavior_name}')
        send_goal_future = self.behavior_client.send_goal_async(goal_msg)
        
        # Wait for the goal to be accepted
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().error('Behavior goal rejected')
            return False
        
        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future)
        
        result = get_result_future.result().result
        
        if result.success:
            self.node.get_logger().info(f'Behavior completed successfully in {result.actual_duration:.2f}s')
            return True
        else:
            self.node.get_logger().warn(f'Behavior failed: {result.message}')
            return False
    
    # Implement the original action_helper.py behaviors using the perform_behavior method
    
    def wave_hands(self):
        """Wave the robot's hands (steering servos)."""
        return self.perform_behavior('wave_hands')
    
    def resist(self):
        """Make the robot resist (move servos back and forth)."""
        return self.perform_behavior('resist')
    
    def act_cute(self):
        """Make the robot act cute."""
        return self.perform_behavior('act_cute')
    
    def rub_hands(self):
        """Make the robot rub hands (steering servos)."""
        return self.perform_behavior('rub_hands')
    
    def think(self):
        """Make the robot think (move camera and steering)."""
        return self.perform_behavior('think')
    
    def keep_think(self):
        """Make the robot keep thinking (continuous movement)."""
        return self.perform_behavior('keep_think')
    
    def shake_head(self):
        """Make the robot shake its head (camera pan)."""
        return self.perform_behavior('shake_head')
    
    def nod(self):
        """Make the robot nod (camera tilt)."""
        return self.perform_behavior('nod')
    
    def depressed(self):
        """Make the robot look depressed."""
        return self.perform_behavior('depressed')
    
    def twist_body(self):
        """Make the robot twist its body."""
        return self.perform_behavior('twist_body')
    
    def celebrate(self):
        """Make the robot celebrate."""
        return self.perform_behavior('celebrate')
    
    def honk(self):
        """Make the robot honk."""
        return self.perform_behavior('honk')
    
    def start_engine(self):
        """Make the robot start its engine sound."""
        return self.perform_behavior('start_engine')
    
    def shutdown(self):
        """Shutdown the API and clean up resources."""
        if self.should_spin:
            self.executor.shutdown()
            self.spin_thread.join()
        
        self.node.get_logger().info('Nevil Navigation API shutdown')


# Define dictionaries for compatibility with the original action_helper.py
actions_dict = {
    "forward": lambda api, *args, **kwargs: api.move_forward_this_way(*args, **kwargs),
    "backward": lambda api, *args, **kwargs: api.move_backward_this_way(*args, **kwargs),
    "left": lambda api, *args, **kwargs: api.turn_left(*args, **kwargs),
    "right": lambda api, *args, **kwargs: api.turn_right(*args, **kwargs),
    "stop": lambda api, *args, **kwargs: api.stop(*args, **kwargs),
    "twist left": lambda api, *args, **kwargs: api.set_dir_servo_angle(-30, *args, **kwargs),
    "twist right": lambda api, *args, **kwargs: api.set_dir_servo_angle(30, *args, **kwargs),
    "shake head": lambda api, *args, **kwargs: api.shake_head(*args, **kwargs),
    "nod": lambda api, *args, **kwargs: api.nod(*args, **kwargs),
    "wave hands": lambda api, *args, **kwargs: api.wave_hands(*args, **kwargs),
    "resist": lambda api, *args, **kwargs: api.resist(*args, **kwargs),
    "act cute": lambda api, *args, **kwargs: api.act_cute(*args, **kwargs),
    "rub hands": lambda api, *args, **kwargs: api.rub_hands(*args, **kwargs),
    "think": lambda api, *args, **kwargs: api.think(*args, **kwargs),
    "twist body": lambda api, *args, **kwargs: api.twist_body(*args, **kwargs),
    "celebrate": lambda api, *args, **kwargs: api.celebrate(*args, **kwargs),
    "depressed": lambda api, *args, **kwargs: api.depressed(*args, **kwargs),
    "keep think": lambda api, *args, **kwargs: api.keep_think(*args, **kwargs),
    "honk": lambda api, *args, **kwargs: api.honk(*args, **kwargs),
    "start engine": lambda api, *args, **kwargs: api.start_engine(*args, **kwargs)
}