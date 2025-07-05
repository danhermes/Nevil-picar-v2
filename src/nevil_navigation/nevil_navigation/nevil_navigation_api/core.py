#!/usr/bin/env python3

import time
import math
import sys
import os
import logging
from typing import Optional

# Add picar-x library to path
PICARX_PATH = '/home/dan/picar-x'
if PICARX_PATH not in sys.path:
    sys.path.insert(0, PICARX_PATH)

# Lazy import of picarx to avoid hardware initialization during build
PICARX_AVAILABLE = False
Picarx = None

def _import_picarx():
    """Lazy import of picarx library."""
    global PICARX_AVAILABLE, Picarx
    if not PICARX_AVAILABLE and Picarx is None:
        try:
            from picarx import Picarx
            PICARX_AVAILABLE = True
        except ImportError:
            PICARX_AVAILABLE = False
            Picarx = None
    return PICARX_AVAILABLE

# Optional ROS2 imports for when ROS2 integration is needed
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class NevilNavigationAPI:
    """
    Direct PiCar-X integration for Nevil Navigation API.
    
    This class provides direct hardware control using PiCar-X, bypassing ROS2 actions
    for immediate response and simplified architecture. It maintains compatibility
    with the original action_helper.py API.
    """
    
    def __init__(self, node=None, force_mock=False):
        """
        Initialize the Nevil Navigation API with direct PiCar-X control.
        
        Args:
            node: Optional ROS2 node for logging (if ROS2 is available)
            force_mock: Force mock mode for testing (default: False)
        """
        # Set up logging
        self.logger = self._setup_logging(node)
        
        # State variables
        self.current_distance = float('inf')
        self.safe_distance = 50.0  # 50cm
        self.danger_distance = 20.0  # 20cm
        self.emergency_stop = False
        self.speed = 50  # Default speed (0-100)
        
        # Initialize PiCar-X directly
        if force_mock:
            self._init_mock_mode()
        else:
            self._init_direct_picarx()
        
        self.logger.info('Nevil Navigation API initialized')
    
    def _setup_logging(self, node):
        """Set up logging with ROS2 node if available, otherwise use Python logging."""
        if node and ROS2_AVAILABLE:
            return node.get_logger()
        else:
            logging.basicConfig(level=logging.INFO)
            return logging.getLogger(__name__)
    
    def _init_direct_picarx(self):
        """Initialize direct PiCar-X control."""
        try:
            if _import_picarx():
                self.picarx = Picarx()
                self.logger.info('Direct PiCar-X control initialized')
            else:
                raise ImportError("PiCar-X library not available")
        except Exception as e:
            self.logger.error(f'Failed to initialize PiCar-X: {e}')
            self._init_mock_mode()
    
    def _init_mock_mode(self):
        """Initialize mock mode for testing."""
        self.picarx = None
        self.logger.info('Mock mode initialized (no hardware control)')
    
    def get_distance(self):
        """Get the current distance from the ultrasonic sensor."""
        try:
            if hasattr(self, 'picarx') and self.picarx:
                distance = self.picarx.get_distance()
                if distance is not None:
                    self.current_distance = distance
                    return distance
            
            return self.current_distance
            
        except Exception as e:
            self.logger.error(f'Failed to get distance: {e}')
            return float('inf')
    
    def check_obstacle(self, direction=0.0, max_distance=1.0):
        """
        Check for obstacles in the specified direction.
        
        Args:
            direction: Direction to check (in radians, 0 = forward)
            max_distance: Maximum distance to check (in meters)
            
        Returns:
            A tuple of (obstacle_detected, distance, direction)
        """
        try:
            # Get current distance
            distance_cm = self.get_distance()
            distance_m = distance_cm / 100.0
            
            # Check if obstacle is within max distance
            obstacle_detected = distance_m < max_distance
            
            return (obstacle_detected, distance_m, direction)
            
        except Exception as e:
            self.logger.error(f'Failed to check obstacle: {e}')
            return (False, float('inf'), 0.0)
    
    def with_obstacle_check(self, func):
        """
        Decorator to add obstacle checking to movement functions.
        
        This maintains compatibility with the original decorator in action_helper.py.
        """
        def wrapper(car, *args, **kwargs):
            def check_distance():
                distance = self.get_distance()
                if distance >= self.safe_distance:
                    return "safe"
                elif distance >= self.danger_distance:
                    self.set_dir_servo_angle(30)
                    return "caution"
                else:
                    self.set_dir_servo_angle(-30)
                    self.move_backward_this_way(10, self.speed)
                    time.sleep(0.5)
                    return "danger"
            
            return func(car, *args, check_distance=check_distance, **kwargs)
        
        return wrapper
    
    def move_forward_this_way(self, distance_cm, speed=None, check_obstacles=True):
        """
        Move forward a specific distance at given speed.
        
        Args:
            distance_cm: Distance to move in centimeters
            speed: Speed to move at (0-100)
            check_obstacles: Whether to check for obstacles during movement
            
        Returns:
            True if movement completed successfully, False otherwise
        """
        if speed is None:
            speed = self.speed
        
        # Clamp speed to valid range
        speed = max(0, min(100, speed))
        
        self.logger.info(f'Moving forward {distance_cm}cm at speed {speed}')
        
        try:
            if hasattr(self, 'picarx') and self.picarx:
                # Use direct PiCar-X control
                if check_obstacles:
                    distance = self.get_distance()
                    if distance < self.safe_distance:
                        self.logger.warning(f'Obstacle detected at {distance:.1f}cm, aborting movement')
                        return False
                
                # Calculate movement time
                move_time = distance_cm / (speed * 0.7)  # Calibration factor
                
                # Start movement
                self.picarx.forward(speed)
                
                # Monitor movement
                start_time = time.time()
                while time.time() - start_time < move_time:
                    if self.emergency_stop:
                        self.picarx.stop()
                        return False
                    
                    if check_obstacles:
                        distance = self.get_distance()
                        if distance < self.danger_distance:
                            self.logger.warning(f'Obstacle detected at {distance:.1f}cm, stopping')
                            self.picarx.stop()
                            return False
                    
                    time.sleep(0.1)
                
                # Stop movement
                self.picarx.stop()
            
            else:
                # Mock mode
                self.logger.info(f'Mock: Moving forward {distance_cm}cm at speed {speed}')
                time.sleep(1.0)  # Simulate movement time
            
            self.logger.info('Movement completed')
            return True
            
        except Exception as e:
            self.logger.error(f'Movement failed: {e}')
            self.stop()
            return False
    
    def move_backward_this_way(self, distance_cm, speed=None):
        """
        Move backward a specific distance at given speed.
        
        Args:
            distance_cm: Distance to move in centimeters
            speed: Speed to move at (0-100)
            
        Returns:
            True if movement completed successfully, False otherwise
        """
        if speed is None:
            speed = self.speed
        
        # Clamp speed to valid range
        speed = max(0, min(100, speed))
        
        self.logger.info(f'Moving backward {distance_cm}cm at speed {speed}')
        
        try:
            if hasattr(self, 'picarx') and self.picarx:
                # Use direct PiCar-X control
                move_time = distance_cm / (speed * 0.7)
                
                self.picarx.backward(speed)
                
                start_time = time.time()
                while time.time() - start_time < move_time:
                    if self.emergency_stop:
                        self.picarx.stop()
                        return False
                    time.sleep(0.1)
                
                self.picarx.stop()
            
            else:
                # Mock mode
                self.logger.info(f'Mock: Moving backward {distance_cm}cm at speed {speed}')
                time.sleep(1.0)
            
            self.logger.info('Backward movement completed')
            return True
            
        except Exception as e:
            self.logger.error(f'Backward movement failed: {e}')
            self.stop()
            return False
    
    def turn_left(self):
        """
        Turn the robot left.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.logger.info('Starting left turn sequence')
        
        try:
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
            
            self.logger.info('Left turn complete')
            return True
            
        except Exception as e:
            self.logger.error(f'Left turn failed: {e}')
            self.stop()
            return False
    
    def turn_right(self):
        """
        Turn the robot right.
        
        Returns:
            True if turn completed successfully, False otherwise
        """
        self.logger.info('Starting right turn sequence')
        
        try:
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
            
            self.logger.info('Right turn complete')
            return True
            
        except Exception as e:
            self.logger.error(f'Right turn failed: {e}')
            self.stop()
            return False
    
    def stop(self):
        """
        Stop the robot.
        
        Returns:
            True if stop command was sent successfully
        """
        try:
            if hasattr(self, 'picarx') and self.picarx:
                self.picarx.stop()
            else:
                self.logger.info('Mock: Stopping robot')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Stop failed: {e}')
            return False
    
    def set_dir_servo_angle(self, angle):
        """
        Set the direction servo angle.
        
        Args:
            angle: Angle in degrees (-30 to 30)
            
        Returns:
            True if command was sent successfully
        """
        try:
            # Clamp angle to valid range
            angle = max(-30, min(30, angle))
            
            if hasattr(self, 'picarx') and self.picarx:
                self.picarx.set_dir_servo_angle(angle)
            else:
                self.logger.info(f'Mock: Setting steering angle to {angle} degrees')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Failed to set steering angle: {e}')
            return False
    
    def set_cam_pan_angle(self, angle):
        """
        Set the camera pan angle.
        
        Args:
            angle: Angle in degrees (-90 to 90)
            
        Returns:
            True if command was sent successfully
        """
        try:
            # Clamp angle to valid range
            angle = max(-90, min(90, angle))
            
            if hasattr(self, 'picarx') and self.picarx:
                self.picarx.set_cam_pan_angle(angle)
            else:
                self.logger.info(f'Mock: Setting camera pan angle to {angle} degrees')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Failed to set camera pan angle: {e}')
            return False
    
    def set_cam_tilt_angle(self, angle):
        """
        Set the camera tilt angle.
        
        Args:
            angle: Angle in degrees (-35 to 65)
            
        Returns:
            True if command was sent successfully
        """
        try:
            # Clamp angle to valid range
            angle = max(-35, min(65, angle))
            
            if hasattr(self, 'picarx') and self.picarx:
                self.picarx.set_cam_tilt_angle(angle)
            else:
                self.logger.info(f'Mock: Setting camera tilt angle to {angle} degrees')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Failed to set camera tilt angle: {e}')
            return False
    
    def perform_behavior(self, behavior_name, duration=5.0, **params):
        """
        Perform a predefined behavior using direct hardware control.
        
        Args:
            behavior_name: Name of the behavior to perform
            duration: Maximum duration of the behavior in seconds
            **params: Additional parameters for the behavior
            
        Returns:
            True if behavior completed successfully, False otherwise
        """
        self.logger.info(f'Performing behavior: {behavior_name}')
        
        try:
            behavior_method = getattr(self, f'_behavior_{behavior_name.lower().replace(" ", "_")}', None)
            if behavior_method:
                return behavior_method(duration, **params)
            else:
                self.logger.warning(f'Unknown behavior: {behavior_name}')
                return False
                
        except Exception as e:
            self.logger.error(f'Behavior {behavior_name} failed: {e}')
            return False
    
    # Behavior implementations using direct hardware control
    
    def _behavior_wave_hands(self, duration=3.0, **params):
        """Wave the robot's hands (steering servos)."""
        end_time = time.time() + duration
        while time.time() < end_time:
            self.set_dir_servo_angle(30)
            time.sleep(0.5)
            self.set_dir_servo_angle(-30)
            time.sleep(0.5)
        self.set_dir_servo_angle(0)
        return True
    
    def _behavior_resist(self, duration=3.0, **params):
        """Make the robot resist (move servos back and forth)."""
        return self._behavior_wave_hands(duration, **params)
    
    def _behavior_act_cute(self, duration=3.0, **params):
        """Make the robot act cute."""
        self.set_cam_tilt_angle(30)
        time.sleep(0.5)
        self.set_dir_servo_angle(15)
        time.sleep(0.5)
        self.set_dir_servo_angle(-15)
        time.sleep(0.5)
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        return True
    
    def _behavior_rub_hands(self, duration=3.0, **params):
        """Make the robot rub hands (steering servos)."""
        end_time = time.time() + duration
        while time.time() < end_time:
            self.set_dir_servo_angle(15)
            time.sleep(0.2)
            self.set_dir_servo_angle(-15)
            time.sleep(0.2)
        self.set_dir_servo_angle(0)
        return True
    
    def _behavior_think(self, duration=3.0, **params):
        """Make the robot think (move camera and steering)."""
        self.set_cam_pan_angle(30)
        self.set_cam_tilt_angle(20)
        time.sleep(1.0)
        self.set_cam_pan_angle(-30)
        time.sleep(1.0)
        self.set_cam_pan_angle(0)
        self.set_cam_tilt_angle(0)
        return True
    
    def _behavior_keep_think(self, duration=5.0, **params):
        """Make the robot keep thinking (continuous movement)."""
        end_time = time.time() + duration
        while time.time() < end_time:
            self.set_cam_pan_angle(20)
            time.sleep(0.5)
            self.set_cam_pan_angle(-20)
            time.sleep(0.5)
        self.set_cam_pan_angle(0)
        return True
    
    def _behavior_shake_head(self, duration=3.0, **params):
        """Make the robot shake its head (camera pan)."""
        for _ in range(3):
            self.set_cam_pan_angle(45)
            time.sleep(0.3)
            self.set_cam_pan_angle(-45)
            time.sleep(0.3)
        self.set_cam_pan_angle(0)
        return True
    
    def _behavior_nod(self, duration=3.0, **params):
        """Make the robot nod (camera tilt)."""
        for _ in range(3):
            self.set_cam_tilt_angle(30)
            time.sleep(0.3)
            self.set_cam_tilt_angle(-20)
            time.sleep(0.3)
        self.set_cam_tilt_angle(0)
        return True
    
    def _behavior_depressed(self, duration=3.0, **params):
        """Make the robot look depressed."""
        self.set_cam_tilt_angle(-30)
        self.set_dir_servo_angle(-15)
        time.sleep(duration)
        self.set_cam_tilt_angle(0)
        self.set_dir_servo_angle(0)
        return True
    
    def _behavior_twist_body(self, duration=3.0, **params):
        """Make the robot twist its body."""
        for _ in range(2):
            self.set_dir_servo_angle(30)
            time.sleep(0.5)
            self.set_dir_servo_angle(-30)
            time.sleep(0.5)
        self.set_dir_servo_angle(0)
        return True
    
    def _behavior_celebrate(self, duration=3.0, **params):
        """Make the robot celebrate."""
        for _ in range(3):
            self.set_cam_tilt_angle(45)
            self.set_dir_servo_angle(30)
            time.sleep(0.3)
            self.set_cam_tilt_angle(0)
            self.set_dir_servo_angle(-30)
            time.sleep(0.3)
        self.set_cam_tilt_angle(0)
        self.set_dir_servo_angle(0)
        return True
    
    def _behavior_honk(self, duration=1.0, **params):
        """Make the robot honk."""
        self.logger.info('Honk! Honk!')
        return True
    
    def _behavior_start_engine(self, duration=2.0, **params):
        """Make the robot start its engine sound."""
        self.logger.info('Engine starting...')
        return True
    
    # Convenience methods for compatibility
    
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
        try:
            self.stop()
            
            if hasattr(self, 'picarx') and self.picarx:
                self.picarx.reset()
            
            self.logger.info('Nevil Navigation API shutdown')
            
        except Exception as e:
            self.logger.error(f'Shutdown failed: {e}')


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