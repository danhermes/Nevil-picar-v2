#!/usr/bin/env python3

"""
PiCar-X Physical Movement Driver for Nevil-picar v2.0

This module provides a physical hardware implementation of the MovementInterface
using the PiCar-X robot platform. It integrates the picarx library to control
the actual hardware components including motors, servos, and sensors.
"""

import sys
import os
import time
import logging
from typing import Optional, Dict, Any

# Add picar-x library to path
PICARX_PATH = '/home/dan/picar-x'
if PICARX_PATH not in sys.path:
    sys.path.insert(0, PICARX_PATH)

try:
    from picarx import Picarx
    PICARX_AVAILABLE = True
except ImportError as e:
    PICARX_AVAILABLE = False
    Picarx = None

from nevil_hardware.interfaces.movement_interface import MovementInterface, MovementStatus


class PiCarXMovementDriver(MovementInterface):
    """
    Physical movement driver for PiCar-X robot platform.
    
    This driver provides direct hardware control using the picarx library,
    implementing the MovementInterface for seamless integration with the
    Nevil robot system.
    """
    
    # PiCar-X hardware limits
    SPEED_MIN = -100
    SPEED_MAX = 100
    STEERING_MIN = -30
    STEERING_MAX = 30
    
    def __init__(self, 
                 logger: Optional[logging.Logger] = None,
                 config: Optional[Dict[str, Any]] = None):
        """
        Initialize the PiCar-X movement driver.
        
        Args:
            logger: Logger instance for debugging
            config: Configuration parameters
        """
        super().__init__()
        self.logger = logger or logging.getLogger(__name__)
        self.config = config or {}
        
        # PiCar-X instance
        self._picarx: Optional[Picarx] = None
        
        # Driver state
        self._initialized = False
        self._emergency_stop = False
        
        # Update status
        self._status.backend_type = "physical"
        self._status.hardware_available = PICARX_AVAILABLE
        
        # Configuration parameters
        self._speed_scale = self.config.get('speed_scale', 1.0)  # Scale factor for speeds
        self._steering_scale = self.config.get('steering_scale', 1.0)  # Scale factor for steering
        self._safety_timeout = self.config.get('safety_timeout', 2.0)  # Safety timeout in seconds
        
        if not PICARX_AVAILABLE:
            self._status.error_message = "PiCar-X library not available"
            self.logger.error("PiCar-X library not available - cannot use physical driver")
    
    def initialize(self) -> bool:
        """
        Initialize the PiCar-X hardware.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        if not PICARX_AVAILABLE:
            self.logger.error("Cannot initialize: PiCar-X library not available")
            return False
        
        try:
            self.logger.info("Initializing PiCar-X hardware...")
            
            # Create PiCar-X instance
            self._picarx = Picarx()
            
            # Reset to safe state
            self._picarx.reset()
            time.sleep(0.5)  # Allow hardware to settle
            
            # Test basic functionality
            if not self._test_hardware():
                self.logger.error("Hardware test failed")
                return False
            
            self._initialized = True
            self._status.hardware_available = True
            self._status.error_message = None
            
            self.logger.info("PiCar-X hardware initialized successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize PiCar-X hardware: {e}")
            self._status.error_message = f"Initialization failed: {e}"
            return False
    
    def _test_hardware(self) -> bool:
        """
        Test basic hardware functionality.
        
        Returns:
            bool: True if hardware test passes
        """
        try:
            # Test servo movement
            self._picarx.set_dir_servo_angle(0)
            time.sleep(0.1)
            
            # Test motor control (brief movement)
            self._picarx.set_motor_speed(1, 10)
            self._picarx.set_motor_speed(2, 10)
            time.sleep(0.1)
            self._picarx.stop()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Hardware test failed: {e}")
            return False
    
    def set_motor_speeds(self, left: float, right: float) -> bool:
        """
        Set motor speeds for differential drive.
        
        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        if not self._check_ready():
            return False
        
        try:
            # Constrain and scale speeds
            left = max(-1.0, min(1.0, left)) * self._speed_scale
            right = max(-1.0, min(1.0, right)) * self._speed_scale
            
            # Convert to PiCar-X speed range (-100 to 100)
            left_speed = int(left * 100)
            right_speed = int(right * 100)
            
            # Set motor speeds (PiCar-X uses 1-indexed motors)
            self._picarx.set_motor_speed(1, left_speed)  # Left motor
            self._picarx.set_motor_speed(2, right_speed)  # Right motor
            
            # Update status
            self._status.left_motor_speed = left
            self._status.right_motor_speed = right
            self._status.is_moving = abs(left) > 0.01 or abs(right) > 0.01
            self._status.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set motor speeds: {e}")
            self._status.error_message = f"Motor control failed: {e}"
            return False
    
    def set_steering_angle(self, angle: float) -> bool:
        """
        Set steering angle for front wheels.
        
        Args:
            angle: Steering angle in degrees (-30 to 30)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        if not self._check_ready():
            return False
        
        try:
            # Constrain and scale angle
            angle = max(-30.0, min(30.0, angle)) * self._steering_scale
            
            # Set steering servo angle
            self._picarx.set_dir_servo_angle(angle)
            
            # Update status
            self._status.steering_angle = angle
            self._status.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set steering angle: {e}")
            self._status.error_message = f"Steering control failed: {e}"
            return False
    
    def stop(self) -> bool:
        """
        Stop all movement immediately.
        
        Returns:
            bool: True if stop successful, False otherwise
        """
        if not self._picarx:
            return False
        
        try:
            # Emergency stop
            self._picarx.stop()
            
            # Update status
            self._status.left_motor_speed = 0.0
            self._status.right_motor_speed = 0.0
            self._status.is_moving = False
            self._status.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to stop: {e}")
            self._status.error_message = f"Stop failed: {e}"
            return False
    
    def reset(self) -> bool:
        """
        Reset movement system to default state.
        
        Returns:
            bool: True if reset successful, False otherwise
        """
        if not self._check_ready():
            return False
        
        try:
            # Reset PiCar-X to default state
            self._picarx.reset()
            
            # Update status
            self._status.left_motor_speed = 0.0
            self._status.right_motor_speed = 0.0
            self._status.steering_angle = 0.0
            self._status.is_moving = False
            self._status.last_command_time = time.time()
            self._status.error_message = None
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to reset: {e}")
            self._status.error_message = f"Reset failed: {e}"
            return False
    
    def get_status(self) -> MovementStatus:
        """
        Get current movement status.
        
        Returns:
            MovementStatus: Current status of movement system
        """
        # Update hardware availability
        self._status.hardware_available = (
            PICARX_AVAILABLE and 
            self._initialized and 
            self._picarx is not None and
            not self._emergency_stop
        )
        
        return self._status
    
    def cleanup(self) -> None:
        """Clean up resources and stop movement."""
        try:
            if self._picarx:
                self.logger.info("Cleaning up PiCar-X hardware...")
                self._picarx.stop()
                self._picarx.reset()
                time.sleep(0.2)  # Allow hardware to settle
            
            self._picarx = None
            self._initialized = False
            
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
    
    def _check_ready(self) -> bool:
        """
        Check if the driver is ready for commands.
        
        Returns:
            bool: True if ready, False otherwise
        """
        if not PICARX_AVAILABLE:
            self._status.error_message = "PiCar-X library not available"
            return False
        
        if not self._initialized or not self._picarx:
            self._status.error_message = "Driver not initialized"
            return False
        
        if self._emergency_stop:
            self._status.error_message = "Emergency stop active"
            return False
        
        return True
    
    # Additional PiCar-X specific methods
    def get_distance(self) -> Optional[float]:
        """
        Get distance reading from ultrasonic sensor.
        
        Returns:
            Optional[float]: Distance in cm, None if unavailable
        """
        if not self._check_ready():
            return None
        
        try:
            return self._picarx.get_distance()
        except Exception as e:
            self.logger.error(f"Failed to get distance: {e}")
            return None
    
    def get_grayscale_data(self) -> Optional[list]:
        """
        Get grayscale sensor readings.
        
        Returns:
            Optional[list]: List of 3 grayscale values, None if unavailable
        """
        if not self._check_ready():
            return None
        
        try:
            return self._picarx.get_grayscale_data()
        except Exception as e:
            self.logger.error(f"Failed to get grayscale data: {e}")
            return None
    
    def set_camera_angle(self, pan: float = 0.0, tilt: float = 0.0) -> bool:
        """
        Set camera pan and tilt angles.
        
        Args:
            pan: Pan angle (-90 to 90 degrees)
            tilt: Tilt angle (-35 to 65 degrees)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self._check_ready():
            return False
        
        try:
            # Constrain angles to PiCar-X limits
            pan = max(-90.0, min(90.0, pan))
            tilt = max(-35.0, min(65.0, tilt))
            
            self._picarx.set_cam_pan_angle(pan)
            self._picarx.set_cam_tilt_angle(tilt)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set camera angles: {e}")
            return False
    
    def emergency_stop(self) -> None:
        """Activate emergency stop mode."""
        self._emergency_stop = True
        self.stop()
        self.logger.warning("Emergency stop activated")
    
    def clear_emergency_stop(self) -> None:
        """Clear emergency stop mode."""
        self._emergency_stop = False
        self.logger.info("Emergency stop cleared")


# Factory function for easy instantiation
def create_picarx_driver(logger: Optional[logging.Logger] = None,
                        config: Optional[Dict[str, Any]] = None) -> PiCarXMovementDriver:
    """
    Create and initialize a PiCar-X movement driver.
    
    Args:
        logger: Logger instance
        config: Configuration parameters
        
    Returns:
        PiCarXMovementDriver: Initialized driver instance
    """
    driver = PiCarXMovementDriver(logger=logger, config=config)
    
    if not driver.initialize():
        raise RuntimeError("Failed to initialize PiCar-X driver")
    
    return driver