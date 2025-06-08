#!/usr/bin/env python3

"""
Real-time Hardware Interface for Nevil-picar v2.0.

This module provides a real-time hardware interface for the Nevil-picar v2.0 system,
interfacing with the PiCar-X platform using the robot_hat library.
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# Import the robot_hat library and Picarx class
import sys
import os
# Add the picarlibs directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../v1.0/picarlibs'))
from picarx import Picarx

# Define our own constrain function
def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class RTHardwareInterface:
    """
    Real-time hardware interface for Nevil-picar v2.0.
    
    This class provides a thread-safe interface to the PiCar-X hardware,
    with proper mutex handling for real-time performance.
    """
    
    def __init__(self, node=None):
        """
        Initialize the hardware interface.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        self.node = node
        self.logger = get_logger('rt_hardware_interface') if node is None else node.get_logger()
        
        # Initialize hardware mutex
        self.hardware_mutex = threading.Lock()
        
        # Initialize the PiCar-X hardware
        try:
            with self.hardware_mutex:
                self.logger.info('Initializing PiCar-X hardware...')
                self.picar = Picarx()
                self.logger.info('PiCar-X hardware initialized successfully')
        except Exception as e:
            self.logger.error(f'Failed to initialize PiCar-X hardware: {e}')
            # Create a mock PiCar for simulation if hardware initialization fails
            self.picar = None
            self.simulation_mode = True
            self.left_motor_speed = 0
            self.right_motor_speed = 0
            self.steering_angle = 0
            self.distance = 100.0  # Default distance in cm
            self.logger.warn('Running in simulation mode')
        else:
            self.simulation_mode = False
            self.logger.info('Running in hardware mode')
    
    def set_motor_speeds(self, left, right):
        """
        Set motor speeds with proper mutex handling.
        
        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
        """
        # Convert from -1.0 to 1.0 range to -100 to 100 range
        left_motor = int(left * 100.0)
        right_motor = int(right * 100.0)
        
        # Log the motor speeds
        self.logger.debug(f'Setting motor speeds: left={left_motor}, right={right_motor}')
        
        if self.simulation_mode:
            # In simulation mode, just store the values
            self.left_motor_speed = left_motor
            self.right_motor_speed = right_motor
            return
        
        # In hardware mode, set the motor speeds with mutex protection
        try:
            with self.hardware_mutex:
                # The PiCar-X uses a different motor numbering scheme
                # Motor 1 is left, Motor 2 is right
                self.picar.set_motor_speed(1, left_motor)
                self.picar.set_motor_speed(2, right_motor)
        except Exception as e:
            self.logger.error(f'Failed to set motor speeds: {e}')
    
    def set_steering_angle(self, angle):
        """
        Set steering angle with proper mutex handling.
        
        Args:
            angle: Steering angle in degrees (-30 to 30)
        """
        # Constrain the angle to the valid range
        angle = constrain(angle, -30, 30)
        
        # Log the steering angle
        self.logger.debug(f'Setting steering angle: {angle}')
        
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.steering_angle = angle
            return
        
        # In hardware mode, set the steering angle with mutex protection
        try:
            with self.hardware_mutex:
                self.picar.set_dir_servo_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set steering angle: {e}')
    
    def get_distance(self):
        """
        Get distance from ultrasonic sensor with proper mutex handling.
        
        Returns:
            Distance in centimeters
        """
        if self.simulation_mode:
            # In simulation mode, return the simulated distance
            return self.distance
        
        # In hardware mode, get the distance with mutex protection
        try:
            with self.hardware_mutex:
                distance = self.picar.get_distance()
                self.logger.debug(f'Ultrasonic distance: {distance} cm')
                return distance
        except Exception as e:
            self.logger.error(f'Failed to get distance: {e}')
            return 100.0  # Default safe distance
    
    def set_camera_pan(self, angle):
        """
        Set camera pan angle with proper mutex handling.
        
        Args:
            angle: Pan angle in degrees (-90 to 90)
        """
        # Constrain the angle to the valid range
        angle = constrain(angle, -90, 90)
        
        # Log the camera pan angle
        self.logger.debug(f'Setting camera pan angle: {angle}')
        
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.camera_pan = angle
            return
        
        # In hardware mode, set the camera pan angle with mutex protection
        try:
            with self.hardware_mutex:
                self.picar.set_cam_pan_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set camera pan angle: {e}')
    
    def set_camera_tilt(self, angle):
        """
        Set camera tilt angle with proper mutex handling.
        
        Args:
            angle: Tilt angle in degrees (-35 to 65)
        """
        # Constrain the angle to the valid range
        angle = constrain(angle, -35, 65)
        
        # Log the camera tilt angle
        self.logger.debug(f'Setting camera tilt angle: {angle}')
        
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.camera_tilt = angle
            return
        
        # In hardware mode, set the camera tilt angle with mutex protection
        try:
            with self.hardware_mutex:
                self.picar.set_cam_tilt_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set camera tilt angle: {e}')
    
    def stop(self):
        """
        Stop all motors with proper mutex handling.
        """
        self.logger.info('Stopping all motors')
        
        if self.simulation_mode:
            # In simulation mode, just reset the values
            self.left_motor_speed = 0
            self.right_motor_speed = 0
            return
        
        # In hardware mode, stop the motors with mutex protection
        try:
            with self.hardware_mutex:
                self.picar.stop()
        except Exception as e:
            self.logger.error(f'Failed to stop motors: {e}')
    
    def reset(self):
        """
        Reset the hardware to default state with proper mutex handling.
        """
        self.logger.info('Resetting hardware to default state')
        
        if self.simulation_mode:
            # In simulation mode, just reset the values
            self.left_motor_speed = 0
            self.right_motor_speed = 0
            self.steering_angle = 0
            self.camera_pan = 0
            self.camera_tilt = 0
            return
        
        # In hardware mode, reset the hardware with mutex protection
        try:
            with self.hardware_mutex:
                self.picar.reset()
        except Exception as e:
            self.logger.error(f'Failed to reset hardware: {e}')
    
    def cleanup(self):
        """
        Clean up hardware resources.
        """
        self.logger.info('Cleaning up hardware resources')
        
        if self.simulation_mode:
            # Nothing to clean up in simulation mode
            return
        
        # In hardware mode, stop the motors and reset the hardware
        try:
            with self.hardware_mutex:
                self.picar.stop()
                self.picar.reset()
        except Exception as e:
            self.logger.error(f'Failed to clean up hardware resources: {e}')


def main(args=None):
    """
    Main function for testing the hardware interface.
    
    This function initializes ROS2, creates a node and hardware interface,
    and runs a series of tests to verify the hardware functionality.
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create a node
    node = Node('rt_hardware_interface_test')
    
    # Create the hardware interface
    hw = RTHardwareInterface(node)
    
    try:
        # Test the hardware interface
        node.get_logger().info('Testing hardware interface...')
        
        # Test motor control
        hw.set_motor_speeds(0.5, 0.5)  # Forward at half speed
        time.sleep(1.0)
        hw.stop()
        time.sleep(0.5)
        
        # Test steering
        hw.set_steering_angle(30)  # Turn right
        time.sleep(1.0)
        hw.set_steering_angle(-30)  # Turn left
        time.sleep(1.0)
        hw.set_steering_angle(0)  # Center
        time.sleep(0.5)
        
        # Test distance sensor
        distance = hw.get_distance()
        node.get_logger().info(f'Distance: {distance} cm')
        
        # Test camera control
        hw.set_camera_pan(45)  # Pan right
        hw.set_camera_tilt(30)  # Tilt up
        time.sleep(1.0)
        hw.set_camera_pan(-45)  # Pan left
        hw.set_camera_tilt(-30)  # Tilt down
        time.sleep(1.0)
        hw.set_camera_pan(0)  # Center
        hw.set_camera_tilt(0)  # Center
        
        # Reset the hardware
        hw.reset()
        
        node.get_logger().info('Hardware interface test completed')
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    finally:
        # Clean up
        hw.cleanup()
        node.destroy_node()
        rclpy.shutdown()


# Simple test code
if __name__ == "__main__":
    main()