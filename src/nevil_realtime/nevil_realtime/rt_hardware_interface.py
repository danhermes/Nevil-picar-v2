#!/usr/bin/env python3

"""
Real-time Hardware Interface for Nevil-picar v2.0.

This module provides a real-time hardware interface for the Nevil-picar v2.0 system,
with clean separation between physical hardware and simulation modes through
an abstract interface architecture.
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Dict, Any

# Define our own constrain function
def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))


class HardwareBackend(Enum):
    """Available hardware backend types."""
    PHYSICAL = "physical"
    SIMULATION = "simulation"


class HardwareInterface(ABC):
    """
    Abstract base class for hardware interfaces.
    
    This defines the contract that all hardware implementations must follow,
    enabling clean separation between physical and simulation modes.
    """
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the hardware interface."""
        pass
    
    @abstractmethod
    def set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speeds (-1.0 to 1.0)."""
        pass
    
    @abstractmethod
    def set_steering_angle(self, angle: float) -> None:
        """Set steering angle (-30 to 30 degrees)."""
        pass
    
    @abstractmethod
    def get_distance(self) -> float:
        """Get distance from ultrasonic sensor in cm."""
        pass
    
    @abstractmethod
    def set_camera_pan(self, angle: float) -> None:
        """Set camera pan angle (-90 to 90 degrees)."""
        pass
    
    @abstractmethod
    def set_camera_tilt(self, angle: float) -> None:
        """Set camera tilt angle (-35 to 65 degrees)."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop all motors."""
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset hardware to default state."""
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Clean up hardware resources."""
        pass
    
    @abstractmethod
    def get_backend_type(self) -> str:
        """Get the backend type (physical/simulation)."""
        pass


class PhysicalHardwareInterface(HardwareInterface):
    """Physical hardware implementation using PiCar-X."""
    
    def __init__(self, logger):
        self.logger = logger
        self.hardware_mutex = threading.Lock()
        self.picar = None
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize the PiCar-X hardware using v1.0 action helper."""
        try:
            # Import the robot_hat library and Picarx class
            import sys
            import os
            
            # Add both v1.0 directories to the Python path
            v1_path = os.path.join(os.path.dirname(__file__), '../../../v1.0')
            picarlibs_path = os.path.join(v1_path, 'picarlibs')
            helpers_path = os.path.join(v1_path, 'helpers')
            
            sys.path.append(picarlibs_path)
            sys.path.append(helpers_path)
            sys.path.append(v1_path)
            
            from picarx import Picarx
            # Import v1.0 action helper functions
            from action_helper import move_forward_this_way, move_backward_this_way, turn_left, turn_right, stop
            
            with self.hardware_mutex:
                self.logger.info('Initializing PiCar-X hardware with v1.0 action helper...')
                self.picar = Picarx()
                
                # Store v1.0 action helper functions
                self.move_forward_this_way = move_forward_this_way
                self.move_backward_this_way = move_backward_this_way
                self.turn_left = turn_left
                self.turn_right = turn_right
                self.stop_action = stop
                
                self.initialized = True
                self.logger.info('PiCar-X hardware initialized successfully with v1.0 integration')
                return True
        except Exception as e:
            self.logger.error(f'Failed to initialize PiCar-X hardware: {e}')
            return False
    
    def set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speeds with proper mutex handling."""
        if not self.initialized:
            return
        
        # Convert from -1.0 to 1.0 range to -100 to 100 range
        left_motor = int(left * 100.0)
        right_motor = int(right * 100.0)
        
        self.logger.debug(f'Setting motor speeds: left={left_motor}, right={right_motor}')
        
        try:
            with self.hardware_mutex:
                # Motor 1 is left, Motor 2 is right
                self.picar.set_motor_speed(1, left_motor)
                self.picar.set_motor_speed(2, right_motor)
        except Exception as e:
            self.logger.error(f'Failed to set motor speeds: {e}')
    
    def set_steering_angle(self, angle: float) -> None:
        """Set steering angle with proper mutex handling."""
        if not self.initialized:
            return
        
        angle = constrain(angle, -30, 30)
        self.logger.debug(f'Setting steering angle: {angle}')
        
        try:
            with self.hardware_mutex:
                self.picar.set_dir_servo_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set steering angle: {e}')
    
    def get_distance(self) -> float:
        """Get distance from ultrasonic sensor."""
        if not self.initialized:
            return 100.0
        
        try:
            with self.hardware_mutex:
                distance = self.picar.get_distance()
                self.logger.debug(f'Ultrasonic distance: {distance} cm')
                return distance
        except Exception as e:
            self.logger.error(f'Failed to get distance: {e}')
            return 100.0
    
    def set_camera_pan(self, angle: float) -> None:
        """Set camera pan angle."""
        if not self.initialized:
            return
        
        angle = constrain(angle, -90, 90)
        self.logger.debug(f'Setting camera pan angle: {angle}')
        
        try:
            with self.hardware_mutex:
                self.picar.set_cam_pan_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set camera pan angle: {e}')
    
    def set_camera_tilt(self, angle: float) -> None:
        """Set camera tilt angle."""
        if not self.initialized:
            return
        
        angle = constrain(angle, -35, 65)
        self.logger.debug(f'Setting camera tilt angle: {angle}')
        
        try:
            with self.hardware_mutex:
                self.picar.set_cam_tilt_angle(angle)
        except Exception as e:
            self.logger.error(f'Failed to set camera tilt angle: {e}')
    
    def stop(self) -> None:
        """Stop all motors."""
        if not self.initialized:
            return
        
        self.logger.info('Stopping all motors')
        try:
            with self.hardware_mutex:
                self.picar.stop()
        except Exception as e:
            self.logger.error(f'Failed to stop motors: {e}')
    
    def reset(self) -> None:
        """Reset hardware to default state."""
        if not self.initialized:
            return
        
        self.logger.info('Resetting hardware to default state')
        try:
            with self.hardware_mutex:
                self.picar.reset()
        except Exception as e:
            self.logger.error(f'Failed to reset hardware: {e}')
    
    def cleanup(self) -> None:
        """Clean up hardware resources."""
        if not self.initialized:
            return
        
        self.logger.info('Cleaning up hardware resources')
        try:
            with self.hardware_mutex:
                self.picar.stop()
                self.picar.reset()
        except Exception as e:
            self.logger.error(f'Failed to clean up hardware resources: {e}')
    
    def get_backend_type(self) -> str:
        """Get the backend type."""
        return HardwareBackend.PHYSICAL.value


class SimulationHardwareInterface(HardwareInterface):
    """Simulation hardware implementation."""
    
    def __init__(self, logger):
        self.logger = logger
        self.left_motor_speed = 0.0
        self.right_motor_speed = 0.0
        self.steering_angle = 0.0
        self.camera_pan = 0.0
        self.camera_tilt = 0.0
        self.distance = 100.0
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize simulation mode."""
        self.logger.info('Initializing simulation mode')
        self.initialized = True
        return True
    
    def set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speeds in simulation."""
        left_motor = int(left * 100.0)
        right_motor = int(right * 100.0)
        
        self.left_motor_speed = left_motor
        self.right_motor_speed = right_motor
        
        self.logger.debug(f'Simulation: Setting motor speeds: left={left_motor}, right={right_motor}')
    
    def set_steering_angle(self, angle: float) -> None:
        """Set steering angle in simulation."""
        angle = constrain(angle, -30, 30)
        self.steering_angle = angle
        self.logger.debug(f'Simulation: Setting steering angle: {angle}')
    
    def get_distance(self) -> float:
        """Get simulated distance."""
        return self.distance
    
    def set_camera_pan(self, angle: float) -> None:
        """Set camera pan in simulation."""
        angle = constrain(angle, -90, 90)
        self.camera_pan = angle
        self.logger.debug(f'Simulation: Setting camera pan angle: {angle}')
    
    def set_camera_tilt(self, angle: float) -> None:
        """Set camera tilt in simulation."""
        angle = constrain(angle, -35, 65)
        self.camera_tilt = angle
        self.logger.debug(f'Simulation: Setting camera tilt angle: {angle}')
    
    def stop(self) -> None:
        """Stop all motors in simulation."""
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.logger.info('Simulation: Stopping all motors')
    
    def reset(self) -> None:
        """Reset simulation to default state."""
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.steering_angle = 0
        self.camera_pan = 0
        self.camera_tilt = 0
        self.logger.info('Simulation: Resetting to default state')
    
    def cleanup(self) -> None:
        """Clean up simulation resources."""
        self.logger.info('Simulation: Cleaning up resources')
    
    def get_backend_type(self) -> str:
        """Get the backend type."""
        return HardwareBackend.SIMULATION.value

class HardwareManager:
    """
    Factory class for creating hardware interface instances.
    
    This class handles the selection and creation of appropriate hardware
    backends based on configuration and runtime detection.
    """
    
    @staticmethod
    def create_hardware_interface(logger, backend: Optional[str] = None,
                                config: Optional[Dict[str, Any]] = None) -> HardwareInterface:
        """
        Create a hardware interface instance.
        
        Args:
            logger: Logger instance for the hardware interface
            backend: Preferred backend type ('physical', 'simulation', or None for auto-detect)
            config: Configuration dictionary for hardware settings
            
        Returns:
            HardwareInterface: Configured hardware interface instance
        """
        if config is None:
            config = {}
        
        # Determine backend selection strategy
        if backend is None:
            backend = config.get('hardware_backend', 'auto')
        
        logger.info(f'Hardware backend selection: {backend}')
        
        # Auto-detection: try physical first, fall back to simulation
        if backend == 'auto':
            physical_interface = PhysicalHardwareInterface(logger)
            if physical_interface.initialize():
                logger.info('Auto-detection: Using physical hardware backend')
                return physical_interface
            else:
                logger.warn('Auto-detection: Physical hardware unavailable, using simulation backend')
                simulation_interface = SimulationHardwareInterface(logger)
                simulation_interface.initialize()
                return simulation_interface
        
        # Explicit backend selection
        elif backend == HardwareBackend.PHYSICAL.value:
            interface = PhysicalHardwareInterface(logger)
            if not interface.initialize():
                raise RuntimeError('Failed to initialize physical hardware backend')
            return interface
        
        elif backend == HardwareBackend.SIMULATION.value:
            interface = SimulationHardwareInterface(logger)
            interface.initialize()
            return interface
        
        else:
            raise ValueError(f'Unknown hardware backend: {backend}')


class RTHardwareInterface:
    """
    Real-time hardware interface for Nevil-picar v2.0.
    
    This class provides a unified interface to hardware operations with
    clean separation between physical and simulation modes through
    dependency injection and the strategy pattern.
    """
    
    def __init__(self, node=None, backend: Optional[str] = None,
                 config: Optional[Dict[str, Any]] = None):
        """
        Initialize the hardware interface.
        
        Args:
            node: ROS2 node for logging (optional)
            backend: Hardware backend type ('physical', 'simulation', 'auto')
            config: Configuration dictionary for hardware settings
        """
        self.node = node
        self.logger = get_logger('rt_hardware_interface') if node is None else node.get_logger()
        
        # Create the appropriate hardware interface using the factory
        try:
            self.hardware = HardwareManager.create_hardware_interface(
                self.logger, backend, config
            )
            self.logger.info(f'Hardware interface initialized with {self.hardware.get_backend_type()} backend')
        except Exception as e:
            self.logger.error(f'Failed to initialize hardware interface: {e}')
            # Emergency fallback to simulation
            self.logger.warn('Emergency fallback: Using simulation backend')
            self.hardware = SimulationHardwareInterface(self.logger)
            self.hardware.initialize()
    
    def set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speeds with proper delegation to backend."""
        self.hardware.set_motor_speeds(left, right)
    
    def set_steering_angle(self, angle: float) -> None:
        """Set steering angle with proper delegation to backend."""
        self.hardware.set_steering_angle(angle)
    
    def get_distance(self) -> float:
        """Get distance from ultrasonic sensor with proper delegation to backend."""
        return self.hardware.get_distance()
    
    def set_camera_pan(self, angle: float) -> None:
        """Set camera pan angle with proper delegation to backend."""
        self.hardware.set_camera_pan(angle)
    
    def set_camera_tilt(self, angle: float) -> None:
        """Set camera tilt angle with proper delegation to backend."""
        self.hardware.set_camera_tilt(angle)
    
    def stop(self) -> None:
        """Stop all motors with proper delegation to backend."""
        self.hardware.stop()
    
    def reset(self) -> None:
        """Reset hardware to default state with proper delegation to backend."""
        self.hardware.reset()
    
    def cleanup(self) -> None:
        """Clean up hardware resources with proper delegation to backend."""
        self.hardware.cleanup()
    
    def get_backend_type(self) -> str:
        """Get the current backend type."""
        return self.hardware.get_backend_type()
    
    @property
    def simulation_mode(self) -> bool:
        """Check if running in simulation mode (for backward compatibility)."""
        return self.hardware.get_backend_type() == HardwareBackend.SIMULATION.value


def main(args=None):
    """
    Main function for testing the hardware interface.
    
    This function initializes ROS2, creates a node and hardware interface,
    and runs a series of tests to verify the hardware functionality with
    different backend configurations.
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create a node
    node = Node('rt_hardware_interface_test')
    
    try:
        # Test different backend configurations
        test_configs = [
            {'backend': 'auto', 'description': 'Auto-detection'},
            {'backend': 'simulation', 'description': 'Forced simulation'},
            # Uncomment to test physical hardware (will fail if not available)
            # {'backend': 'physical', 'description': 'Forced physical hardware'},
        ]
        
        for config in test_configs:
            node.get_logger().info(f'Testing {config["description"]} backend...')
            
            # Create the hardware interface with specific backend
            hw = RTHardwareInterface(node, backend=config['backend'])
            
            node.get_logger().info(f'Backend type: {hw.get_backend_type()}')
            node.get_logger().info(f'Simulation mode: {hw.simulation_mode}')
            
            # Test motor control
            hw.set_motor_speeds(0.5, 0.5)  # Forward at half speed
            time.sleep(0.5)
            hw.stop()
            time.sleep(0.2)
            
            # Test steering
            hw.set_steering_angle(30)  # Turn right
            time.sleep(0.5)
            hw.set_steering_angle(-30)  # Turn left
            time.sleep(0.5)
            hw.set_steering_angle(0)  # Center
            time.sleep(0.2)
            
            # Test distance sensor
            distance = hw.get_distance()
            node.get_logger().info(f'Distance: {distance} cm')
            
            # Test camera control
            hw.set_camera_pan(45)  # Pan right
            hw.set_camera_tilt(30)  # Tilt up
            time.sleep(0.5)
            hw.set_camera_pan(-45)  # Pan left
            hw.set_camera_tilt(-30)  # Tilt down
            time.sleep(0.5)
            hw.set_camera_pan(0)  # Center
            hw.set_camera_tilt(0)  # Center
            
            # Reset the hardware
            hw.reset()
            
            # Clean up this instance
            hw.cleanup()
            
            node.get_logger().info(f'{config["description"]} backend test completed\n')
        
        node.get_logger().info('All hardware interface tests completed successfully')
        
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    except Exception as e:
        node.get_logger().error(f'Test failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Simple test code
if __name__ == "__main__":
    main()