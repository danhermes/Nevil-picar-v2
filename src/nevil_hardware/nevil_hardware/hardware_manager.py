#!/usr/bin/env python3

"""
Hardware Manager Factory for Nevil-picar v2.0

This module provides configuration-driven hardware backend selection,
allowing runtime switching between physical and simulation modes based
on launch parameters and hardware availability.
"""

import os
import sys
import importlib
import logging
from typing import Optional, Dict, Any
from enum import Enum

from nevil_hardware.interfaces.movement_interface import MovementInterface
from nevil_hardware.interfaces.sensor_interface import SensorInterface


class HardwareBackend(Enum):
    """Available hardware backend types."""
    PHYSICAL = "physical"
    SIMULATION = "simulation"
    MOCK = "mock"


class HardwareManager:
    """
    Factory class for creating hardware interfaces based on configuration.
    
    This manager handles:
    - Runtime detection of hardware availability
    - Configuration-driven backend selection
    - Graceful fallback to simulation mode
    - Dynamic loading of hardware drivers
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        """Initialize the hardware manager."""
        self.logger = logger or logging.getLogger(__name__)
        self._movement_interface: Optional[MovementInterface] = None
        self._sensor_interface: Optional[SensorInterface] = None
        self._current_backend: Optional[HardwareBackend] = None
        self._shared_picarx = None  # Shared PiCar-X instance for physical hardware
    
    def detect_hardware_availability(self) -> Dict[str, bool]:
        """
        Detect available hardware capabilities.
        
        Returns:
            Dict[str, bool]: Hardware availability status
        """
        availability = {
            'robot_hat': False,
            'gpio': False,
            'picarx_libs': False,
            'camera': False,
            'ultrasonic': False
        }
        
        try:
            # Check for robot_hat library
            import robot_hat
            availability['robot_hat'] = True
            self.logger.debug("robot_hat library available")
        except ImportError:
            self.logger.debug("robot_hat library not available")
        
        try:
            # Check for GPIO access
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            availability['gpio'] = True
            self.logger.debug("GPIO access available")
        except (ImportError, RuntimeError):
            self.logger.debug("GPIO access not available")
        
        try:
            # Check for PiCar-X libraries
            picarx_path = '/home/dan/picar-x'
            if os.path.exists(picarx_path) and picarx_path not in sys.path:
                sys.path.insert(0, picarx_path)
            
            from picarx import Picarx
            availability['picarx_libs'] = True
            self.logger.debug("PiCar-X libraries available")
        except (ImportError, FileNotFoundError):
            self.logger.debug("PiCar-X libraries not available")
        
        # TODO: Add camera and ultrasonic detection
        
        return availability
    
    def determine_backend(self, 
                         requested_backend: Optional[str] = None,
                         force_simulation: bool = False) -> HardwareBackend:
        """
        Determine the appropriate hardware backend.
        
        Args:
            requested_backend: Explicitly requested backend type
            force_simulation: Force simulation mode regardless of hardware
            
        Returns:
            HardwareBackend: Selected backend type
        """
        if force_simulation:
            self.logger.info("Forcing simulation mode")
            return HardwareBackend.SIMULATION
        
        # Check explicit request
        if requested_backend:
            try:
                backend = HardwareBackend(requested_backend.lower())
                self.logger.info(f"Using explicitly requested backend: {backend.value}")
                return backend
            except ValueError:
                self.logger.warning(f"Invalid backend requested: {requested_backend}")
        
        # Auto-detect based on hardware availability
        availability = self.detect_hardware_availability()
        
        if availability['robot_hat'] and availability['gpio'] and availability['picarx_libs']:
            self.logger.info("Physical hardware detected and available")
            return HardwareBackend.PHYSICAL
        else:
            self.logger.info("Physical hardware not available, using simulation")
            return HardwareBackend.SIMULATION
    
    def create_movement_interface(self, 
                                backend: Optional[HardwareBackend] = None,
                                config: Optional[Dict[str, Any]] = None) -> MovementInterface:
        """
        Create a movement interface based on backend type.
        
        Args:
            backend: Hardware backend type (auto-detected if None)
            config: Configuration parameters for the interface
            
        Returns:
            MovementInterface: Configured movement interface
            
        Raises:
            ImportError: If required backend modules are not available
            RuntimeError: If interface creation fails
        """
        if backend is None:
            backend = self.determine_backend()
        
        config = config or {}
        
        try:
            if backend == HardwareBackend.PHYSICAL:
                return self._create_physical_interface(config)
            elif backend == HardwareBackend.SIMULATION:
                return self._create_simulation_interface(config)
            elif backend == HardwareBackend.MOCK:
                return self._create_mock_interface(config)
            else:
                raise ValueError(f"Unsupported backend: {backend}")
                
        except Exception as e:
            self.logger.error(f"Failed to create {backend.value} interface: {e}")
            # Fallback to simulation
            if backend != HardwareBackend.SIMULATION:
                self.logger.info("Falling back to simulation interface")
                return self._create_simulation_interface(config)
            raise
    
    def _create_physical_interface(self, config: Dict[str, Any]) -> MovementInterface:
        """Create physical hardware interface."""
        try:
            from nevil_hardware.drivers.physical.picarx_movement_driver import PiCarXMovementDriver
            interface = PiCarXMovementDriver(logger=self.logger, config=config)
            
            if not interface.initialize():
                raise RuntimeError("Failed to initialize physical hardware")
            
            self.logger.info("Physical movement interface created successfully")
            return interface
            
        except ImportError as e:
            raise ImportError(f"Physical hardware driver not available: {e}")
    
    def _create_simulation_interface(self, config: Dict[str, Any]) -> MovementInterface:
        """Create simulation interface."""
        try:
            from nevil_hardware.drivers.simulation.simulation_movement_driver import SimulationMovementDriver
            interface = SimulationMovementDriver(logger=self.logger, config=config)
            
            if not interface.initialize():
                raise RuntimeError("Failed to initialize simulation interface")
            
            self.logger.info("Simulation movement interface created successfully")
            return interface
            
        except ImportError as e:
            raise ImportError(f"Simulation driver not available: {e}")
    
    def _create_mock_interface(self, config: Dict[str, Any]) -> MovementInterface:
        """Create mock interface for testing."""
        from nevil_hardware.interfaces.movement_interface import MockMovementInterface
        interface = MockMovementInterface(logger=self.logger)
        
        if not interface.initialize():
            raise RuntimeError("Failed to initialize mock interface")
        
        self.logger.info("Mock movement interface created successfully")
        return interface
    
    def get_movement_interface(self, 
                             backend_type: Optional[str] = None,
                             force_simulation: bool = False,
                             config: Optional[Dict[str, Any]] = None) -> MovementInterface:
        """
        Get or create movement interface with caching.
        
        Args:
            backend_type: Requested backend type
            force_simulation: Force simulation mode
            config: Configuration parameters
            
        Returns:
            MovementInterface: Movement interface instance
        """
        requested_backend = None
        if backend_type:
            try:
                requested_backend = HardwareBackend(backend_type.lower())
            except ValueError:
                self.logger.warning(f"Invalid backend type: {backend_type}")
        
        # Determine backend
        backend = self.determine_backend(
            requested_backend=backend_type,
            force_simulation=force_simulation
        )
        
        # Create new interface if needed
        if (self._movement_interface is None or 
            self._current_backend != backend):
            
            # Cleanup existing interface
            if self._movement_interface:
                self._movement_interface.cleanup()
            
            # Create new interface
            self._movement_interface = self.create_movement_interface(backend, config)
            self._current_backend = backend
        
        return self._movement_interface
    
    def create_sensor_interface(self,
                              backend: Optional[HardwareBackend] = None,
                              config: Optional[Dict[str, Any]] = None) -> SensorInterface:
        """
        Create a sensor interface based on backend type.
        
        Args:
            backend: Hardware backend type (auto-detected if None)
            config: Configuration parameters for the interface
            
        Returns:
            SensorInterface: Configured sensor interface
            
        Raises:
            ImportError: If required backend modules are not available
            RuntimeError: If interface creation fails
        """
        if backend is None:
            backend = self.determine_backend()
        
        config = config or {}
        
        try:
            if backend == HardwareBackend.PHYSICAL:
                return self._create_physical_sensor_interface(config)
            elif backend == HardwareBackend.SIMULATION:
                return self._create_simulation_sensor_interface(config)
            elif backend == HardwareBackend.MOCK:
                return self._create_mock_sensor_interface(config)
            else:
                raise ValueError(f"Unsupported backend: {backend}")
                
        except Exception as e:
            self.logger.error(f"Failed to create {backend.value} sensor interface: {e}")
            # Fallback to mock
            if backend != HardwareBackend.MOCK:
                self.logger.info("Falling back to mock sensor interface")
                return self._create_mock_sensor_interface(config)
            raise
    
    def _create_physical_sensor_interface(self, config: Dict[str, Any]) -> SensorInterface:
        """Create physical sensor interface."""
        try:
            from nevil_hardware.drivers.physical.picarx_sensor_driver import PiCarXSensorDriver
            interface = PiCarXSensorDriver(logger=self.logger, config=config)
            
            if not interface.initialize():
                raise RuntimeError("Failed to initialize physical sensors")
            
            self.logger.info("Physical sensor interface created successfully")
            return interface
            
        except ImportError as e:
            raise ImportError(f"Physical sensor driver not available: {e}")
    
    def _create_simulation_sensor_interface(self, config: Dict[str, Any]) -> SensorInterface:
        """Create simulation sensor interface."""
        try:
            from nevil_hardware.interfaces.sensor_interface import MockSensorInterface
            interface = MockSensorInterface()
            
            if not interface.initialize():
                raise RuntimeError("Failed to initialize simulation sensors")
            
            self.logger.info("Simulation sensor interface created successfully")
            return interface
            
        except ImportError as e:
            raise ImportError(f"Simulation sensor driver not available: {e}")
    
    def _create_mock_sensor_interface(self, config: Dict[str, Any]) -> SensorInterface:
        """Create mock sensor interface for testing."""
        from nevil_hardware.interfaces.sensor_interface import MockSensorInterface
        interface = MockSensorInterface()
        
        if not interface.initialize():
            raise RuntimeError("Failed to initialize mock sensors")
        
        self.logger.info("Mock sensor interface created successfully")
        return interface
    
    def get_sensor_interface(self,
                           backend_type: Optional[str] = None,
                           force_simulation: bool = False,
                           config: Optional[Dict[str, Any]] = None) -> SensorInterface:
        """
        Get or create sensor interface with caching.
        
        Args:
            backend_type: Requested backend type
            force_simulation: Force simulation mode
            config: Configuration parameters
            
        Returns:
            SensorInterface: Sensor interface instance
        """
        requested_backend = None
        if backend_type:
            try:
                requested_backend = HardwareBackend(backend_type.lower())
            except ValueError:
                self.logger.warning(f"Invalid backend type: {backend_type}")
        
        # Determine backend
        backend = self.determine_backend(
            requested_backend=backend_type,
            force_simulation=force_simulation
        )
        
        # Create new interface if needed
        if (self._sensor_interface is None or
            self._current_backend != backend):
            
            # Cleanup existing interface
            if self._sensor_interface:
                self._sensor_interface.cleanup()
            
            # Create new interface
            self._sensor_interface = self.create_sensor_interface(backend, config)
        
        return self._sensor_interface
    
    def cleanup(self) -> None:
        """Clean up all hardware interfaces."""
        if self._movement_interface:
            self._movement_interface.cleanup()
            self._movement_interface = None
        
        if self._sensor_interface:
            self._sensor_interface.cleanup()
            self._sensor_interface = None
            
        self._current_backend = None
        self._shared_picarx = None
        
        self.logger.info("Hardware manager cleaned up")
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current hardware manager status.
        
        Returns:
            Dict[str, Any]: Status information
        """
        status = {
            'current_backend': self._current_backend.value if self._current_backend else None,
            'movement_interface_active': self._movement_interface is not None,
            'sensor_interface_active': self._sensor_interface is not None,
            'hardware_availability': self.detect_hardware_availability()
        }
        
        if self._movement_interface:
            status['movement_status'] = self._movement_interface.get_status().__dict__
        
        if self._sensor_interface:
            status['sensor_status'] = self._sensor_interface.get_status().__dict__
        
        return status


# Convenience functions for ROS2 integration
def create_hardware_manager_from_params(node) -> HardwareManager:
    """
    Create hardware manager from ROS2 node parameters.
    
    Args:
        node: ROS2 node with parameters
        
    Returns:
        HardwareManager: Configured hardware manager
    """
    logger = node.get_logger()
    return HardwareManager(logger=logger)


def get_movement_interface_from_params(node) -> MovementInterface:
    """
    Get movement interface from ROS2 node parameters.
    
    Args:
        node: ROS2 node with parameters
        
    Returns:
        MovementInterface: Configured movement interface
    """
    # Get parameters
    backend_type = node.declare_parameter('hardware_backend', 'auto').value
    force_simulation = node.declare_parameter('force_simulation', False).value
    
    # Create hardware manager
    manager = create_hardware_manager_from_params(node)
    
    # Get interface
    return manager.get_movement_interface(
        backend_type=backend_type if backend_type != 'auto' else None,
        force_simulation=force_simulation
    )


def get_sensor_interface_from_params(node) -> SensorInterface:
    """
    Get sensor interface from ROS2 node parameters.
    
    Args:
        node: ROS2 node with parameters
        
    Returns:
        SensorInterface: Configured sensor interface
    """
    # Get parameters
    backend_type = node.declare_parameter('hardware_backend', 'auto').value
    force_simulation = node.declare_parameter('force_simulation', False).value
    
    # Create hardware manager
    manager = create_hardware_manager_from_params(node)
    
    # Get interface
    return manager.get_sensor_interface(
        backend_type=backend_type if backend_type != 'auto' else None,
        force_simulation=force_simulation
    )


def get_both_interfaces_from_params(node) -> tuple[MovementInterface, SensorInterface]:
    """
    Get both movement and sensor interfaces from ROS2 node parameters.
    
    Args:
        node: ROS2 node with parameters
        
    Returns:
        tuple[MovementInterface, SensorInterface]: Configured interfaces
    """
    # Get parameters
    backend_type = node.declare_parameter('hardware_backend', 'auto').value
    force_simulation = node.declare_parameter('force_simulation', False).value
    
    # Create hardware manager
    manager = create_hardware_manager_from_params(node)
    
    # Get interfaces
    movement = manager.get_movement_interface(
        backend_type=backend_type if backend_type != 'auto' else None,
        force_simulation=force_simulation
    )
    
    sensor = manager.get_sensor_interface(
        backend_type=backend_type if backend_type != 'auto' else None,
        force_simulation=force_simulation
    )
    
    return movement, sensor