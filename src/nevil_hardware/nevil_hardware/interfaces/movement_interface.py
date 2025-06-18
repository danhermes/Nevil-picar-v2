#!/usr/bin/env python3

"""
Movement Interface for Nevil-picar v2.0

This module defines the abstract interface for movement control,
enabling swappable implementations for physical hardware and simulation.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
import time


@dataclass
class MovementStatus:
    """Status information for movement system."""
    is_moving: bool = False
    left_motor_speed: float = 0.0  # -1.0 to 1.0
    right_motor_speed: float = 0.0  # -1.0 to 1.0
    steering_angle: float = 0.0  # -30 to 30 degrees
    last_command_time: float = 0.0
    error_message: Optional[str] = None
    hardware_available: bool = True
    backend_type: str = "unknown"  # "physical", "simulation", "mock"


@dataclass
class MovementCommand:
    """Command structure for movement operations."""
    left_speed: float = 0.0  # -1.0 to 1.0
    right_speed: float = 0.0  # -1.0 to 1.0
    steering_angle: float = 0.0  # -30 to 30 degrees
    duration: Optional[float] = None  # seconds, None for indefinite
    timestamp: float = 0.0


class MovementInterface(ABC):
    """
    Abstract interface for movement control.
    
    This interface defines the contract that all movement implementations
    must follow, whether physical hardware or simulation.
    """
    
    def __init__(self):
        """Initialize the movement interface."""
        self._status = MovementStatus()
        self._last_command = MovementCommand()
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the movement system.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        pass
    
    @abstractmethod
    def set_motor_speeds(self, left: float, right: float) -> bool:
        """
        Set motor speeds for differential drive.
        
        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        pass
    
    @abstractmethod
    def set_steering_angle(self, angle: float) -> bool:
        """
        Set steering angle for front wheels.
        
        Args:
            angle: Steering angle in degrees (-30 to 30)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """
        Stop all movement immediately.
        
        Returns:
            bool: True if stop successful, False otherwise
        """
        pass
    
    @abstractmethod
    def reset(self) -> bool:
        """
        Reset movement system to default state.
        
        Returns:
            bool: True if reset successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_status(self) -> MovementStatus:
        """
        Get current movement status.
        
        Returns:
            MovementStatus: Current status of movement system
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Clean up resources and stop movement."""
        pass
    
    # Convenience methods with default implementations
    def forward(self, speed: float) -> bool:
        """Move forward at specified speed."""
        return self.set_motor_speeds(speed, speed)
    
    def backward(self, speed: float) -> bool:
        """Move backward at specified speed."""
        return self.set_motor_speeds(-speed, -speed)
    
    def turn_left(self, speed: float) -> bool:
        """Turn left in place."""
        return self.set_motor_speeds(-speed, speed)
    
    def turn_right(self, speed: float) -> bool:
        """Turn right in place."""
        return self.set_motor_speeds(speed, -speed)
    
    def is_available(self) -> bool:
        """Check if movement system is available."""
        return self.get_status().hardware_available
    
    def get_backend_type(self) -> str:
        """Get the backend type (physical/simulation/mock)."""
        return self.get_status().backend_type


class MockMovementInterface(MovementInterface):
    """
    Mock implementation for testing.
    
    This implementation does nothing but provides valid responses
    for testing and development without hardware.
    """
    
    def __init__(self):
        super().__init__()
        self._status.backend_type = "mock"
        self._status.hardware_available = True
    
    def initialize(self) -> bool:
        """Mock initialization always succeeds."""
        return True
    
    def set_motor_speeds(self, left: float, right: float) -> bool:
        """Mock motor control - just update status."""
        self._status.left_motor_speed = max(-1.0, min(1.0, left))
        self._status.right_motor_speed = max(-1.0, min(1.0, right))
        self._status.is_moving = abs(left) > 0.01 or abs(right) > 0.01
        self._status.last_command_time = time.time()
        return True
    
    def set_steering_angle(self, angle: float) -> bool:
        """Mock steering control - just update status."""
        self._status.steering_angle = max(-30.0, min(30.0, angle))
        self._status.last_command_time = time.time()
        return True
    
    def stop(self) -> bool:
        """Mock stop - reset speeds."""
        self._status.left_motor_speed = 0.0
        self._status.right_motor_speed = 0.0
        self._status.is_moving = False
        self._status.last_command_time = time.time()
        return True
    
    def reset(self) -> bool:
        """Mock reset - reset all values."""
        self._status.left_motor_speed = 0.0
        self._status.right_motor_speed = 0.0
        self._status.steering_angle = 0.0
        self._status.is_moving = False
        self._status.last_command_time = time.time()
        return True
    
    def get_status(self) -> MovementStatus:
        """Return current mock status."""
        return self._status
    
    def cleanup(self) -> None:
        """Mock cleanup - nothing to do."""
        pass