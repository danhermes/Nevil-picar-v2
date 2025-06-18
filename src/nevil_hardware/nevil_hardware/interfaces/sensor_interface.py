#!/usr/bin/env python3

"""
Sensor Interface for Nevil-picar v2.0

This module defines the abstract interface for sensor control,
enabling swappable implementations for physical hardware and simulation.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import time


@dataclass
class UltrasonicReading:
    """Ultrasonic sensor reading."""
    distance_cm: float
    timestamp: float
    valid: bool = True
    error_message: Optional[str] = None


@dataclass
class GrayscaleReading:
    """Grayscale sensor reading."""
    values: List[float]  # List of 3 sensor values
    line_detected: bool = False
    cliff_detected: bool = False
    timestamp: float = 0.0
    valid: bool = True
    error_message: Optional[str] = None


@dataclass
class CameraStatus:
    """Camera status information."""
    pan_angle: float = 0.0  # -90 to 90 degrees
    tilt_angle: float = 0.0  # -35 to 65 degrees
    is_active: bool = False
    timestamp: float = 0.0
    error_message: Optional[str] = None


@dataclass
class SensorStatus:
    """Overall sensor system status."""
    ultrasonic_available: bool = False
    grayscale_available: bool = False
    camera_available: bool = False
    last_update_time: float = 0.0
    error_message: Optional[str] = None
    backend_type: str = "unknown"


class SensorInterface(ABC):
    """
    Abstract interface for sensor control.
    
    This interface defines the contract that all sensor implementations
    must follow, whether physical hardware or simulation.
    """
    
    def __init__(self):
        """Initialize the sensor interface."""
        self._status = SensorStatus()
        self._camera_status = CameraStatus()
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the sensor system.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_ultrasonic_reading(self) -> Optional[UltrasonicReading]:
        """
        Get ultrasonic distance reading.
        
        Returns:
            Optional[UltrasonicReading]: Distance reading or None if unavailable
        """
        pass
    
    @abstractmethod
    def get_grayscale_reading(self) -> Optional[GrayscaleReading]:
        """
        Get grayscale sensor reading.
        
        Returns:
            Optional[GrayscaleReading]: Grayscale reading or None if unavailable
        """
        pass
    
    @abstractmethod
    def set_camera_angles(self, pan: float, tilt: float) -> bool:
        """
        Set camera pan and tilt angles.
        
        Args:
            pan: Pan angle in degrees (-90 to 90)
            tilt: Tilt angle in degrees (-35 to 65)
            
        Returns:
            bool: True if successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_camera_status(self) -> CameraStatus:
        """
        Get current camera status.
        
        Returns:
            CameraStatus: Current camera status
        """
        pass
    
    @abstractmethod
    def calibrate_grayscale(self, reference_values: List[float]) -> bool:
        """
        Calibrate grayscale sensors with reference values.
        
        Args:
            reference_values: List of 3 reference values for calibration
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_status(self) -> SensorStatus:
        """
        Get current sensor system status.
        
        Returns:
            SensorStatus: Current status of sensor system
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Clean up sensor resources."""
        pass
    
    # Convenience methods
    def is_obstacle_detected(self, threshold_cm: float = 20.0) -> bool:
        """Check if obstacle is detected within threshold distance."""
        reading = self.get_ultrasonic_reading()
        if reading and reading.valid:
            return reading.distance_cm < threshold_cm
        return False
    
    def is_line_detected(self) -> bool:
        """Check if line is detected by grayscale sensors."""
        reading = self.get_grayscale_reading()
        if reading and reading.valid:
            return reading.line_detected
        return False
    
    def is_cliff_detected(self) -> bool:
        """Check if cliff is detected by grayscale sensors."""
        reading = self.get_grayscale_reading()
        if reading and reading.valid:
            return reading.cliff_detected
        return False


class MockSensorInterface(SensorInterface):
    """
    Mock implementation for testing.
    
    This implementation provides simulated sensor readings
    for testing and development without hardware.
    """
    
    def __init__(self):
        super().__init__()
        self._status.backend_type = "mock"
        self._status.ultrasonic_available = True
        self._status.grayscale_available = True
        self._status.camera_available = True
        
        # Mock sensor state
        self._mock_distance = 50.0
        self._mock_grayscale = [500.0, 500.0, 500.0]
        self._grayscale_reference = [1000.0, 1000.0, 1000.0]
    
    def initialize(self) -> bool:
        """Mock initialization always succeeds."""
        return True
    
    def get_ultrasonic_reading(self) -> Optional[UltrasonicReading]:
        """Return mock ultrasonic reading."""
        return UltrasonicReading(
            distance_cm=self._mock_distance,
            timestamp=time.time(),
            valid=True
        )
    
    def get_grayscale_reading(self) -> Optional[GrayscaleReading]:
        """Return mock grayscale reading."""
        # Simple line detection logic
        line_detected = any(val < ref * 0.7 for val, ref in 
                           zip(self._mock_grayscale, self._grayscale_reference))
        
        # Simple cliff detection logic  
        cliff_detected = any(val < 300.0 for val in self._mock_grayscale)
        
        return GrayscaleReading(
            values=self._mock_grayscale.copy(),
            line_detected=line_detected,
            cliff_detected=cliff_detected,
            timestamp=time.time(),
            valid=True
        )
    
    def set_camera_angles(self, pan: float, tilt: float) -> bool:
        """Mock camera control - just update status."""
        self._camera_status.pan_angle = max(-90.0, min(90.0, pan))
        self._camera_status.tilt_angle = max(-35.0, min(65.0, tilt))
        self._camera_status.timestamp = time.time()
        return True
    
    def get_camera_status(self) -> CameraStatus:
        """Return current mock camera status."""
        return self._camera_status
    
    def calibrate_grayscale(self, reference_values: List[float]) -> bool:
        """Mock calibration - just store reference values."""
        if len(reference_values) == 3:
            self._grayscale_reference = reference_values.copy()
            return True
        return False
    
    def get_status(self) -> SensorStatus:
        """Return current mock status."""
        self._status.last_update_time = time.time()
        return self._status
    
    def cleanup(self) -> None:
        """Mock cleanup - nothing to do."""
        pass
    
    # Mock control methods for testing
    def set_mock_distance(self, distance: float) -> None:
        """Set mock ultrasonic distance for testing."""
        self._mock_distance = distance
    
    def set_mock_grayscale(self, values: List[float]) -> None:
        """Set mock grayscale values for testing."""
        if len(values) == 3:
            self._mock_grayscale = values.copy()