#!/usr/bin/env python3

"""
PiCar-X Physical Sensor Driver for Nevil-picar v2.0

This module provides a physical hardware implementation of the SensorInterface
using the PiCar-X robot platform sensors including ultrasonic, grayscale, and camera.
"""

import sys
import os
import time
import logging
from typing import Optional, List, Dict, Any

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

from nevil_hardware.interfaces.sensor_interface import (
    SensorInterface, SensorStatus, UltrasonicReading, 
    GrayscaleReading, CameraStatus
)


class PiCarXSensorDriver(SensorInterface):
    """
    Physical sensor driver for PiCar-X robot platform.
    
    This driver provides direct hardware sensor access using the picarx library,
    implementing the SensorInterface for seamless integration with the
    Nevil robot system.
    """
    
    # PiCar-X sensor limits and defaults
    ULTRASONIC_MAX_DISTANCE = 300.0  # cm
    ULTRASONIC_MIN_DISTANCE = 2.0    # cm
    GRAYSCALE_SENSOR_COUNT = 3
    DEFAULT_LINE_REFERENCE = [1000.0, 1000.0, 1000.0]
    DEFAULT_CLIFF_REFERENCE = [500.0, 500.0, 500.0]
    
    def __init__(self, 
                 logger: Optional[logging.Logger] = None,
                 config: Optional[Dict[str, Any]] = None):
        """
        Initialize the PiCar-X sensor driver.
        
        Args:
            logger: Logger instance for debugging
            config: Configuration parameters
        """
        super().__init__()
        self.logger = logger or logging.getLogger(__name__)
        self.config = config or {}
        
        # PiCar-X instance (shared with movement driver if available)
        self._picarx: Optional[Picarx] = None
        self._owns_picarx = False  # Track if we created the instance
        
        # Driver state
        self._initialized = False
        
        # Sensor configuration
        self._ultrasonic_timeout = self.config.get('ultrasonic_timeout', 0.5)
        self._grayscale_update_rate = self.config.get('grayscale_update_rate', 10.0)  # Hz
        self._line_reference = self.config.get('line_reference', self.DEFAULT_LINE_REFERENCE.copy())
        self._cliff_reference = self.config.get('cliff_reference', self.DEFAULT_CLIFF_REFERENCE.copy())
        
        # Update status
        self._status.backend_type = "physical"
        self._status.ultrasonic_available = PICARX_AVAILABLE
        self._status.grayscale_available = PICARX_AVAILABLE
        self._status.camera_available = PICARX_AVAILABLE
        
        if not PICARX_AVAILABLE:
            self._status.error_message = "PiCar-X library not available"
            self.logger.error("PiCar-X library not available - cannot use physical sensor driver")
    
    def initialize(self, picarx_instance: Optional[Picarx] = None) -> bool:
        """
        Initialize the PiCar-X sensors.
        
        Args:
            picarx_instance: Existing PiCar-X instance to use (optional)
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        if not PICARX_AVAILABLE:
            self.logger.error("Cannot initialize: PiCar-X library not available")
            return False
        
        try:
            self.logger.info("Initializing PiCar-X sensors...")
            
            # Use provided instance or create new one
            if picarx_instance:
                self._picarx = picarx_instance
                self._owns_picarx = False
            else:
                self._picarx = Picarx()
                self._owns_picarx = True
            
            # Set up grayscale sensor references
            self._picarx.set_line_reference(self._line_reference)
            self._picarx.set_cliff_reference(self._cliff_reference)
            
            # Test sensors
            if not self._test_sensors():
                self.logger.error("Sensor test failed")
                return False
            
            self._initialized = True
            self._status.error_message = None
            
            self.logger.info("PiCar-X sensors initialized successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize PiCar-X sensors: {e}")
            self._status.error_message = f"Sensor initialization failed: {e}"
            return False
    
    def _test_sensors(self) -> bool:
        """
        Test basic sensor functionality.
        
        Returns:
            bool: True if sensor test passes
        """
        try:
            # Test ultrasonic sensor
            distance = self._picarx.get_distance()
            if distance is None or distance < 0:
                self.logger.warning("Ultrasonic sensor test failed")
                self._status.ultrasonic_available = False
            
            # Test grayscale sensors
            grayscale_data = self._picarx.get_grayscale_data()
            if not grayscale_data or len(grayscale_data) != 3:
                self.logger.warning("Grayscale sensor test failed")
                self._status.grayscale_available = False
            
            # Camera servos are tested during movement initialization
            
            return True
            
        except Exception as e:
            self.logger.error(f"Sensor test failed: {e}")
            return False
    
    def get_ultrasonic_reading(self) -> Optional[UltrasonicReading]:
        """
        Get ultrasonic distance reading.
        
        Returns:
            Optional[UltrasonicReading]: Distance reading or None if unavailable
        """
        if not self._check_ready() or not self._status.ultrasonic_available:
            return None
        
        try:
            distance = self._picarx.get_distance()
            
            if distance is None:
                return UltrasonicReading(
                    distance_cm=0.0,
                    timestamp=time.time(),
                    valid=False,
                    error_message="No reading from sensor"
                )
            
            # Validate reading
            valid = (self.ULTRASONIC_MIN_DISTANCE <= distance <= self.ULTRASONIC_MAX_DISTANCE)
            
            return UltrasonicReading(
                distance_cm=distance,
                timestamp=time.time(),
                valid=valid,
                error_message=None if valid else "Reading out of range"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to get ultrasonic reading: {e}")
            return UltrasonicReading(
                distance_cm=0.0,
                timestamp=time.time(),
                valid=False,
                error_message=f"Sensor error: {e}"
            )
    
    def get_grayscale_reading(self) -> Optional[GrayscaleReading]:
        """
        Get grayscale sensor reading.
        
        Returns:
            Optional[GrayscaleReading]: Grayscale reading or None if unavailable
        """
        if not self._check_ready() or not self._status.grayscale_available:
            return None
        
        try:
            # Get raw grayscale data
            grayscale_data = self._picarx.get_grayscale_data()
            
            if not grayscale_data or len(grayscale_data) != 3:
                return GrayscaleReading(
                    values=[0.0, 0.0, 0.0],
                    timestamp=time.time(),
                    valid=False,
                    error_message="Invalid grayscale data"
                )
            
            # Check for line detection
            line_detected = self._picarx.get_line_status(grayscale_data)
            
            # Check for cliff detection
            cliff_detected = self._picarx.get_cliff_status(grayscale_data)
            
            return GrayscaleReading(
                values=grayscale_data.copy(),
                line_detected=line_detected,
                cliff_detected=cliff_detected,
                timestamp=time.time(),
                valid=True
            )
            
        except Exception as e:
            self.logger.error(f"Failed to get grayscale reading: {e}")
            return GrayscaleReading(
                values=[0.0, 0.0, 0.0],
                timestamp=time.time(),
                valid=False,
                error_message=f"Sensor error: {e}"
            )
    
    def set_camera_angles(self, pan: float, tilt: float) -> bool:
        """
        Set camera pan and tilt angles.
        
        Args:
            pan: Pan angle in degrees (-90 to 90)
            tilt: Tilt angle in degrees (-35 to 65)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self._check_ready() or not self._status.camera_available:
            return False
        
        try:
            # Constrain angles to PiCar-X limits
            pan = max(-90.0, min(90.0, pan))
            tilt = max(-35.0, min(65.0, tilt))
            
            # Set camera angles
            self._picarx.set_cam_pan_angle(pan)
            self._picarx.set_cam_tilt_angle(tilt)
            
            # Update camera status
            self._camera_status.pan_angle = pan
            self._camera_status.tilt_angle = tilt
            self._camera_status.is_active = True
            self._camera_status.timestamp = time.time()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set camera angles: {e}")
            self._camera_status.error_message = f"Camera control failed: {e}"
            return False
    
    def get_camera_status(self) -> CameraStatus:
        """
        Get current camera status.
        
        Returns:
            CameraStatus: Current camera status
        """
        return self._camera_status
    
    def calibrate_grayscale(self, reference_values: List[float]) -> bool:
        """
        Calibrate grayscale sensors with reference values.
        
        Args:
            reference_values: List of 3 reference values for calibration
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self._check_ready() or not self._status.grayscale_available:
            return False
        
        if len(reference_values) != 3:
            self.logger.error("Grayscale calibration requires exactly 3 reference values")
            return False
        
        try:
            # Update line reference
            self._line_reference = reference_values.copy()
            self._picarx.set_line_reference(self._line_reference)
            
            self.logger.info(f"Grayscale calibrated with reference: {reference_values}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to calibrate grayscale sensors: {e}")
            return False
    
    def set_cliff_reference(self, reference_values: List[float]) -> bool:
        """
        Set cliff detection reference values.
        
        Args:
            reference_values: List of 3 reference values for cliff detection
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self._check_ready() or not self._status.grayscale_available:
            return False
        
        if len(reference_values) != 3:
            self.logger.error("Cliff reference requires exactly 3 values")
            return False
        
        try:
            self._cliff_reference = reference_values.copy()
            self._picarx.set_cliff_reference(self._cliff_reference)
            
            self.logger.info(f"Cliff reference set to: {reference_values}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set cliff reference: {e}")
            return False
    
    def get_status(self) -> SensorStatus:
        """
        Get current sensor system status.
        
        Returns:
            SensorStatus: Current status of sensor system
        """
        self._status.last_update_time = time.time()
        return self._status
    
    def cleanup(self) -> None:
        """Clean up sensor resources."""
        try:
            if self._picarx and self._owns_picarx:
                self.logger.info("Cleaning up PiCar-X sensors...")
                # Reset camera to center position
                self._picarx.set_cam_pan_angle(0)
                self._picarx.set_cam_tilt_angle(0)
                time.sleep(0.2)
            
            if self._owns_picarx:
                self._picarx = None
            
            self._initialized = False
            
        except Exception as e:
            self.logger.error(f"Error during sensor cleanup: {e}")
    
    def _check_ready(self) -> bool:
        """
        Check if the sensor driver is ready.
        
        Returns:
            bool: True if ready, False otherwise
        """
        if not PICARX_AVAILABLE:
            self._status.error_message = "PiCar-X library not available"
            return False
        
        if not self._initialized or not self._picarx:
            self._status.error_message = "Sensor driver not initialized"
            return False
        
        return True
    
    # Additional utility methods
    def get_line_reference(self) -> List[float]:
        """Get current line detection reference values."""
        return self._line_reference.copy()
    
    def get_cliff_reference(self) -> List[float]:
        """Get current cliff detection reference values."""
        return self._cliff_reference.copy()
    
    def reset_camera_position(self) -> bool:
        """Reset camera to center position."""
        return self.set_camera_angles(0.0, 0.0)


# Factory function for easy instantiation
def create_picarx_sensor_driver(logger: Optional[logging.Logger] = None,
                               config: Optional[Dict[str, Any]] = None,
                               picarx_instance: Optional[Picarx] = None) -> PiCarXSensorDriver:
    """
    Create and initialize a PiCar-X sensor driver.
    
    Args:
        logger: Logger instance
        config: Configuration parameters
        picarx_instance: Existing PiCar-X instance to share
        
    Returns:
        PiCarXSensorDriver: Initialized driver instance
    """
    driver = PiCarXSensorDriver(logger=logger, config=config)
    
    if not driver.initialize(picarx_instance=picarx_instance):
        raise RuntimeError("Failed to initialize PiCar-X sensor driver")
    
    return driver