#!/usr/bin/env python3

"""
Test script for PiCar-X integration with Nevil-picar v2.0

This script tests the integration of the PiCar-X hardware with the Nevil robot system,
verifying both movement and sensor interfaces work correctly.
"""

import sys
import time
import logging
from typing import Optional

# Add the src directory to Python path for imports
sys.path.insert(0, 'src')

from nevil_hardware.hardware_manager import HardwareManager, HardwareBackend
from nevil_hardware.interfaces.movement_interface import MovementInterface, MovementStatus
from nevil_hardware.interfaces.sensor_interface import SensorInterface, SensorStatus


def setup_logging() -> logging.Logger:
    """Set up logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def test_hardware_detection(manager: HardwareManager, logger: logging.Logger) -> None:
    """Test hardware detection capabilities."""
    logger.info("=== Testing Hardware Detection ===")
    
    availability = manager.detect_hardware_availability()
    logger.info(f"Hardware availability: {availability}")
    
    backend = manager.determine_backend()
    logger.info(f"Determined backend: {backend.value}")
    
    status = manager.get_status()
    logger.info(f"Manager status: {status}")


def test_movement_interface(movement: MovementInterface, logger: logging.Logger) -> None:
    """Test movement interface functionality."""
    logger.info("=== Testing Movement Interface ===")
    
    # Get initial status
    status = movement.get_status()
    logger.info(f"Movement status: {status}")
    logger.info(f"Backend type: {movement.get_backend_type()}")
    logger.info(f"Available: {movement.is_available()}")
    
    if not movement.is_available():
        logger.warning("Movement interface not available, skipping movement tests")
        return
    
    try:
        # Test basic movements
        logger.info("Testing forward movement...")
        movement.forward(0.3)
        time.sleep(1)
        
        logger.info("Testing backward movement...")
        movement.backward(0.3)
        time.sleep(1)
        
        logger.info("Testing left turn...")
        movement.turn_left(0.3)
        time.sleep(1)
        
        logger.info("Testing right turn...")
        movement.turn_right(0.3)
        time.sleep(1)
        
        logger.info("Testing steering...")
        movement.set_steering_angle(15.0)
        movement.forward(0.3)
        time.sleep(1)
        
        movement.set_steering_angle(-15.0)
        movement.forward(0.3)
        time.sleep(1)
        
        logger.info("Testing stop...")
        movement.stop()
        
        logger.info("Testing reset...")
        movement.reset()
        
        # Get final status
        final_status = movement.get_status()
        logger.info(f"Final movement status: {final_status}")
        
    except Exception as e:
        logger.error(f"Movement test failed: {e}")
    finally:
        movement.stop()


def test_sensor_interface(sensor: SensorInterface, logger: logging.Logger) -> None:
    """Test sensor interface functionality."""
    logger.info("=== Testing Sensor Interface ===")
    
    # Get initial status
    status = sensor.get_status()
    logger.info(f"Sensor status: {status}")
    
    try:
        # Test ultrasonic sensor
        logger.info("Testing ultrasonic sensor...")
        for i in range(5):
            reading = sensor.get_ultrasonic_reading()
            if reading:
                logger.info(f"Ultrasonic reading {i+1}: {reading.distance_cm:.2f}cm (valid: {reading.valid})")
            else:
                logger.warning(f"No ultrasonic reading {i+1}")
            time.sleep(0.5)
        
        # Test grayscale sensors
        logger.info("Testing grayscale sensors...")
        for i in range(5):
            reading = sensor.get_grayscale_reading()
            if reading:
                logger.info(f"Grayscale reading {i+1}: {reading.values} (line: {reading.line_detected}, cliff: {reading.cliff_detected})")
            else:
                logger.warning(f"No grayscale reading {i+1}")
            time.sleep(0.5)
        
        # Test camera control
        logger.info("Testing camera control...")
        camera_positions = [
            (0, 0),    # Center
            (30, 20),  # Right up
            (-30, -20), # Left down
            (0, 0)     # Back to center
        ]
        
        for pan, tilt in camera_positions:
            logger.info(f"Setting camera to pan={pan}, tilt={tilt}")
            success = sensor.set_camera_angles(pan, tilt)
            if success:
                status = sensor.get_camera_status()
                logger.info(f"Camera status: pan={status.pan_angle}, tilt={status.tilt_angle}")
            else:
                logger.warning(f"Failed to set camera angles")
            time.sleep(1)
        
        # Test convenience methods
        logger.info("Testing convenience methods...")
        obstacle = sensor.is_obstacle_detected(30.0)
        line = sensor.is_line_detected()
        cliff = sensor.is_cliff_detected()
        logger.info(f"Obstacle detected: {obstacle}, Line detected: {line}, Cliff detected: {cliff}")
        
    except Exception as e:
        logger.error(f"Sensor test failed: {e}")


def test_picarx_specific_features(manager: HardwareManager, logger: logging.Logger) -> None:
    """Test PiCar-X specific features if using physical hardware."""
    logger.info("=== Testing PiCar-X Specific Features ===")
    
    try:
        # Try to get physical interfaces
        movement = manager.get_movement_interface(backend_type='physical')
        sensor = manager.get_sensor_interface(backend_type='physical')
        
        if movement.get_backend_type() != 'physical':
            logger.info("Physical hardware not available, skipping PiCar-X specific tests")
            return
        
        # Test PiCar-X specific movement features
        if hasattr(movement, 'get_distance'):
            distance = movement.get_distance()
            logger.info(f"PiCar-X ultrasonic distance: {distance}")
        
        if hasattr(movement, 'get_grayscale_data'):
            grayscale = movement.get_grayscale_data()
            logger.info(f"PiCar-X grayscale data: {grayscale}")
        
        if hasattr(movement, 'set_camera_angle'):
            logger.info("Testing PiCar-X camera control...")
            movement.set_camera_angle(pan=45, tilt=30)
            time.sleep(1)
            movement.set_camera_angle(pan=0, tilt=0)
        
        # Test emergency stop
        if hasattr(movement, 'emergency_stop'):
            logger.info("Testing emergency stop...")
            movement.emergency_stop()
            time.sleep(0.5)
            movement.clear_emergency_stop()
        
    except Exception as e:
        logger.error(f"PiCar-X specific test failed: {e}")


def main():
    """Main test function."""
    logger = setup_logging()
    logger.info("Starting PiCar-X integration test...")
    
    try:
        # Create hardware manager
        manager = HardwareManager(logger=logger)
        
        # Test hardware detection
        test_hardware_detection(manager, logger)
        
        # Get interfaces
        logger.info("Creating interfaces...")
        movement = manager.get_movement_interface()
        sensor = manager.get_sensor_interface()
        
        # Test interfaces
        test_movement_interface(movement, logger)
        test_sensor_interface(sensor, logger)
        
        # Test PiCar-X specific features
        test_picarx_specific_features(manager, logger)
        
        logger.info("=== Test Summary ===")
        final_status = manager.get_status()
        logger.info(f"Final manager status: {final_status}")
        
        logger.info("PiCar-X integration test completed successfully!")
        
    except Exception as e:
        logger.error(f"Test failed with error: {e}")
        return 1
    
    finally:
        # Cleanup
        try:
            if 'manager' in locals():
                manager.cleanup()
        except Exception as e:
            logger.error(f"Cleanup failed: {e}")
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)