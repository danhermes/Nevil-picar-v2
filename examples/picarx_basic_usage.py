#!/usr/bin/env python3

"""
Basic PiCar-X Usage Example for Nevil-picar v2.0

This example demonstrates how to use the PiCar-X integration with the Nevil robot system
for basic movement and sensor operations.
"""

import sys
import time
import logging

# Add the src directory to Python path for imports
sys.path.insert(0, '../src')

from nevil_hardware.hardware_manager import HardwareManager


def setup_logging():
    """Set up basic logging."""
    logging.basicConfig(level=logging.INFO)
    return logging.getLogger(__name__)


def basic_movement_demo(movement, logger):
    """Demonstrate basic movement capabilities."""
    logger.info("=== Basic Movement Demo ===")
    
    if not movement.is_available():
        logger.warning("Movement interface not available")
        return
    
    try:
        # Move forward
        logger.info("Moving forward...")
        movement.forward(0.5)
        time.sleep(2)
        
        # Turn left
        logger.info("Turning left...")
        movement.set_steering_angle(-20)
        movement.forward(0.4)
        time.sleep(2)
        
        # Turn right
        logger.info("Turning right...")
        movement.set_steering_angle(20)
        movement.forward(0.4)
        time.sleep(2)
        
        # Stop and reset
        logger.info("Stopping...")
        movement.stop()
        movement.reset()
        
    except Exception as e:
        logger.error(f"Movement demo failed: {e}")
        movement.stop()


def sensor_monitoring_demo(sensor, logger):
    """Demonstrate sensor monitoring capabilities."""
    logger.info("=== Sensor Monitoring Demo ===")
    
    try:
        # Monitor sensors for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10:
            # Get ultrasonic reading
            ultrasonic = sensor.get_ultrasonic_reading()
            if ultrasonic and ultrasonic.valid:
                distance = ultrasonic.distance_cm
                logger.info(f"Distance: {distance:.1f}cm")
                
                # Check for obstacles
                if distance < 20:
                    logger.warning("Obstacle detected!")
            
            # Get grayscale reading
            grayscale = sensor.get_grayscale_reading()
            if grayscale and grayscale.valid:
                if grayscale.line_detected:
                    logger.info("Line detected!")
                if grayscale.cliff_detected:
                    logger.warning("Cliff detected!")
            
            time.sleep(0.5)
            
    except Exception as e:
        logger.error(f"Sensor demo failed: {e}")


def obstacle_avoidance_demo(movement, sensor, logger):
    """Demonstrate simple obstacle avoidance."""
    logger.info("=== Obstacle Avoidance Demo ===")
    
    if not movement.is_available():
        logger.warning("Movement interface not available")
        return
    
    try:
        # Simple obstacle avoidance for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            # Get distance reading
            ultrasonic = sensor.get_ultrasonic_reading()
            
            if ultrasonic and ultrasonic.valid:
                distance = ultrasonic.distance_cm
                
                if distance > 40:
                    # Clear path - move forward
                    movement.set_steering_angle(0)
                    movement.forward(0.4)
                    logger.info(f"Moving forward (distance: {distance:.1f}cm)")
                    
                elif distance > 20:
                    # Obstacle ahead - turn right
                    movement.set_steering_angle(25)
                    movement.forward(0.3)
                    logger.info(f"Turning right (distance: {distance:.1f}cm)")
                    
                else:
                    # Too close - back up and turn
                    movement.backward(0.3)
                    time.sleep(0.5)
                    movement.set_steering_angle(-30)
                    movement.forward(0.3)
                    logger.warning(f"Backing up and turning (distance: {distance:.1f}cm)")
            else:
                # No sensor reading - stop for safety
                movement.stop()
                logger.warning("No sensor reading - stopping")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("Obstacle avoidance demo interrupted by user")
    except Exception as e:
        logger.error(f"Obstacle avoidance demo failed: {e}")
    finally:
        movement.stop()


def camera_control_demo(sensor, logger):
    """Demonstrate camera control."""
    logger.info("=== Camera Control Demo ===")
    
    try:
        # Pan camera left and right
        logger.info("Panning camera...")
        positions = [
            (0, 0),      # Center
            (45, 0),     # Right
            (0, 30),     # Up
            (-45, 0),    # Left
            (0, -20),    # Down
            (0, 0)       # Back to center
        ]
        
        for pan, tilt in positions:
            logger.info(f"Setting camera to pan={pan}°, tilt={tilt}°")
            sensor.set_camera_angles(pan, tilt)
            time.sleep(1)
            
            # Get camera status
            status = sensor.get_camera_status()
            logger.info(f"Camera at pan={status.pan_angle}°, tilt={status.tilt_angle}°")
            
    except Exception as e:
        logger.error(f"Camera demo failed: {e}")


def main():
    """Main demonstration function."""
    logger = setup_logging()
    logger.info("Starting PiCar-X basic usage demo...")
    
    try:
        # Create hardware manager
        manager = HardwareManager(logger=logger)
        
        # Get interfaces
        movement = manager.get_movement_interface()
        sensor = manager.get_sensor_interface()
        
        logger.info(f"Using {movement.get_backend_type()} movement interface")
        logger.info(f"Using {sensor.get_status().backend_type} sensor interface")
        
        # Run demos
        basic_movement_demo(movement, logger)
        time.sleep(1)
        
        sensor_monitoring_demo(sensor, logger)
        time.sleep(1)
        
        camera_control_demo(sensor, logger)
        time.sleep(1)
        
        # Ask user if they want to run obstacle avoidance
        try:
            response = input("\nRun obstacle avoidance demo? (y/N): ").strip().lower()
            if response == 'y':
                logger.info("Starting obstacle avoidance demo (Ctrl+C to stop)...")
                obstacle_avoidance_demo(movement, sensor, logger)
        except KeyboardInterrupt:
            logger.info("Demo interrupted by user")
        
        logger.info("Demo completed successfully!")
        
    except Exception as e:
        logger.error(f"Demo failed: {e}")
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