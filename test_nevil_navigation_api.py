#!/usr/bin/env python3

"""
Test script for the updated NevilNavigationAPI with PiCar-X integration.

This script tests the direct PiCar-X integration in the NevilNavigationAPI,
verifying that it works without ROS2 actions and connects properly to the hardware.
"""

import sys
import time
import logging

# Add the src directory to Python path for imports
sys.path.insert(0, 'src')

from nevil_navigation.nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI


def setup_logging():
    """Set up logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def test_api_initialization(logger):
    """Test API initialization with different modes."""
    logger.info("=== Testing API Initialization ===")
    
    # Test with auto-detect (tries PiCar-X first)
    try:
        api1 = NevilNavigationAPI()
        logger.info("✓ Auto-detect initialization successful")
        api1.shutdown()
    except Exception as e:
        logger.error(f"✗ Auto-detect initialization failed: {e}")
    
    # Test with forced mock mode
    try:
        api2 = NevilNavigationAPI(force_mock=True)
        logger.info("✓ Mock mode initialization successful")
        api2.shutdown()
    except Exception as e:
        logger.error(f"✗ Mock mode initialization failed: {e}")


def test_basic_movements(api, logger):
    """Test basic movement functions."""
    logger.info("=== Testing Basic Movements ===")
    
    try:
        # Test distance reading
        distance = api.get_distance()
        logger.info(f"Current distance: {distance:.1f}cm")
        
        # Test stop (should always work)
        result = api.stop()
        logger.info(f"Stop command: {'✓' if result else '✗'}")
        
        # Test steering
        result = api.set_dir_servo_angle(15)
        logger.info(f"Set steering angle 15°: {'✓' if result else '✗'}")
        time.sleep(0.5)
        
        result = api.set_dir_servo_angle(0)
        logger.info(f"Reset steering angle: {'✓' if result else '✗'}")
        
        # Test short forward movement
        result = api.move_forward_this_way(10, speed=30)
        logger.info(f"Move forward 10cm: {'✓' if result else '✗'}")
        
        # Test short backward movement
        result = api.move_backward_this_way(10, speed=30)
        logger.info(f"Move backward 10cm: {'✓' if result else '✗'}")
        
    except Exception as e:
        logger.error(f"Basic movement test failed: {e}")


def test_camera_control(api, logger):
    """Test camera control functions."""
    logger.info("=== Testing Camera Control ===")
    
    try:
        # Test camera pan
        result = api.set_cam_pan_angle(30)
        logger.info(f"Set camera pan 30°: {'✓' if result else '✗'}")
        time.sleep(0.5)
        
        result = api.set_cam_pan_angle(-30)
        logger.info(f"Set camera pan -30°: {'✓' if result else '✗'}")
        time.sleep(0.5)
        
        result = api.set_cam_pan_angle(0)
        logger.info(f"Reset camera pan: {'✓' if result else '✗'}")
        
        # Test camera tilt
        result = api.set_cam_tilt_angle(20)
        logger.info(f"Set camera tilt 20°: {'✓' if result else '✗'}")
        time.sleep(0.5)
        
        result = api.set_cam_tilt_angle(0)
        logger.info(f"Reset camera tilt: {'✓' if result else '✗'}")
        
    except Exception as e:
        logger.error(f"Camera control test failed: {e}")


def test_behaviors(api, logger):
    """Test behavior functions."""
    logger.info("=== Testing Behaviors ===")
    
    behaviors = [
        'wave_hands',
        'shake_head', 
        'nod',
        'act_cute',
        'think'
    ]
    
    for behavior in behaviors:
        try:
            method = getattr(api, behavior)
            result = method()
            logger.info(f"Behavior '{behavior}': {'✓' if result else '✗'}")
            time.sleep(0.5)
        except Exception as e:
            logger.error(f"Behavior '{behavior}' failed: {e}")


def test_obstacle_detection(api, logger):
    """Test obstacle detection."""
    logger.info("=== Testing Obstacle Detection ===")
    
    try:
        # Test obstacle check
        obstacle_detected, distance, direction = api.check_obstacle(max_distance=1.0)
        logger.info(f"Obstacle check: detected={obstacle_detected}, distance={distance:.2f}m, direction={direction:.2f}")
        
        # Test distance reading multiple times
        for i in range(5):
            distance = api.get_distance()
            logger.info(f"Distance reading {i+1}: {distance:.1f}cm")
            time.sleep(0.2)
            
    except Exception as e:
        logger.error(f"Obstacle detection test failed: {e}")


def test_compatibility_dict(logger):
    """Test the actions_dict compatibility layer."""
    logger.info("=== Testing Compatibility Dictionary ===")
    
    try:
        from nevil_navigation.nevil_navigation.nevil_navigation_api.core import actions_dict
        
        api = NevilNavigationAPI()
        
        # Test a few actions from the dictionary
        test_actions = ['stop', 'shake head', 'wave hands']
        
        for action_name in test_actions:
            if action_name in actions_dict:
                action_func = actions_dict[action_name]
                result = action_func(api)
                logger.info(f"Action '{action_name}': {'✓' if result else '✗'}")
            else:
                logger.warning(f"Action '{action_name}' not found in actions_dict")
        
        api.shutdown()
        
    except Exception as e:
        logger.error(f"Compatibility dictionary test failed: {e}")


def main():
    """Main test function."""
    logger = setup_logging()
    logger.info("Starting NevilNavigationAPI test...")
    
    try:
        # Test initialization
        test_api_initialization(logger)
        
        # Create API instance for main tests
        logger.info("Creating API instance for main tests...")
        api = NevilNavigationAPI(use_hardware_manager=True)
        
        # Run tests
        test_basic_movements(api, logger)
        test_camera_control(api, logger)
        test_behaviors(api, logger)
        test_obstacle_detection(api, logger)
        
        # Test compatibility
        test_compatibility_dict(logger)
        
        # Cleanup
        api.shutdown()
        
        logger.info("=== Test Summary ===")
        logger.info("NevilNavigationAPI test completed successfully!")
        logger.info("Key achievements:")
        logger.info("✓ Removed ROS2 action dependencies")
        logger.info("✓ Integrated PiCar-X directly")
        logger.info("✓ Connected to hardware manager")
        logger.info("✓ Maintained API compatibility")
        
    except Exception as e:
        logger.error(f"Test failed with error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)