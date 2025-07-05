#!/usr/bin/env python3

"""
PiCar-X Setup Script for Nevil-picar v2.0

This script helps set up and verify the PiCar-X integration with the Nevil robot system.
It checks dependencies, tests hardware connectivity, and provides setup guidance.
"""

import os
import sys
import subprocess
import importlib
import logging
from pathlib import Path


def setup_logging():
    """Set up logging for the setup script."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)s: %(message)s'
    )
    return logging.getLogger(__name__)


def check_python_version(logger):
    """Check if Python version is compatible."""
    logger.info("Checking Python version...")
    
    if sys.version_info < (3, 7):
        logger.error(f"Python 3.7+ required, found {sys.version}")
        return False
    
    logger.info(f"Python version: {sys.version}")
    return True


def check_picarx_library(logger):
    """Check if PiCar-X library is available."""
    logger.info("Checking PiCar-X library...")
    
    picarx_path = '/home/dan/picar-x'
    
    if not os.path.exists(picarx_path):
        logger.error(f"PiCar-X library not found at {picarx_path}")
        logger.info("Please ensure the PiCar-X library is installed at the correct location")
        return False
    
    # Check if picarx module can be imported
    try:
        if picarx_path not in sys.path:
            sys.path.insert(0, picarx_path)
        
        import picarx
        logger.info("PiCar-X library found and importable")
        return True
        
    except ImportError as e:
        logger.error(f"Failed to import PiCar-X library: {e}")
        return False


def check_dependencies(logger):
    """Check required Python dependencies."""
    logger.info("Checking Python dependencies...")
    
    required_packages = [
        'robot_hat',
        'RPi.GPIO'
    ]
    
    optional_packages = [
        'rclpy',  # For ROS2 integration
        'numpy',  # For sensor data processing
    ]
    
    missing_required = []
    missing_optional = []
    
    for package in required_packages:
        try:
            importlib.import_module(package)
            logger.info(f"✓ {package} found")
        except ImportError:
            logger.warning(f"✗ {package} missing (required)")
            missing_required.append(package)
    
    for package in optional_packages:
        try:
            importlib.import_module(package)
            logger.info(f"✓ {package} found")
        except ImportError:
            logger.info(f"- {package} missing (optional)")
            missing_optional.append(package)
    
    if missing_required:
        logger.error("Missing required dependencies. Install with:")
        for package in missing_required:
            if package == 'robot_hat':
                logger.error("  pip3 install robot-hat")
            elif package == 'RPi.GPIO':
                logger.error("  pip3 install RPi.GPIO")
    
    if missing_optional:
        logger.info("Optional dependencies can be installed with:")
        for package in missing_optional:
            logger.info(f"  pip3 install {package}")
    
    return len(missing_required) == 0


def check_hardware_permissions(logger):
    """Check hardware access permissions."""
    logger.info("Checking hardware permissions...")
    
    # Check GPIO access
    gpio_path = '/dev/gpiomem'
    if os.path.exists(gpio_path):
        if os.access(gpio_path, os.R_OK | os.W_OK):
            logger.info("✓ GPIO access available")
        else:
            logger.warning("✗ GPIO access denied")
            logger.info("Run with sudo or add user to gpio group:")
            logger.info("  sudo usermod -a -G gpio $USER")
            return False
    else:
        logger.warning("GPIO device not found (may not be on Raspberry Pi)")
    
    return True


def test_nevil_integration(logger):
    """Test the Nevil hardware integration."""
    logger.info("Testing Nevil hardware integration...")
    
    try:
        # Add src to path
        src_path = os.path.join(os.path.dirname(__file__), 'src')
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        
        from nevil_hardware.hardware_manager import HardwareManager
        
        # Create hardware manager
        manager = HardwareManager(logger=logger)
        
        # Test hardware detection
        availability = manager.detect_hardware_availability()
        logger.info(f"Hardware availability: {availability}")
        
        # Test backend determination
        backend = manager.determine_backend()
        logger.info(f"Selected backend: {backend.value}")
        
        # Test interface creation
        movement = manager.get_movement_interface()
        sensor = manager.get_sensor_interface()
        
        logger.info(f"Movement interface: {movement.get_backend_type()}")
        logger.info(f"Sensor interface: {sensor.get_status().backend_type}")
        
        # Cleanup
        manager.cleanup()
        
        logger.info("✓ Nevil integration test passed")
        return True
        
    except Exception as e:
        logger.error(f"✗ Nevil integration test failed: {e}")
        return False


def run_basic_test(logger):
    """Run a basic hardware test if available."""
    logger.info("Running basic hardware test...")
    
    try:
        # Add src to path
        src_path = os.path.join(os.path.dirname(__file__), 'src')
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        
        from nevil_hardware.hardware_manager import HardwareManager
        
        manager = HardwareManager(logger=logger)
        movement = manager.get_movement_interface()
        sensor = manager.get_sensor_interface()
        
        if movement.get_backend_type() == 'physical':
            logger.info("Testing physical hardware...")
            
            # Test movement
            logger.info("Testing movement (brief)...")
            movement.forward(0.2)
            import time
            time.sleep(0.5)
            movement.stop()
            
            # Test sensors
            logger.info("Testing sensors...")
            ultrasonic = sensor.get_ultrasonic_reading()
            if ultrasonic:
                logger.info(f"Ultrasonic: {ultrasonic.distance_cm:.1f}cm")
            
            grayscale = sensor.get_grayscale_reading()
            if grayscale:
                logger.info(f"Grayscale: {grayscale.values}")
            
            logger.info("✓ Basic hardware test completed")
        else:
            logger.info("Physical hardware not available, skipping hardware test")
        
        manager.cleanup()
        return True
        
    except Exception as e:
        logger.error(f"✗ Basic hardware test failed: {e}")
        return False


def print_setup_summary(logger, results):
    """Print setup summary and next steps."""
    logger.info("\n" + "="*50)
    logger.info("SETUP SUMMARY")
    logger.info("="*50)
    
    all_passed = all(results.values())
    
    for test, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        logger.info(f"{test:<30} {status}")
    
    if all_passed:
        logger.info("\n🎉 All checks passed! PiCar-X integration is ready to use.")
        logger.info("\nNext steps:")
        logger.info("1. Run the integration test: python3 test_picarx_integration.py")
        logger.info("2. Try the basic usage example: cd examples && python3 picarx_basic_usage.py")
        logger.info("3. Read the documentation: docs/PICARX_INTEGRATION.md")
    else:
        logger.error("\n❌ Some checks failed. Please address the issues above.")
        logger.info("\nFor help:")
        logger.info("- Check docs/PICARX_INTEGRATION.md")
        logger.info("- Ensure PiCar-X hardware is properly connected")
        logger.info("- Install missing dependencies")
        logger.info("- Run with appropriate permissions")


def main():
    """Main setup function."""
    logger = setup_logging()
    logger.info("PiCar-X Setup for Nevil-picar v2.0")
    logger.info("="*50)
    
    # Run all checks
    results = {
        'Python Version': check_python_version(logger),
        'PiCar-X Library': check_picarx_library(logger),
        'Dependencies': check_dependencies(logger),
        'Hardware Permissions': check_hardware_permissions(logger),
        'Nevil Integration': test_nevil_integration(logger),
    }
    
    # Ask user if they want to run hardware test
    if results['Nevil Integration']:
        try:
            response = input("\nRun basic hardware test? (y/N): ").strip().lower()
            if response == 'y':
                results['Hardware Test'] = run_basic_test(logger)
        except KeyboardInterrupt:
            logger.info("\nSetup interrupted by user")
    
    # Print summary
    print_setup_summary(logger, results)
    
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)