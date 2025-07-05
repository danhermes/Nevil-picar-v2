#!/usr/bin/env python3

"""
Test Direct Hardware Access for Nevil-picar v2.0

This script tests direct hardware access using the v1.0 picarx library.
"""

import sys
import os

# Add the picarlibs directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'v1.0/picarlibs'))

def test_robot_hat_import():
    """Test if robot_hat can be imported."""
    try:
        from robot_hat import Pin, PWM, Servo, utils
        print("✓ robot_hat library imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Failed to import robot_hat: {e}")
        return False

def test_picarx_initialization():
    """Test if Picarx can be initialized."""
    try:
        from picarx import Picarx
        print("✓ Picarx class imported successfully")
        
        print("Attempting to initialize Picarx hardware...")
        px = Picarx()
        print("✓ Picarx hardware initialized successfully")
        
        # Test basic motor control
        print("Testing motor control...")
        px.set_motor_speed(1, 50)  # Left motor forward
        px.set_motor_speed(2, -50)  # Right motor backward
        print("✓ Motor commands sent successfully")
        
        # Stop motors
        px.stop()
        print("✓ Motors stopped")
        
        # Test servo control
        print("Testing servo control...")
        px.set_dir_servo_angle(15)  # Turn right
        print("✓ Servo command sent successfully")
        
        # Reset to center
        px.set_dir_servo_angle(0)
        print("✓ Servo reset to center")
        
        return True
        
    except ImportError as e:
        print(f"✗ Failed to import Picarx: {e}")
        return False
    except Exception as e:
        print(f"✗ Failed to initialize Picarx hardware: {e}")
        print("This is expected if running on a non-PiCar-X system")
        return False

def test_hardware_permissions():
    """Test if we have the necessary permissions for hardware access."""
    try:
        # Check if we can access GPIO
        import os
        if os.path.exists('/dev/gpiomem'):
            print("✓ GPIO device found at /dev/gpiomem")
        else:
            print("✗ GPIO device not found - may need to run on Raspberry Pi")
        
        # Check if we're running as root or in gpio group
        import pwd
        import grp
        
        current_user = pwd.getpwuid(os.getuid()).pw_name
        print(f"Running as user: {current_user}")
        
        try:
            gpio_group = grp.getgrnam('gpio')
            if current_user in [member for member in gpio_group.gr_mem]:
                print("✓ User is in gpio group")
            else:
                print("✗ User is not in gpio group - may need to add user to gpio group")
        except KeyError:
            print("✗ gpio group not found")
        
        return True
        
    except Exception as e:
        print(f"✗ Error checking hardware permissions: {e}")
        return False

def main():
    """Main test function."""
    print("=== Nevil-picar v2.0 Direct Hardware Test ===\n")
    
    print("1. Testing robot_hat library import...")
    robot_hat_ok = test_robot_hat_import()
    print()
    
    print("2. Testing hardware permissions...")
    permissions_ok = test_hardware_permissions()
    print()
    
    print("3. Testing Picarx initialization...")
    picarx_ok = test_picarx_initialization()
    print()
    
    print("=== Test Summary ===")
    print(f"robot_hat import: {'✓' if robot_hat_ok else '✗'}")
    print(f"Hardware permissions: {'✓' if permissions_ok else '✗'}")
    print(f"Picarx initialization: {'✓' if picarx_ok else '✗'}")
    
    if robot_hat_ok and picarx_ok:
        print("\n🎉 Hardware is ready for real movement!")
        print("The RT hardware interface should work in hardware mode.")
    elif robot_hat_ok:
        print("\n⚠️  robot_hat is available but Picarx initialization failed.")
        print("This is normal if not running on actual PiCar-X hardware.")
        print("The system will run in simulation mode.")
    else:
        print("\n❌ robot_hat library is not available.")
        print("Install robot_hat library for hardware support:")
        print("pip install robot-hat")

if __name__ == '__main__':
    main()