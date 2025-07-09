#!/usr/bin/env python3
"""
Nevil 2.0 Servo Calibration Utility

This script helps diagnose and fix servo calibration issues,
particularly the 45-degree left listing problem.
"""

import os
import sys
import time
import argparse
from pathlib import Path

# Add the navigation module to path
sys.path.append(str(Path(__file__).parent / "src" / "nevil_navigation"))

def check_permissions():
    """Check if we have the necessary permissions"""
    print("🔍 Checking permissions...")
    
    # Check if running as root or with sudo
    if os.geteuid() == 0:
        print("✅ Running with root privileges")
        return True
    
    # Check GPIO access
    gpio_devices = ['/dev/gpiomem', '/dev/mem']
    for device in gpio_devices:
        if os.path.exists(device) and os.access(device, os.R_OK | os.W_OK):
            print(f"✅ GPIO access available via {device}")
            return True
    
    print("❌ Insufficient permissions for hardware access")
    print("💡 Please run with sudo: sudo python3 calibrate_servos.py")
    return False

def diagnose_configuration():
    """Diagnose configuration file issues"""
    print("\n🔍 Diagnosing configuration...")
    
    config_path = '/opt/picar-x/picar-x.conf'
    
    if not os.path.exists(config_path):
        print(f"❌ Config file missing: {config_path}")
        return False
    
    print(f"✅ Config file exists: {config_path}")
    
    # Check permissions
    readable = os.access(config_path, os.R_OK)
    writable = os.access(config_path, os.W_OK)
    
    print(f"📖 Readable: {readable}")
    print(f"✏️  Writable: {writable}")
    
    if readable:
        try:
            with open(config_path, 'r') as f:
                content = f.read()
                print(f"📄 Config file content:")
                for line in content.split('\n'):
                    if line.strip() and not line.startswith('#'):
                        print(f"   {line}")
        except Exception as e:
            print(f"❌ Error reading config: {e}")
            return False
    
    return readable and writable

def create_backup_config():
    """Create a backup of the current configuration"""
    config_path = '/opt/picar-x/picar-x.conf'
    backup_path = f"{config_path}.backup.{int(time.time())}"
    
    try:
        if os.path.exists(config_path):
            import shutil
            shutil.copy2(config_path, backup_path)
            print(f"✅ Config backed up to: {backup_path}")
            return backup_path
    except Exception as e:
        print(f"⚠️  Could not create backup: {e}")
    
    return None

def reset_configuration():
    """Reset configuration to defaults"""
    print("\n🔄 Resetting configuration to defaults...")
    
    config_path = '/opt/picar-x/picar-x.conf'
    
    # Create backup first
    create_backup_config()
    
    default_config = """# PiCar-X Configuration File
# Reset by Nevil 2.0 Calibration Utility

# Servo calibration values (degrees)
picarx_dir_servo=0
picarx_cam_pan_servo=0
picarx_cam_tilt_servo=0

# Motor calibration values
picarx_dir_motor=[1, 1]

# Sensor calibration values
line_reference=[1000, 1000, 1000]
cliff_reference=[500, 500, 500]
"""
    
    try:
        # Ensure directory exists
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        
        # Write new config
        with open(config_path, 'w') as f:
            f.write(default_config)
        
        # Set permissions
        os.chmod(config_path, 0o666)
        
        print(f"✅ Configuration reset successfully")
        return True
        
    except Exception as e:
        print(f"❌ Failed to reset configuration: {e}")
        return False

def test_servo_movement():
    """Test servo movement and calibration"""
    print("\n🧪 Testing servo movement...")
    
    try:
        from nevil_navigation.enhanced_picarx import create_enhanced_picarx
        
        print("🤖 Creating Enhanced PicarX instance...")
        car = create_enhanced_picarx()
        
        print("🔍 Running diagnostics...")
        diagnosis = car.diagnose_servo_issue()
        
        print(f"\n📊 Diagnosis Results:")
        for key, value in diagnosis.items():
            status = "✅" if value else "❌"
            print(f"   {status} {key}: {value}")
        
        print("\n🔄 Testing servo positions...")
        
        # Test direction servo
        print("Testing direction servo...")
        car.set_dir_servo_angle(0)  # Center
        time.sleep(1)
        
        print("Moving left...")
        car.set_dir_servo_angle(-15)  # Left
        time.sleep(1)
        
        print("Moving right...")
        car.set_dir_servo_angle(15)   # Right
        time.sleep(1)
        
        print("Returning to center...")
        car.set_dir_servo_angle(0)    # Center
        time.sleep(1)
        
        # Test camera servos
        print("Testing camera servos...")
        car.set_cam_pan_angle(0)
        car.set_cam_tilt_angle(0)
        time.sleep(1)
        
        print("✅ Servo test completed")
        return True
        
    except Exception as e:
        print(f"❌ Servo test failed: {e}")
        return False

def interactive_calibration():
    """Interactive servo calibration"""
    print("\n🎯 Interactive Servo Calibration")
    print("This will help you calibrate the direction servo to fix the 45-degree issue.")
    
    try:
        from nevil_navigation.enhanced_picarx import create_enhanced_picarx
        
        car = create_enhanced_picarx()
        
        print("\n📐 Current servo position should be straight ahead.")
        print("If the wheels are turned left, we need positive calibration.")
        print("If the wheels are turned right, we need negative calibration.")
        
        while True:
            try:
                current_cal = getattr(car, 'dir_cali_val', 0)
                print(f"\nCurrent calibration: {current_cal}")
                
                user_input = input("\nEnter calibration value (-45 to 45, 'q' to quit, 't' to test): ").strip()
                
                if user_input.lower() == 'q':
                    break
                elif user_input.lower() == 't':
                    print("Testing current position...")
                    car.set_dir_servo_angle(0)
                    continue
                
                cal_value = float(user_input)
                
                if not -45 <= cal_value <= 45:
                    print("❌ Value must be between -45 and 45")
                    continue
                
                print(f"🔧 Applying calibration: {cal_value}")
                success = car.recalibrate_direction_servo(cal_value)
                
                if success:
                    print("✅ Calibration applied successfully")
                    print("🔍 Check if the wheels are now straight")
                else:
                    print("❌ Calibration failed")
                
            except ValueError:
                print("❌ Please enter a valid number")
            except KeyboardInterrupt:
                break
        
        print("\n🔄 Resetting to neutral position...")
        car.reset_to_neutral()
        
        return True
        
    except Exception as e:
        print(f"❌ Interactive calibration failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Nevil 2.0 Servo Calibration Utility")
    parser.add_argument('--diagnose', action='store_true', help='Run diagnostics only')
    parser.add_argument('--reset-config', action='store_true', help='Reset configuration to defaults')
    parser.add_argument('--test', action='store_true', help='Test servo movement')
    parser.add_argument('--calibrate', action='store_true', help='Interactive calibration')
    parser.add_argument('--fix-45-degree', action='store_true', help='Attempt to fix 45-degree left issue')
    
    args = parser.parse_args()
    
    print("🤖 Nevil 2.0 Servo Calibration Utility")
    print("=" * 50)
    
    # Check permissions first
    if not check_permissions():
        return 1
    
    # Diagnose configuration
    config_ok = diagnose_configuration()
    
    if args.diagnose:
        print(f"\n📊 Configuration Status: {'✅ OK' if config_ok else '❌ Issues Found'}")
        return 0 if config_ok else 1
    
    if args.reset_config:
        if reset_configuration():
            print("✅ Configuration reset completed")
        else:
            print("❌ Configuration reset failed")
            return 1
    
    if args.test:
        if test_servo_movement():
            print("✅ Servo test completed successfully")
        else:
            print("❌ Servo test failed")
            return 1
    
    if args.calibrate:
        if interactive_calibration():
            print("✅ Interactive calibration completed")
        else:
            print("❌ Interactive calibration failed")
            return 1
    
    if args.fix_45_degree:
        print("\n🔧 Attempting to fix 45-degree left issue...")
        
        # Reset config first
        if not reset_configuration():
            print("❌ Could not reset configuration")
            return 1
        
        # Test servo movement
        if not test_servo_movement():
            print("❌ Servo test failed after reset")
            return 1
        
        print("✅ 45-degree issue fix attempt completed")
        print("💡 If wheels are still not straight, run: python3 calibrate_servos.py --calibrate")
    
    # If no specific action, show help
    if not any([args.diagnose, args.reset_config, args.test, args.calibrate, args.fix_45_degree]):
        print("\n💡 Usage examples:")
        print("  sudo python3 calibrate_servos.py --diagnose")
        print("  sudo python3 calibrate_servos.py --fix-45-degree")
        print("  sudo python3 calibrate_servos.py --calibrate")
        print("  sudo python3 calibrate_servos.py --test")
        print("  sudo python3 calibrate_servos.py --reset-config")
    
    return 0

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\n🛑 Calibration interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1)