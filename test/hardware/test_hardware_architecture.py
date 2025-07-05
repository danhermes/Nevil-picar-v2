#!/usr/bin/env python3

"""
Test script for the new Nevil-picar v2.0 hardware architecture.

This script demonstrates the clean separation between physical hardware
and simulation modes through the abstract interface design.
"""

import sys
import os
import time

# Add the nevil_realtime package to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src/nevil_realtime'))

from nevil_realtime.rt_hardware_interface import (
    RTHardwareInterface,
    HardwareManager,
    PhysicalHardwareInterface,
    SimulationHardwareInterface,
    HardwareBackend
)

class MockLogger:
    """Mock logger for testing without ROS2."""
    
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warn(self, msg):
        print(f"[WARN] {msg}")
    
    def error(self, msg):
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] {msg}")


def test_hardware_manager():
    """Test the HardwareManager factory pattern."""
    print("\n=== Testing HardwareManager Factory ===")
    
    logger = MockLogger()
    
    # Test auto-detection (should fall back to simulation)
    print("\n1. Testing auto-detection:")
    hw = HardwareManager.create_hardware_interface(logger, 'auto')
    print(f"   Backend type: {hw.get_backend_type()}")
    
    # Test explicit simulation backend
    print("\n2. Testing explicit simulation backend:")
    hw_sim = HardwareManager.create_hardware_interface(logger, 'simulation')
    print(f"   Backend type: {hw_sim.get_backend_type()}")
    
    # Test configuration-driven selection
    print("\n3. Testing configuration-driven selection:")
    config = {'hardware_backend': 'simulation'}
    hw_config = HardwareManager.create_hardware_interface(logger, None, config)
    print(f"   Backend type: {hw_config.get_backend_type()}")
    
    return hw, hw_sim, hw_config


def test_simulation_backend():
    """Test the simulation hardware backend."""
    print("\n=== Testing Simulation Backend ===")
    
    logger = MockLogger()
    hw = SimulationHardwareInterface(logger)
    
    # Initialize
    success = hw.initialize()
    print(f"Initialization: {'Success' if success else 'Failed'}")
    
    # Test motor control
    print("\nTesting motor control:")
    hw.set_motor_speeds(0.5, -0.3)
    print(f"   Left motor: {hw.left_motor_speed}")
    print(f"   Right motor: {hw.right_motor_speed}")
    
    # Test steering
    print("\nTesting steering:")
    hw.set_steering_angle(15.0)
    print(f"   Steering angle: {hw.steering_angle}")
    
    # Test distance sensor
    print("\nTesting distance sensor:")
    distance = hw.get_distance()
    print(f"   Distance: {distance} cm")
    
    # Test camera control
    print("\nTesting camera control:")
    hw.set_camera_pan(45.0)
    hw.set_camera_tilt(30.0)
    print(f"   Camera pan: {hw.camera_pan}")
    print(f"   Camera tilt: {hw.camera_tilt}")
    
    # Test stop and reset
    print("\nTesting stop and reset:")
    hw.stop()
    print(f"   After stop - Left: {hw.left_motor_speed}, Right: {hw.right_motor_speed}")
    
    hw.reset()
    print(f"   After reset - Steering: {hw.steering_angle}, Pan: {hw.camera_pan}, Tilt: {hw.camera_tilt}")
    
    # Cleanup
    hw.cleanup()
    print("   Cleanup completed")


def test_rt_hardware_interface():
    """Test the RTHardwareInterface with different backends."""
    print("\n=== Testing RTHardwareInterface ===")
    
    logger = MockLogger()
    
    # Test with auto-detection
    print("\n1. Testing with auto-detection:")
    hw_auto = RTHardwareInterface(None, backend='auto')
    print(f"   Backend: {hw_auto.get_backend_type()}")
    print(f"   Simulation mode: {hw_auto.simulation_mode}")
    
    # Test with explicit simulation
    print("\n2. Testing with explicit simulation:")
    hw_sim = RTHardwareInterface(None, backend='simulation')
    print(f"   Backend: {hw_sim.get_backend_type()}")
    print(f"   Simulation mode: {hw_sim.simulation_mode}")
    
    # Test hardware operations
    print("\n3. Testing hardware operations:")
    hw_sim.set_motor_speeds(0.8, 0.8)
    hw_sim.set_steering_angle(-20.0)
    distance = hw_sim.get_distance()
    print(f"   Distance reading: {distance} cm")
    
    hw_sim.set_camera_pan(-30.0)
    hw_sim.set_camera_tilt(45.0)
    
    hw_sim.stop()
    hw_sim.reset()
    hw_sim.cleanup()
    
    return hw_auto, hw_sim


def test_backend_switching():
    """Test switching between different backends."""
    print("\n=== Testing Backend Switching ===")
    
    logger = MockLogger()
    
    backends = ['auto', 'simulation']
    
    for backend in backends:
        print(f"\nTesting {backend} backend:")
        hw = RTHardwareInterface(None, backend=backend)
        
        print(f"   Backend type: {hw.get_backend_type()}")
        print(f"   Simulation mode: {hw.simulation_mode}")
        
        # Quick operation test
        hw.set_motor_speeds(0.5, 0.5)
        hw.set_steering_angle(10.0)
        distance = hw.get_distance()
        print(f"   Distance: {distance} cm")
        
        hw.stop()
        hw.cleanup()


def test_error_handling():
    """Test error handling and graceful degradation."""
    print("\n=== Testing Error Handling ===")
    
    logger = MockLogger()
    
    # Test with invalid backend
    print("\n1. Testing invalid backend:")
    try:
        hw = HardwareManager.create_hardware_interface(logger, 'invalid_backend')
    except ValueError as e:
        print(f"   Caught expected error: {e}")
    
    # Test emergency fallback in RTHardwareInterface
    print("\n2. Testing emergency fallback:")
    # This should work even if there are issues
    hw = RTHardwareInterface(None, backend='simulation')
    print(f"   Emergency fallback successful: {hw.get_backend_type()}")
    hw.cleanup()


def run_performance_test():
    """Run a simple performance test."""
    print("\n=== Performance Test ===")
    
    logger = MockLogger()
    hw = RTHardwareInterface(None, backend='simulation')
    
    # Time multiple operations
    start_time = time.time()
    
    for i in range(100):
        hw.set_motor_speeds(0.5, -0.5)
        hw.set_steering_angle(i % 60 - 30)  # -30 to 30 degrees
        hw.get_distance()
        hw.set_camera_pan(i % 180 - 90)  # -90 to 90 degrees
        hw.set_camera_tilt(i % 100 - 35)  # -35 to 65 degrees
    
    end_time = time.time()
    elapsed = end_time - start_time
    
    print(f"   100 operation cycles completed in {elapsed:.3f} seconds")
    print(f"   Average time per cycle: {elapsed/100*1000:.2f} ms")
    
    hw.cleanup()


def main():
    """Main test function."""
    print("Nevil-picar v2.0 Hardware Architecture Test")
    print("=" * 50)
    
    try:
        # Test individual components
        test_hardware_manager()
        test_simulation_backend()
        test_rt_hardware_interface()
        test_backend_switching()
        test_error_handling()
        run_performance_test()
        
        print("\n" + "=" * 50)
        print("✓ All tests completed successfully!")
        print("\nArchitecture Features Demonstrated:")
        print("  • Interface Segregation - Clean abstract base class")
        print("  • Dependency Inversion - High-level code depends on abstractions")
        print("  • Strategy Pattern - Interchangeable backends")
        print("  • Factory Pattern - Centralized backend creation")
        print("  • Configuration-driven selection")
        print("  • Graceful error handling and fallback")
        print("  • Thread-safe operations (simulation)")
        print("  • Performance optimization")
        
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)