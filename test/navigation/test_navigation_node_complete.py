#!/usr/bin/env python3

"""
Complete test of NavigationNode functionality including AI command integration.
This script tests both hardware initialization and AI command message availability.
"""

import sys
import os
import time

# Add navigation module to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'nevil_navigation'))

def test_navigation_node():
    """Test NavigationNode with full functionality"""
    print("=" * 60)
    print("NEVIL PICAR V2.0 - NAVIGATION NODE COMPLETE TEST")
    print("=" * 60)
    
    # Test 1: Import test
    print("\n1. Testing imports...")
    try:
        import rclpy
        from nevil_navigation.navigation_node import NavigationNode
        print("✓ NavigationNode imported successfully")
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        return False
    
    # Test 2: AI Command message availability
    print("\n2. Testing AI command message availability...")
    try:
        from nevil_interfaces_ai_msgs.msg import AICommand
        print("✓ AI command messages available")
        ai_available = True
    except ImportError:
        print("✗ AI command messages not available (ROS2 environment not sourced)")
        ai_available = False
    
    # Test 3: NavigationNode creation
    print("\n3. Testing NavigationNode creation...")
    rclpy.init()
    try:
        node = NavigationNode()
        print("✓ NavigationNode created successfully")
        
        # Test hardware status
        if node.car is not None:
            print("✓ PiCar hardware initialized and available")
            hardware_available = True
        else:
            print("⚠ Running in simulation mode (no hardware)")
            hardware_available = False
        
        # Test AI command subscription
        if hasattr(node, 'action_command_subscription'):
            print("✓ AI command subscription created")
            ai_subscription = True
        else:
            print("⚠ AI command subscription not available")
            ai_subscription = False
        
        # Test action execution capability
        print("\n4. Testing action execution capability...")
        if hardware_available:
            print("Testing basic actions...")
            try:
                # Test a simple action
                node.execute_action('stop', {})
                print("✓ Action execution working")
                
                # Test a movement action
                node.execute_action('forward', {'distance_cm': 5, 'speed': 20})
                time.sleep(0.5)
                node.execute_action('stop', {})
                print("✓ Movement actions working")
                
                # Test a gesture action
                node.execute_action('nod', {})
                print("✓ Gesture actions working")
                
            except Exception as e:
                print(f"⚠ Action execution error: {e}")
        else:
            print("⚠ Skipping action tests (no hardware)")
        
        # Cleanup
        node.destroy_node()
        print("✓ Node destroyed successfully")
        
    except Exception as e:
        print(f"✗ NavigationNode creation failed: {e}")
        return False
    finally:
        rclpy.shutdown()
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Hardware Available: {'✓' if hardware_available else '⚠ Simulation'}")
    print(f"AI Messages Available: {'✓' if ai_available else '✗'}")
    print(f"AI Subscription: {'✓' if ai_subscription else '⚠'}")
    print(f"Action Execution: {'✓' if hardware_available else '⚠ Simulation'}")
    
    if not ai_available:
        print("\n" + "!" * 60)
        print("IMPORTANT: To enable AI command integration, run:")
        print("source install/setup.bash")
        print("Then run this test again.")
        print("!" * 60)
    
    return True

if __name__ == "__main__":
    test_navigation_node()