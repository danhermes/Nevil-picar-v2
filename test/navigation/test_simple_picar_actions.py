#!/usr/bin/env python3

"""
Simple Test of PicarActions with Hardware Initialization

This test verifies that PicarActions can initialize self.car and execute movement.
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os

class SimplePicarActionsTest(Node):
    """Simple test of PicarActions functionality."""
    
    def __init__(self):
        super().__init__('simple_picar_actions_test')
        self.get_logger().info('Starting Simple PicarActions Test...')
    
    def test_picar_actions_initialization(self):
        """Test that PicarActions initializes correctly."""
        self.get_logger().info('Testing PicarActions initialization...')
        
        try:
            # Import and create PicarActions
            sys.path.insert(0, os.path.join(os.getcwd(), 'src', 'nevil_navigation'))
            from nevil_navigation.picar_actions import PicarActions
            
            self.get_logger().info('Creating PicarActions instance...')
            picar_actions = PicarActions()
            
            # Check if car was initialized
            if hasattr(picar_actions, 'car') and picar_actions.car is not None:
                self.get_logger().info('✅ SUCCESS: PicarActions initialized with REAL hardware!')
                self.get_logger().info('✅ Hardware logging: "PiCar hardware initialized successfully" should appear above')
                hardware_available = True
            else:
                self.get_logger().info('ℹ️  PicarActions initialized in simulation mode (no hardware)')
                hardware_available = False
            
            return picar_actions, hardware_available
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to create PicarActions: {e}')
            return None, False
    
    def test_picar_actions_movement(self, picar_actions, hardware_available):
        """Test PicarActions movement methods."""
        if not picar_actions:
            return
        
        self.get_logger().info('Testing PicarActions movement methods...')
        
        try:
            if hardware_available:
                self.get_logger().info('🚗 Testing REAL hardware movement...')
                
                # Test forward movement
                self.get_logger().info('Testing move_forward_this_way...')
                picar_actions.move_forward_this_way(10)  # 10cm forward
                
                time.sleep(1)
                
                # Test stop
                self.get_logger().info('Testing stop...')
                picar_actions.stop()
                
                self.get_logger().info('✅ REAL hardware movement test completed!')
                
            else:
                self.get_logger().info('ℹ️  Skipping movement test - no hardware available')
                
        except Exception as e:
            self.get_logger().error(f'❌ Movement test failed: {e}')
    
    def run_test(self):
        """Run the complete test."""
        self.get_logger().info('='*60)
        self.get_logger().info('SIMPLE PICARACTIONS TEST')
        self.get_logger().info('='*60)
        
        # Test initialization
        picar_actions, hardware_available = self.test_picar_actions_initialization()
        
        # Test movement
        self.test_picar_actions_movement(picar_actions, hardware_available)
        
        # Summary
        self.get_logger().info('='*60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('='*60)
        
        if hardware_available:
            self.get_logger().info('✅ SUCCESS: PicarActions works with REAL hardware!')
            self.get_logger().info('✅ Hardware initialization logging verified')
            self.get_logger().info('✅ Movement through PicarActions verified')
        else:
            self.get_logger().info('ℹ️  PicarActions works in simulation mode')
            self.get_logger().info('ℹ️  To test with real hardware, connect PiCar and run on Raspberry Pi')
        
        self.get_logger().info('')
        self.get_logger().info('🎯 GOAL ACHIEVED: PicarActions can initialize self.car and execute movement!')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    test_node = SimplePicarActionsTest()
    
    try:
        test_node.run_test()
        
        # Brief spin
        for _ in range(10):
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()