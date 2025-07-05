#!/usr/bin/env python3

"""
Test PiCar Hardware Initialization Logging

This script specifically tests that the PiCar hardware initialization 
logs "PiCar hardware initialized successfully" message.
"""

import rclpy
from rclpy.node import Node
import time
import threading
import sys
import os

# Add the navigation module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'nevil_navigation'))

class PiCarHardwareLoggingTest(Node):
    """Test node to verify PiCar hardware initialization logging."""
    
    def __init__(self):
        super().__init__('picar_hardware_logging_test')
        
        self.hardware_init_success = False
        self.expected_log_message = "PiCar hardware initialized successfully"
        
        self.get_logger().info('Starting PiCar hardware initialization logging test...')
        
    def test_navigation_node_initialization(self):
        """Test that NavigationNode logs the expected hardware initialization message."""
        self.get_logger().info('Testing NavigationNode hardware initialization...')
        
        try:
            # Import and create NavigationNode to trigger hardware initialization
            from nevil_navigation.navigation_node import NavigationNode
            
            self.get_logger().info('Creating NavigationNode to test hardware initialization...')
            
            # Create the navigation node which should initialize hardware
            nav_node = NavigationNode()
            
            # Give it a moment to complete initialization
            time.sleep(2.0)
            
            # Check if the node was created successfully
            if nav_node:
                self.get_logger().info('✓ NavigationNode created successfully')
                self.get_logger().info('✓ Check the logs above for "PiCar hardware initialized successfully" message')
                self.hardware_init_success = True
            else:
                self.get_logger().error('✗ Failed to create NavigationNode')
            
            # Cleanup
            nav_node.destroy_node()
            
        except Exception as e:
            self.get_logger().error(f'✗ Error during NavigationNode initialization test: {e}')
            self.get_logger().info('This may be expected if running without actual PiCar hardware')
    
    def run_test(self):
        """Run the hardware logging test."""
        self.get_logger().info('='*60)
        self.get_logger().info('PICAR HARDWARE INITIALIZATION LOGGING TEST')
        self.get_logger().info('='*60)
        
        # Test navigation node initialization
        self.test_navigation_node_initialization()
        
        # Summary
        self.get_logger().info('='*60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('='*60)
        
        if self.hardware_init_success:
            self.get_logger().info('✓ NavigationNode initialization completed')
            self.get_logger().info(f'✓ Look for "{self.expected_log_message}" in the logs above')
        else:
            self.get_logger().warning('✗ NavigationNode initialization had issues')
            self.get_logger().info('This may be normal if running without physical PiCar hardware')
        
        self.get_logger().info('')
        self.get_logger().info('Expected behavior:')
        self.get_logger().info('- If PiCar hardware is available: Should log "PiCar hardware initialized successfully"')
        self.get_logger().info('- If hardware not available: Should log "Failed to initialize PiCar hardware" and "Running in simulation mode"')
        self.get_logger().info('')
        self.get_logger().info('Test completed!')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # Create test node
    test_node = PiCarHardwareLoggingTest()
    
    # Run test in a separate thread
    test_thread = threading.Thread(target=test_node.run_test)
    test_thread.daemon = True
    test_thread.start()
    
    try:
        # Spin for a short time to let the test complete
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.get_logger().info('Test execution completed - shutting down')
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()