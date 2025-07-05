#!/usr/bin/env python3

"""
Test script to verify system mode is set to 'active' instead of 'standby'.
This will help confirm that the navigation node can receive and execute actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SystemModeChecker(Node):
    """Simple node to check the current system mode."""
    
    def __init__(self):
        super().__init__('system_mode_checker')
        
        # Subscribe to system mode
        self.mode_sub = self.create_subscription(
            String,
            '/system_mode',
            self.mode_callback,
            10
        )
        
        self.received_mode = None
        self.get_logger().info('System Mode Checker initialized - waiting for system mode...')
    
    def mode_callback(self, msg):
        """Handle system mode messages."""
        self.received_mode = msg.data
        self.get_logger().info(f'Received system mode: {self.received_mode}')
        
        if self.received_mode == 'active':
            self.get_logger().info('✓ SUCCESS: System is in ACTIVE mode - navigation should work!')
        elif self.received_mode == 'standby':
            self.get_logger().error('✗ PROBLEM: System is still in STANDBY mode - navigation will be blocked!')
        else:
            self.get_logger().warning(f'? UNKNOWN: System mode is "{self.received_mode}"')

def main():
    rclpy.init()
    
    checker = SystemModeChecker()
    
    try:
        # Wait for a few seconds to receive the mode
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(checker, timeout_sec=0.1)
            if checker.received_mode:
                break
        
        if not checker.received_mode:
            checker.get_logger().error('No system mode received - system manager may not be running')
        
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()