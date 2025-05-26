#!/usr/bin/env python3

"""
Integration Test for Nevil-picar v2.0

This script runs integration tests to verify that all components work together correctly.
"""

import os
import sys
import time
import yaml
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nevil_interfaces.msg import SystemStatus
from ament_index_python.packages import get_package_share_directory

class IntegrationTestNode(Node):
    """Integration test node for Nevil-picar v2.0."""
    
    def __init__(self):
        """Initialize the integration test node."""
        super().__init__('integration_test')
        
        # Initialize variables
        self.system_status = None
        self.system_mode = None
        self.test_result = None
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            SystemStatus,
            '/nevil/system/status',
            self.status_callback,
            10
        )
        
        # Create publishers
        self.mode_pub = self.create_publisher(
            String,
            '/nevil/system/set_mode',
            10
        )
        
        self.get_logger().info('Integration test node initialized')
    
    def status_callback(self, msg):
        """Callback for system status messages."""
        self.system_status = msg
        self.system_mode = msg.mode
    
    def set_mode(self, mode):
        """Set the system mode."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        
        # Wait for mode change
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.system_mode == mode:
                self.get_logger().info(f'System mode changed to {mode}')
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().warning(f'Timeout waiting for mode change to {mode}')
        return False
    
    def wait_for_status(self, timeout=5.0):
        """Wait for a status message."""
        start_time = time.time()
        while self.system_status is None and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.system_status is not None
    
    def run_test_system_startup(self):
        """Test system startup."""
        self.get_logger().info('Running system startup test...')
        
        # Wait for status message
        if not self.wait_for_status():
            self.get_logger().error('Failed to receive status message')
            return False
        
        # Check initial mode
        if self.system_mode != 'standby':
            self.get_logger().error(f'Unexpected initial mode: {self.system_mode}')
            return False
        
        # Check system status
        if not self.system_status.ok:
            self.get_logger().error(f'System status not OK: {self.system_status.error_message}')
            return False
        
        self.get_logger().info('System startup test passed')
        return True
    
    def run_test_mode_changes(self):
        """Test mode changes."""
        self.get_logger().info('Running mode changes test...')
        
        # Test changing to manual mode
        if not self.set_mode('manual'):
            self.get_logger().error('Failed to change to manual mode')
            return False
        
        # Test changing to autonomous mode
        if not self.set_mode('autonomous'):
            self.get_logger().error('Failed to change to autonomous mode')
            return False
        
        # Test changing back to standby mode
        if not self.set_mode('standby'):
            self.get_logger().error('Failed to change back to standby mode')
            return False
        
        self.get_logger().info('Mode changes test passed')
        return True
    
    def run_all_tests(self):
        """Run all integration tests."""
        self.get_logger().info('Running all integration tests...')
        
        # Run tests
        test_results = {
            'system_startup': self.run_test_system_startup(),
            'mode_changes': self.run_test_mode_changes()
        }
        
        # Check if all tests passed
        all_passed = all(test_results.values())
        
        if all_passed:
            self.get_logger().info('All integration tests passed')
        else:
            self.get_logger().error('Some integration tests failed')
            for test_name, result in test_results.items():
                status = 'PASSED' if result else 'FAILED'
                self.get_logger().info(f'Test {test_name}: {status}')
        
        self.test_result = all_passed
        return all_passed


class IntegrationTest(unittest.TestCase):
    """Integration test class for Nevil-picar v2.0."""
    
    @classmethod
    def setUpClass(cls):
        """Set up the test class."""
        # Initialize ROS
        rclpy.init()
        
        # Create test node
        cls.test_node = IntegrationTestNode()
        
        # Wait for system to initialize
        time.sleep(2.0)
    
    @classmethod
    def tearDownClass(cls):
        """Tear down the test class."""
        # Clean up
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_integration(self):
        """Run integration tests."""
        # Run all tests
        result = self.test_node.run_all_tests()
        
        # Assert that all tests passed
        self.assertTrue(result, "Integration tests failed")


def main(args=None):
    """Main entry point."""
    # Run the tests
    unittest.main()


if __name__ == '__main__':
    main()