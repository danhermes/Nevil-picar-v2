#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nevil_interfaces.msg import SystemStatus
from nevil_interfaces.srv import CheckObstacle

from nevil_testing.test_base import NevilTestBase

class TestSystemManagerNode(NevilTestBase):
    """
    Unit tests for the system_manager_node in the nevil_core package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a subscription to system status
        self.system_status = None
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            self.system_status_callback,
            10
        )
        
        # Wait for the system manager node to be ready
        self.wait_for_system_manager()
    
    def system_status_callback(self, msg):
        """Callback for system status messages."""
        self.system_status = msg
    
    def wait_for_system_manager(self, timeout_sec=5.0):
        """Wait for the system manager node to be ready."""
        # Wait for system status
        if not self.spin_until(lambda: self.system_status is not None, timeout_sec):
            self.fail('Timed out waiting for system status')
    
    def test_system_status_publishing(self):
        """Test that the system manager publishes system status."""
        # Verify that we received a system status message
        self.assertIsNotNone(self.system_status)
        
        # Verify the system status message fields
        self.assertIsNotNone(self.system_status.header)
        self.assertIsNotNone(self.system_status.mode)
        self.assertIsNotNone(self.system_status.battery_level)
        self.assertIsNotNone(self.system_status.system_ok)
    
    def test_system_mode_change(self):
        """Test changing the system mode."""
        # Create a publisher to the mode change topic
        mode_pub = self.create_publisher(String, '/system/change_mode', 10)
        
        # Publish a mode change message
        mode_msg = String()
        mode_msg.data = 'autonomous'
        mode_pub.publish(mode_msg)
        
        # Wait for the mode to change
        def check_mode():
            return self.system_status is not None and self.system_status.mode == 'autonomous'
        
        if not self.spin_until(check_mode, 1.0):
            self.fail('Timed out waiting for mode change')
        
        # Verify the mode changed
        self.assertEqual(self.system_status.mode, 'autonomous')
        
        # Change back to standby mode
        mode_msg.data = 'standby'
        mode_pub.publish(mode_msg)
        
        # Wait for the mode to change back
        def check_mode_back():
            return self.system_status is not None and self.system_status.mode == 'standby'
        
        if not self.spin_until(check_mode_back, 1.0):
            self.fail('Timed out waiting for mode change back')
        
        # Verify the mode changed back
        self.assertEqual(self.system_status.mode, 'standby')
    
    def test_error_handling(self):
        """Test error handling in the system manager."""
        # Create a publisher to the error topic
        error_pub = self.create_publisher(String, '/system/error', 10)
        
        # Publish an error message
        error_msg = String()
        error_msg.data = 'test_error'
        error_pub.publish(error_msg)
        
        # Wait for the error to be processed
        def check_error():
            return (self.system_status is not None and 
                    self.system_status.has_errors and 
                    'test_error' in self.system_status.error_codes)
        
        if not self.spin_until(check_error, 1.0):
            self.fail('Timed out waiting for error to be processed')
        
        # Verify the error was processed
        self.assertTrue(self.system_status.has_errors)
        self.assertIn('test_error', self.system_status.error_codes)
    
    def test_node_monitoring(self):
        """Test that the system manager monitors other nodes."""
        # Create a test node
        test_node = self.create_test_node('test_monitored_node')
        
        # Add the node to the executor
        self.executor.add_node(test_node)
        
        # Wait for the node to be detected
        time.sleep(2.0)  # Give time for the system manager to detect the node
        
        # Verify the node is in the active nodes list
        self.assertIn('test_monitored_node', self.system_status.active_nodes)
        
        # Destroy the test node
        self.executor.remove_node(test_node)
        test_node.destroy_node()
        
        # Wait for the node to be removed from the active nodes list
        def check_node_removed():
            return (self.system_status is not None and 
                    'test_monitored_node' not in self.system_status.active_nodes)
        
        if not self.spin_until(check_node_removed, 3.0):
            self.fail('Timed out waiting for node to be removed from active nodes list')
        
        # Verify the node is not in the active nodes list
        self.assertNotIn('test_monitored_node', self.system_status.active_nodes)
    
    def test_performance_metrics(self):
        """Test that the system manager reports performance metrics."""
        # Verify that performance metrics are reported
        self.assertGreaterEqual(self.system_status.cpu_usage, 0.0)
        self.assertLessEqual(self.system_status.cpu_usage, 1.0)
        self.assertGreaterEqual(self.system_status.memory_usage, 0.0)
        self.assertLessEqual(self.system_status.memory_usage, 1.0)
        self.assertGreaterEqual(self.system_status.disk_usage, 0.0)
        self.assertLessEqual(self.system_status.disk_usage, 1.0)

if __name__ == '__main__':
    unittest.main()