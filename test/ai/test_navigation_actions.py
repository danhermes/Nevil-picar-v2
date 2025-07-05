#!/usr/bin/env python3

"""
Test script to verify navigation node action execution.
This script tests that the navigation node can receive and execute AI action commands.
"""

import rclpy
from rclpy.node import Node
from nevil_interfaces_ai_msgs.msg import AICommand
from std_msgs.msg import String
import json
import time

class NavigationActionTester(Node):
    """Test node to send action commands to the navigation node."""
    
    def __init__(self):
        super().__init__('navigation_action_tester')
        
        # Create publishers
        self.action_command_pub = self.create_publisher(
            AICommand,
            '/nevil/action_command',
            10
        )
        
        self.system_mode_pub = self.create_publisher(
            String,
            '/system_mode',
            10
        )
        
        self.get_logger().info('Navigation Action Tester initialized')
    
    def publish_system_mode(self, mode):
        """Publish system mode."""
        msg = String()
        msg.data = mode
        self.system_mode_pub.publish(msg)
        self.get_logger().info(f'Published system mode: {mode}')
    
    def send_action_command(self, action_type, action_data=None):
        """Send an action command to the navigation node."""
        msg = AICommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = action_type
        msg.command_data = json.dumps(action_data) if action_data else "{}"
        
        self.action_command_pub.publish(msg)
        self.get_logger().info(f'Sent action command: {action_type}')
    
    def run_tests(self):
        """Run a series of action tests."""
        # Wait for subscribers to connect
        time.sleep(2.0)
        
        # Ensure system is in active mode
        self.publish_system_mode('active')
        time.sleep(1.0)
        
        # Test basic movement actions
        test_actions = [
            ('move_forward', {'duration': 2.0, 'distance': 30}),
            ('stop', {}),
            ('turn_left', {'duration': 1.0}),
            ('stop', {}),
            ('turn_right', {'duration': 1.0}),
            ('stop', {}),
            ('move_backward', {'duration': 1.0, 'distance': 20}),
            ('stop', {}),
        ]
        
        for action_type, action_data in test_actions:
            self.send_action_command(action_type, action_data)
            time.sleep(3.0)  # Wait between commands
        
        self.get_logger().info('All test actions sent successfully!')

def main():
    rclpy.init()
    
    tester = NavigationActionTester()
    
    try:
        # Run the tests
        tester.run_tests()
        
        # Keep the node alive for a bit
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()