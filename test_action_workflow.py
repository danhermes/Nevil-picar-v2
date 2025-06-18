#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nevil_interfaces_ai_msgs.msg import AICommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class ActionTester(Node):
    def __init__(self):
        super().__init__('action_tester')
        
        # Use same QoS as ai_interface_node
        ai_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher to send action commands
        self.action_pub = self.create_publisher(
            AICommand, 
            '/nevil/action_command', 
            qos_profile=ai_qos
        )
        
        self.get_logger().info('Action Tester initialized')
        
    def send_test_action(self, action_name):
        """Send a test action command"""
        msg = AICommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = action_name
        msg.command_data = "{}"
        
        self.action_pub.publish(msg)
        self.get_logger().info(f'Sent action command: {action_name}')

def main():
    rclpy.init()
    
    tester = ActionTester()
    
    # Wait a moment for connections
    time.sleep(2)
    
    # Test actions
    test_actions = ["wave hands", "nod", "think", "honk"]
    
    for action in test_actions:
        tester.send_test_action(action)
        time.sleep(3)  # Wait 3 seconds between actions
    
    tester.get_logger().info('Test completed')
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()