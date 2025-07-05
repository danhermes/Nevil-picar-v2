#!/usr/bin/env python3

"""
Test script for action execution system.

This script tests the complete action execution flow:
1. AI interface publishes action commands
2. Navigation node receives and executes actions
3. Motion commands are sent to cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nevil_interfaces_ai_msgs.msg import AICommand
from geometry_msgs.msg import Twist
import json
import time

class ActionExecutionTester(Node):
    """Test node for action execution system"""
    
    def __init__(self):
        super().__init__('action_execution_tester')
        
        # QoS profile
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Publisher for action commands
        self.action_pub = self.create_publisher(
            AICommand,
            '/nevil/action_command',
            qos_profile=self.qos
        )
        
        # Subscriber for cmd_vel to verify actions are executed
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Test state
        self.received_commands = []
        self.test_results = {}
        
        self.get_logger().info('Action execution tester initialized')
    
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel messages to verify action execution"""
        self.received_commands.append({
            'timestamp': time.time(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        })
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    def publish_action(self, action_type, action_data=None):
        """Publish an action command"""
        if action_data is None:
            action_data = {}
        
        msg = AICommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = action_type
        msg.command_data = json.dumps(action_data)
        
        self.action_pub.publish(msg)
        self.get_logger().info(f'Published action: {action_type} with data: {action_data}')
    
    def run_tests(self):
        """Run a series of action execution tests including v1.0 actions"""
        self.get_logger().info('Starting comprehensive v1.0-style action execution tests...')
        
        # Basic Movement Tests (v1.0 compatible)
        self.get_logger().info('=== BASIC MOVEMENT TESTS ===')
        
        # Test 1: Move forward (v1.0 style)
        self.get_logger().info('Test 1: Move forward (v1.0 style)')
        self.publish_action('forward', {'distance': 20, 'speed': 0.3, 'duration': 2.0})
        time.sleep(3.0)
        
        # Test 2: Turn left (v1.0 style)
        self.get_logger().info('Test 2: Turn left (v1.0 style)')
        self.publish_action('left', {'duration': 1.0})
        time.sleep(2.0)
        
        # Test 3: Turn right (v1.0 style)
        self.get_logger().info('Test 3: Turn right (v1.0 style)')
        self.publish_action('right', {'duration': 1.0})
        time.sleep(2.0)
        
        # Test 4: Move backward (v1.0 style)
        self.get_logger().info('Test 4: Move backward (v1.0 style)')
        self.publish_action('backward', {'distance': 20, 'speed': 0.2, 'duration': 1.5})
        time.sleep(2.5)
        
        # Test 5: Twist movements (v1.0 style)
        self.get_logger().info('Test 5: Twist left (v1.0 style)')
        self.publish_action('twist left', {'duration': 1.0})
        time.sleep(2.0)
        
        self.get_logger().info('Test 6: Twist right (v1.0 style)')
        self.publish_action('twist right', {'duration': 1.0})
        time.sleep(2.0)
        
        # Advanced Behavioral Tests (v1.0 expressive actions)
        self.get_logger().info('=== BEHAVIORAL TESTS ===')
        
        # Test 7: Wave hands
        self.get_logger().info('Test 7: Wave hands (v1.0 behavior)')
        self.publish_action('wave hands')
        time.sleep(2.0)
        
        # Test 8: Shake head
        self.get_logger().info('Test 8: Shake head (v1.0 behavior)')
        self.publish_action('shake head')
        time.sleep(2.0)
        
        # Test 9: Nod
        self.get_logger().info('Test 9: Nod (v1.0 behavior)')
        self.publish_action('nod')
        time.sleep(1.5)
        
        # Test 10: Act cute
        self.get_logger().info('Test 10: Act cute (v1.0 behavior)')
        self.publish_action('act cute')
        time.sleep(3.0)
        
        # Test 11: Think
        self.get_logger().info('Test 11: Think (v1.0 behavior)')
        self.publish_action('think')
        time.sleep(2.0)
        
        # Test 12: Celebrate
        self.get_logger().info('Test 12: Celebrate (v1.0 behavior)')
        self.publish_action('celebrate')
        time.sleep(3.0)
        
        # Sound Tests (v1.0 audio actions)
        self.get_logger().info('=== SOUND TESTS ===')
        
        # Test 13: Honk
        self.get_logger().info('Test 13: Honk (v1.0 sound)')
        self.publish_action('honk')
        time.sleep(1.0)
        
        # Test 14: Start engine
        self.get_logger().info('Test 14: Start engine (v1.0 sound)')
        self.publish_action('start engine')
        time.sleep(2.0)
        
        # Test 15: Stop
        self.get_logger().info('Test 15: Stop (v1.0 style)')
        self.publish_action('stop')
        time.sleep(1.0)
        
        # Analyze results
        self.analyze_results()
    
    def analyze_results(self):
        """Analyze test results"""
        self.get_logger().info(f'Test completed. Received {len(self.received_commands)} cmd_vel messages')
        
        if len(self.received_commands) >= 5:
            self.get_logger().info('✓ Action execution system working correctly')
            self.get_logger().info('✓ AI interface successfully publishes action commands')
            self.get_logger().info('✓ Navigation node successfully receives and executes actions')
            self.get_logger().info('✓ Motion commands are properly sent to cmd_vel')
        else:
            self.get_logger().warning('⚠ Some actions may not have been executed properly')
        
        # Print command summary
        for i, cmd in enumerate(self.received_commands):
            self.get_logger().info(f'Command {i+1}: linear.x={cmd["linear_x"]}, angular.z={cmd["angular_z"]}')

def main(args=None):
    rclpy.init(args=args)
    
    tester = ActionExecutionTester()
    
    try:
        # Wait a moment for other nodes to start
        time.sleep(2.0)
        
        # Run tests
        tester.run_tests()
        
        # Keep spinning for a bit to receive any remaining messages
        rclpy.spin_once(tester, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()