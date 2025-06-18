#!/usr/bin/env python3

"""
Test Hardware Initialization for Nevil-picar v2.0

This script tests that the hardware initialization and motor control pipeline is working correctly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import threading

class HardwareInitializationTest(Node):
    """Test node for hardware initialization and motor control."""
    
    def __init__(self):
        super().__init__('hardware_initialization_test')
        
        # Create publisher for cmd_vel (motor commands)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber for system status
        self.system_status_sub = self.create_subscription(
            String,
            'system_status',
            self.system_status_callback,
            10
        )
        
        # Test state
        self.system_status_received = False
        self.test_results = []
        
        self.get_logger().info('Hardware initialization test node started')
    
    def system_status_callback(self, msg):
        """Callback for system status messages."""
        self.get_logger().info(f'System status: {msg.data}')
        self.system_status_received = True
    
    def test_motor_commands(self):
        """Test sending motor commands."""
        self.get_logger().info('Testing motor commands...')
        
        # Test 1: Forward movement
        self.get_logger().info('Test 1: Forward movement')
        cmd = Twist()
        cmd.linear.x = 0.3  # 30cm/s forward
        cmd.angular.z = 0.0
        
        for i in range(5):
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'Published forward command {i+1}/5: linear.x={cmd.linear.x}')
            time.sleep(0.5)
        
        # Test 2: Stop
        self.get_logger().info('Test 2: Stop')
        cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Published stop command')
        time.sleep(1.0)
        
        # Test 3: Turn left
        self.get_logger().info('Test 3: Turn left')
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5  # Turn left
        
        for i in range(3):
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'Published left turn command {i+1}/3: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
            time.sleep(0.5)
        
        # Test 4: Turn right
        self.get_logger().info('Test 4: Turn right')
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = -0.5  # Turn right
        
        for i in range(3):
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'Published right turn command {i+1}/3: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
            time.sleep(0.5)
        
        # Test 5: Final stop
        self.get_logger().info('Test 5: Final stop')
        cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Published final stop command')
        
        self.get_logger().info('Motor command tests completed')
    
    def run_tests(self):
        """Run all hardware tests."""
        self.get_logger().info('Starting hardware initialization tests...')
        
        # Wait a moment for system to initialize
        time.sleep(2.0)
        
        # Test motor commands
        self.test_motor_commands()
        
        # Summary
        self.get_logger().info('Hardware initialization test completed')
        self.get_logger().info('Check the real-time motor control node logs to verify hardware commands are being processed')
        
        if self.system_status_received:
            self.get_logger().info('✓ System status messages are being received')
        else:
            self.get_logger().warn('✗ No system status messages received')

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # Create test node
    test_node = HardwareInitializationTest()
    
    # Run tests in a separate thread
    test_thread = threading.Thread(target=test_node.run_tests)
    test_thread.daemon = True
    test_thread.start()
    
    try:
        # Spin the node for 15 seconds
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 15.0:
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.get_logger().info('Test completed - shutting down')
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()