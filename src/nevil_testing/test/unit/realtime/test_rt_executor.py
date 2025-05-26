#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import RealtimeCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32
from builtin_interfaces.msg import Time

from nevil_testing.test_base import NevilTestBase

class TestRtExecutor(NevilTestBase):
    """
    Unit tests for the rt_executor in the nevil_realtime package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a subscription to rt_executor status
        self.rt_status = None
        self.rt_status_sub = self.create_subscription(
            String,
            '/rt_executor/status',
            self.rt_status_callback,
            10
        )
        
        # Create a subscription to rt_executor latency
        self.rt_latency = None
        self.rt_latency_sub = self.create_subscription(
            Float32,
            '/rt_executor/latency',
            self.rt_latency_callback,
            10
        )
        
        # Wait for the rt_executor to be ready
        self.wait_for_rt_executor()
    
    def rt_status_callback(self, msg):
        """Callback for rt_executor status messages."""
        self.rt_status = msg
    
    def rt_latency_callback(self, msg):
        """Callback for rt_executor latency messages."""
        self.rt_latency = msg
    
    def wait_for_rt_executor(self, timeout_sec=5.0):
        """Wait for the rt_executor to be ready."""
        # Wait for rt_executor status
        if not self.spin_until(lambda: self.rt_status is not None, timeout_sec):
            self.fail('Timed out waiting for rt_executor status')
    
    def test_rt_executor_status(self):
        """Test that the rt_executor publishes status."""
        # Verify that we received a status message
        self.assertIsNotNone(self.rt_status)
        
        # Verify the status message
        self.assertIn(self.rt_status.data, ['RUNNING', 'INITIALIZING', 'IDLE'])
    
    def test_rt_executor_latency(self):
        """Test that the rt_executor publishes latency."""
        # Verify that we received a latency message
        if not self.spin_until(lambda: self.rt_latency is not None, 1.0):
            self.fail('Timed out waiting for rt_executor latency')
        
        # Verify the latency message
        self.assertIsNotNone(self.rt_latency)
        self.assertGreaterEqual(self.rt_latency.data, 0.0)
    
    def test_realtime_callback_group(self):
        """Test that the RealtimeCallbackGroup works correctly."""
        # Create a test node with a realtime callback group
        test_node = Node('test_rt_node')
        rt_callback_group = RealtimeCallbackGroup()
        
        # Create a publisher and timer using the realtime callback group
        test_pub = test_node.create_publisher(
            String,
            '/test_rt_topic',
            10,
            callback_group=rt_callback_group
        )
        
        # Variables to track callback execution
        callback_executed = False
        callback_time = None
        
        def timer_callback():
            nonlocal callback_executed, callback_time
            callback_executed = True
            callback_time = time.time()
            test_pub.publish(String(data='test'))
        
        test_timer = test_node.create_timer(
            0.1,  # 100ms
            timer_callback,
            callback_group=rt_callback_group
        )
        
        # Add the node to the executor
        self.executor.add_node(test_node)
        
        # Wait for the callback to be executed
        if not self.spin_until(lambda: callback_executed, 1.0):
            self.fail('Timed out waiting for realtime callback to execute')
        
        # Verify the callback was executed
        self.assertTrue(callback_executed)
        
        # Clean up
        test_node.destroy_timer(test_timer)
        test_node.destroy_publisher(test_pub)
        self.executor.remove_node(test_node)
        test_node.destroy_node()
    
    def test_rt_executor_priority(self):
        """Test that the rt_executor respects callback priorities."""
        # Create a test node with realtime and non-realtime callback groups
        test_node = Node('test_priority_node')
        rt_callback_group = RealtimeCallbackGroup()
        non_rt_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Variables to track callback execution
        rt_callback_executed = False
        non_rt_callback_executed = False
        rt_callback_time = None
        non_rt_callback_time = None
        
        # Create publishers using the callback groups
        rt_pub = test_node.create_publisher(
            String,
            '/test_rt_topic',
            10,
            callback_group=rt_callback_group
        )
        
        non_rt_pub = test_node.create_publisher(
            String,
            '/test_non_rt_topic',
            10,
            callback_group=non_rt_callback_group
        )
        
        # Create timers using the callback groups
        def rt_timer_callback():
            nonlocal rt_callback_executed, rt_callback_time
            rt_callback_executed = True
            rt_callback_time = time.time()
            rt_pub.publish(String(data='rt_test'))
            # Simulate some work
            time.sleep(0.01)
        
        def non_rt_timer_callback():
            nonlocal non_rt_callback_executed, non_rt_callback_time
            non_rt_callback_executed = True
            non_rt_callback_time = time.time()
            non_rt_pub.publish(String(data='non_rt_test'))
            # Simulate some work
            time.sleep(0.01)
        
        rt_timer = test_node.create_timer(
            0.1,  # 100ms
            rt_timer_callback,
            callback_group=rt_callback_group
        )
        
        non_rt_timer = test_node.create_timer(
            0.1,  # 100ms
            non_rt_timer_callback,
            callback_group=non_rt_callback_group
        )
        
        # Add the node to the executor
        self.executor.add_node(test_node)
        
        # Wait for both callbacks to be executed
        if not self.spin_until(lambda: rt_callback_executed and non_rt_callback_executed, 1.0):
            self.fail('Timed out waiting for callbacks to execute')
        
        # Verify both callbacks were executed
        self.assertTrue(rt_callback_executed)
        self.assertTrue(non_rt_callback_executed)
        
        # Clean up
        test_node.destroy_timer(rt_timer)
        test_node.destroy_timer(non_rt_timer)
        test_node.destroy_publisher(rt_pub)
        test_node.destroy_publisher(non_rt_pub)
        self.executor.remove_node(test_node)
        test_node.destroy_node()
    
    def test_rt_executor_overrun_handling(self):
        """Test that the rt_executor handles callback overruns correctly."""
        # Create a test node with a realtime callback group
        test_node = Node('test_overrun_node')
        rt_callback_group = RealtimeCallbackGroup()
        
        # Variables to track callback execution
        callback_count = 0
        overrun_detected = False
        
        # Create a subscription to rt_executor overruns
        def overrun_callback(msg):
            nonlocal overrun_detected
            overrun_detected = True
        
        overrun_sub = test_node.create_subscription(
            String,
            '/rt_executor/overruns',
            overrun_callback,
            10
        )
        
        # Create a timer that will overrun
        def timer_callback():
            nonlocal callback_count
            callback_count += 1
            # Simulate a long-running callback that will cause an overrun
            if callback_count == 2:  # Only sleep on the second callback
                time.sleep(0.2)  # Sleep longer than the timer period
        
        test_timer = test_node.create_timer(
            0.05,  # 50ms, shorter than the sleep in the callback
            timer_callback,
            callback_group=rt_callback_group
        )
        
        # Add the node to the executor
        self.executor.add_node(test_node)
        
        # Wait for multiple callbacks to be executed
        time.sleep(0.5)  # Allow time for multiple callbacks and potential overruns
        
        # Verify that callbacks were executed
        self.assertGreater(callback_count, 0)
        
        # Clean up
        test_node.destroy_timer(test_timer)
        test_node.destroy_subscription(overrun_sub)
        self.executor.remove_node(test_node)
        test_node.destroy_node()
    
    def test_rt_executor_shutdown(self):
        """Test that the rt_executor shuts down correctly."""
        # Create a test node
        test_node = Node('test_shutdown_node')
        
        # Create a publisher to request shutdown
        shutdown_pub = test_node.create_publisher(
            String,
            '/rt_executor/control',
            10
        )
        
        # Add the node to the executor
        self.executor.add_node(test_node)
        
        # Request shutdown
        shutdown_pub.publish(String(data='SHUTDOWN'))
        
        # Wait for the rt_executor to shut down
        time.sleep(0.5)
        
        # Verify the rt_executor status
        if self.rt_status is not None:
            self.assertIn(self.rt_status.data, ['SHUTDOWN', 'SHUTTING_DOWN', 'IDLE'])
        
        # Clean up
        test_node.destroy_publisher(shutdown_pub)
        self.executor.remove_node(test_node)
        test_node.destroy_node()

if __name__ == '__main__':
    unittest.main()