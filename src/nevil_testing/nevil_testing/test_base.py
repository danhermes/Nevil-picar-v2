#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class NevilTestBase(unittest.TestCase):
    """
    Base class for Nevil-picar v2.0 tests.
    Provides common functionality for testing ROS2 nodes.
    """
    
    def setUp(self):
        """Set up the test."""
        # Initialize ROS2
        rclpy.init()
        
        # Create a test node
        self.node = Node('test_node')
        
        # Create an executor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        
        # Wait for node to be fully initialized
        time.sleep(0.5)
    
    def tearDown(self):
        """Tear down the test."""
        # Stop the executor
        self.executor.shutdown()
        
        # Wait for the spin thread to finish
        self.spin_thread.join(timeout=1.0)
        
        # Destroy the node
        self.node.destroy_node()
        
        # Shutdown ROS2
        rclpy.shutdown()
    
    def create_publisher(self, msg_type, topic, qos_profile=None):
        """Create a publisher."""
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        
        return self.node.create_publisher(msg_type, topic, qos_profile)
    
    def create_subscription(self, msg_type, topic, callback, qos_profile=None):
        """Create a subscription."""
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        
        return self.node.create_subscription(msg_type, topic, callback, qos_profile)
    
    def create_client(self, srv_type, srv_name, timeout_sec=1.0):
        """Create a client and wait for the service to be available."""
        client = self.node.create_client(srv_type, srv_name)
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.fail(f'Service {srv_name} not available within timeout')
        
        return client
    
    def create_action_client(self, action_type, action_name, timeout_sec=1.0):
        """Create an action client and wait for the action server to be available."""
        from rclpy.action import ActionClient
        
        client = ActionClient(self.node, action_type, action_name)
        
        # Wait for action server to be available
        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.fail(f'Action server {action_name} not available within timeout')
        
        return client
    
    def wait_for_message(self, subscription, timeout_sec=1.0):
        """Wait for a message on a subscription."""
        message = None
        
        def callback(msg):
            nonlocal message
            message = msg
        
        # Store the original callback
        original_callback = subscription.callback
        
        # Set our callback
        subscription.callback = callback
        
        # Wait for the message
        start_time = time.time()
        while message is None and time.time() - start_time < timeout_sec:
            time.sleep(0.01)
        
        # Restore the original callback
        subscription.callback = original_callback
        
        return message
    
    def wait_for_service_response(self, client, request, timeout_sec=1.0):
        """Send a request and wait for the response."""
        future = client.call_async(request)
        
        # Wait for the response
        start_time = time.time()
        while not future.done() and time.time() - start_time < timeout_sec:
            time.sleep(0.01)
        
        if not future.done():
            self.fail(f'Service response not received within timeout')
        
        return future.result()
    
    def wait_for_action_result(self, client, goal_msg, timeout_sec=5.0):
        """Send a goal and wait for the result."""
        send_goal_future = client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not send_goal_future.done() and time.time() - start_time < timeout_sec:
            time.sleep(0.01)
        
        if not send_goal_future.done():
            self.fail(f'Goal acceptance not received within timeout')
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.fail(f'Goal was rejected')
        
        # Wait for the result
        result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        while not result_future.done() and time.time() - start_time < timeout_sec:
            time.sleep(0.01)
        
        if not result_future.done():
            self.fail(f'Action result not received within timeout')
        
        return result_future.result()
    
    def assert_parameters_equal(self, node, parameter_name, expected_value):
        """Assert that a parameter has the expected value."""
        param = node.get_parameter(parameter_name)
        self.assertEqual(param.value, expected_value)
    
    def set_parameters(self, node, parameters):
        """Set parameters on a node."""
        param_list = []
        for name, value in parameters.items():
            param_list.append(Parameter(name, value=value))
        
        return node.set_parameters(param_list)
    
    def create_test_node(self, node_name, parameters=None):
        """Create a test node with the given parameters."""
        node = Node(node_name)
        
        if parameters:
            self.set_parameters(node, parameters)
        
        return node
    
    def spin_until(self, predicate, timeout_sec=1.0, period_sec=0.01):
        """Spin until a predicate is true or a timeout occurs."""
        start_time = time.time()
        while not predicate() and time.time() - start_time < timeout_sec:
            time.sleep(period_sec)
        
        return predicate()
    
    def assert_topic_published(self, topic, msg_type, timeout_sec=1.0):
        """Assert that a message is published on a topic."""
        message_received = False
        
        def callback(msg):
            nonlocal message_received
            message_received = True
        
        subscription = self.create_subscription(msg_type, topic, callback)
        
        # Wait for the message
        result = self.spin_until(lambda: message_received, timeout_sec)
        
        # Clean up
        self.node.destroy_subscription(subscription)
        
        self.assertTrue(result, f'No message received on topic {topic} within timeout')
    
    def assert_service_available(self, service_name, srv_type, timeout_sec=1.0):
        """Assert that a service is available."""
        client = self.node.create_client(srv_type, service_name)
        
        # Wait for service to be available
        result = client.wait_for_service(timeout_sec=timeout_sec)
        
        # Clean up
        self.node.destroy_client(client)
        
        self.assertTrue(result, f'Service {service_name} not available within timeout')
    
    def assert_action_available(self, action_name, action_type, timeout_sec=1.0):
        """Assert that an action server is available."""
        from rclpy.action import ActionClient
        
        client = ActionClient(self.node, action_type, action_name)
        
        # Wait for action server to be available
        result = client.wait_for_server(timeout_sec=timeout_sec)
        
        # Clean up
        client.destroy()
        
        self.assertTrue(result, f'Action server {action_name} not available within timeout')


# Fix missing import
import threading