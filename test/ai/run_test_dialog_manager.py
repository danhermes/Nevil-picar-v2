#!/usr/bin/env python3

"""
Direct script to run the test_dialog_manager_node without relying on ROS2 message types.
This is a temporary solution until the package can be properly built and installed.
"""

import os
import sys
import unittest
import time
import threading

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

# Add the specific directories to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src/nevil_interfaces_ai'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src/nevil_testing/nevil_testing'))

# Create mock classes for ROS2 components
class Node:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscriptions = {}
        self.clients = {}
        self.action_clients = {}
        
    def create_publisher(self, msg_type, topic, qos_profile):
        self.publishers[topic] = (msg_type, qos_profile)
        return Publisher(topic, msg_type)
    
    def create_subscription(self, msg_type, topic, callback, qos_profile):
        self.subscriptions[topic] = (msg_type, callback, qos_profile)
        return Subscription(topic, msg_type, callback)
    
    def create_client(self, srv_type, srv_name):
        self.clients[srv_name] = srv_type
        return Client(srv_name, srv_type)
    
    def get_clock(self):
        return Clock()
    
    def destroy_node(self):
        pass
    
    def destroy_subscription(self, subscription):
        pass
    
    def destroy_client(self, client):
        pass

class Publisher:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
    
    def publish(self, msg):
        print(f"Publishing to {self.topic}: {msg}")

class Subscription:
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

class Client:
    def __init__(self, srv_name, srv_type):
        self.srv_name = srv_name
        self.srv_type = srv_type
    
    def wait_for_service(self, timeout_sec=1.0):
        return True
    
    def call_async(self, request):
        future = Future()
        response = self.srv_type.Response()
        future.set_result(response)
        return future

class ActionClient:
    def __init__(self, node, action_type, action_name):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
    
    def wait_for_server(self, timeout_sec=1.0):
        return True
    
    def send_goal_async(self, goal_msg):
        future = Future()
        goal_handle = GoalHandle(True)
        future.set_result(goal_handle)
        return future
    
    def destroy(self):
        pass

class GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
    
    def get_result_async(self):
        future = Future()
        result = ActionResult()
        future.set_result(result)
        return future

class ActionResult:
    def __init__(self):
        self.result = type('Result', (), {'success': True, 'dialog_summary': 'Mock dialog summary', 'final_state': 'completed'})

class Future:
    def __init__(self):
        self._result = None
        self._done = False
    
    def set_result(self, result):
        self._result = result
        self._done = True
    
    def result(self):
        return self._result
    
    def done(self):
        return self._done

class Clock:
    def now(self):
        return self
    
    def to_msg(self):
        return None

class MultiThreadedExecutor:
    def __init__(self):
        self.nodes = []
    
    def add_node(self, node):
        self.nodes.append(node)
    
    def spin(self):
        while True:
            time.sleep(0.1)
    
    def shutdown(self):
        pass

class Parameter:
    def __init__(self, name, value=None):
        self.name = name
        self.value = value

class QoSProfile:
    def __init__(self, reliability=None, history=None, depth=10):
        self.reliability = reliability
        self.history = history
        self.depth = depth

class QoSReliabilityPolicy:
    RELIABLE = 1

class QoSHistoryPolicy:
    KEEP_LAST = 1

# Mock ROS2 initialization functions
def init(args=None):
    pass

def shutdown():
    pass

# Create mock classes for the missing message types
class MockHeader:
    def __init__(self):
        self.stamp = None

class TextCommand:
    def __init__(self):
        self.header = MockHeader()
        self.command_text = ""
        self.source = ""
        self.command_type = ""
        self.priority = 0
        self.command_id = ""
        self.context_id = ""

class TextResponse:
    def __init__(self):
        self.header = MockHeader()
        self.response_text = ""
        self.source = ""
        self.response_id = ""
        self.command_id = ""
        self.context_id = ""

class VoiceCommand:
    def __init__(self):
        self.header = MockHeader()
        self.recognized_text = ""
        self.confidence = 0.0
        self.command_id = ""
        self.context_id = ""
        self.speaker_id = ""

class VoiceResponse:
    def __init__(self):
        self.header = MockHeader()
        self.response_text = ""
        self.source = ""
        self.response_id = ""
        self.command_id = ""
        self.context_id = ""

class DialogState:
    def __init__(self):
        self.header = MockHeader()
        self.state = "idle"
        self.context_id = ""
        self.last_command_id = ""
        self.last_response_id = ""

class QueryCapabilities:
    class Request:
        def __init__(self):
            self.query_type = ""
    
    class Response:
        def __init__(self):
            self.success = True
            self.capabilities = ""

class TranslateCommand:
    class Request:
        def __init__(self):
            self.natural_language_command = ""
            self.context_id = ""
    
    class Response:
        def __init__(self):
            self.success = True
            self.command_type = "navigation"
            self.parameters = ""
            self.confidence = 1.0

class ProcessDialog:
    class Goal:
        def __init__(self):
            self.initial_utterance = ""
            self.context_id = ""
            self.dialog_mode = ""
            self.timeout = 0.0

class String:
    def __init__(self):
        self.data = ""

# Create a mock NevilTestBase class
class NevilTestBase(unittest.TestCase):
    """
    Mock base class for Nevil-picar v2.0 tests.
    """
    
    def setUp(self):
        """Set up the test."""
        # Initialize ROS2
        init()
        
        # Create a test node
        self.node = Node('test_node')
        
        # Create an executor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        
        # Wait for node to be fully initialized
        time.sleep(0.1)
    
    def tearDown(self):
        """Tear down the test."""
        # Stop the executor
        self.executor.shutdown()
        
        # Wait for the spin thread to finish
        self.spin_thread.join(timeout=0.1)
        
        # Destroy the node
        self.node.destroy_node()
        
        # Shutdown ROS2
        shutdown()
    
    def create_publisher(self, msg_type, topic, qos_profile=10):
        """Create a publisher."""
        return self.node.create_publisher(msg_type, topic, qos_profile)
    
    def create_subscription(self, msg_type, topic, callback, qos_profile=10):
        """Create a subscription."""
        return self.node.create_subscription(msg_type, topic, callback, qos_profile)
    
    def create_client(self, srv_type, srv_name, timeout_sec=1.0):
        """Create a client and wait for the service to be available."""
        client = self.node.create_client(srv_type, srv_name)
        return client
    
    def create_action_client(self, action_type, action_name, timeout_sec=1.0):
        """Create an action client and wait for the action server to be available."""
        client = ActionClient(self.node, action_type, action_name)
        return client
    
    def wait_for_service_response(self, client, request, timeout_sec=1.0):
        """Send a request and wait for the response."""
        future = client.call_async(request)
        return future.result()
    
    def wait_for_action_result(self, client, goal_msg, timeout_sec=5.0):
        """Send a goal and wait for the result."""
        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        return result_future.result()
    
    def spin_until(self, predicate, timeout_sec=1.0, period_sec=0.01):
        """Spin until a predicate is true or a timeout occurs."""
        start_time = time.time()
        while not predicate() and time.time() - start_time < timeout_sec:
            time.sleep(period_sec)
        
        return predicate()

# Add the mock classes to the modules
import sys
import types

# Create mock modules
rclpy_module = types.ModuleType('rclpy')
rclpy_module.init = init
rclpy_module.shutdown = shutdown
rclpy_module.node = types.ModuleType('rclpy.node')
rclpy_module.node.Node = Node
rclpy_module.executors = types.ModuleType('rclpy.executors')
rclpy_module.executors.MultiThreadedExecutor = MultiThreadedExecutor
rclpy_module.parameter = types.ModuleType('rclpy.parameter')
rclpy_module.parameter.Parameter = Parameter
rclpy_module.qos = types.ModuleType('rclpy.qos')
rclpy_module.qos.QoSProfile = QoSProfile
rclpy_module.qos.QoSReliabilityPolicy = QoSReliabilityPolicy
rclpy_module.qos.QoSHistoryPolicy = QoSHistoryPolicy
rclpy_module.action = types.ModuleType('rclpy.action')
rclpy_module.action.ActionClient = ActionClient

std_msgs_module = types.ModuleType('std_msgs.msg')
std_msgs_module.String = String

msg_module = types.ModuleType('nevil_interfaces_ai.msg')
msg_module.TextCommand = TextCommand
msg_module.TextResponse = TextResponse
msg_module.VoiceCommand = VoiceCommand
msg_module.VoiceResponse = VoiceResponse
msg_module.DialogState = DialogState

srv_module = types.ModuleType('nevil_interfaces_ai.srv')
srv_module.QueryCapabilities = QueryCapabilities
srv_module.TranslateCommand = TranslateCommand

action_module = types.ModuleType('nevil_interfaces_ai.action')
action_module.ProcessDialog = ProcessDialog

test_base_module = types.ModuleType('nevil_testing.test_base')
test_base_module.NevilTestBase = NevilTestBase

# Add the mock modules to sys.modules
sys.modules['rclpy'] = rclpy_module
sys.modules['rclpy.node'] = rclpy_module.node
sys.modules['rclpy.executors'] = rclpy_module.executors
sys.modules['rclpy.parameter'] = rclpy_module.parameter
sys.modules['rclpy.qos'] = rclpy_module.qos
sys.modules['rclpy.action'] = rclpy_module.action
sys.modules['std_msgs.msg'] = std_msgs_module
sys.modules['nevil_interfaces_ai.msg'] = msg_module
sys.modules['nevil_interfaces_ai.srv'] = srv_module
sys.modules['nevil_interfaces_ai.action'] = action_module
sys.modules['nevil_testing.test_base'] = test_base_module

# Now try to import the test module
try:
    from test_dialog_manager_node import TestDialogManagerNode
    print("Successfully imported TestDialogManagerNode")
except ImportError as e:
    print(f"Error importing TestDialogManagerNode: {e}")
    print("Python path:", sys.path)
    sys.exit(1)

if __name__ == '__main__':
    print("Running test_dialog_manager_node tests...")
    unittest.main(module='test_dialog_manager_node')