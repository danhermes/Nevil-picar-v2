#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nevil_interfaces_ai.msg import TextCommand, TextResponse, VoiceCommand, VoiceResponse, DialogState
from nevil_interfaces_ai.srv import QueryCapabilities, TranslateCommand
from nevil_interfaces_ai.action import ProcessDialog

from nevil_testing.test_base import NevilTestBase

class TestDialogManagerNode(NevilTestBase):
    """
    Unit tests for the dialog_manager_node in the nevil_interfaces_ai package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create publishers for text and voice commands
        self.text_command_pub = self.create_publisher(TextCommand, '/text_command', 10)
        self.voice_command_pub = self.create_publisher(VoiceCommand, '/voice_command', 10)
        
        # Create subscriptions for text and voice responses
        self.text_response = None
        self.text_response_sub = self.create_subscription(
            TextResponse,
            '/text_response',
            self.text_response_callback,
            10
        )
        
        self.voice_response = None
        self.voice_response_sub = self.create_subscription(
            VoiceResponse,
            '/voice_response',
            self.voice_response_callback,
            10
        )
        
        # Create a subscription for dialog state
        self.dialog_state = None
        self.dialog_state_sub = self.create_subscription(
            DialogState,
            '/dialog_state',
            self.dialog_state_callback,
            10
        )
        
        # Wait for the dialog manager node to be ready
        self.wait_for_dialog_manager()
    
    def text_response_callback(self, msg):
        """Callback for text response messages."""
        self.text_response = msg
    
    def voice_response_callback(self, msg):
        """Callback for voice response messages."""
        self.voice_response = msg
    
    def dialog_state_callback(self, msg):
        """Callback for dialog state messages."""
        self.dialog_state = msg
    
    def wait_for_dialog_manager(self, timeout_sec=5.0):
        """Wait for the dialog manager node to be ready."""
        # Send a test text command
        self.send_text_command("test")
        
        # Wait for dialog state
        if not self.spin_until(lambda: self.dialog_state is not None, timeout_sec):
            self.fail('Timed out waiting for dialog state')
    
    def send_text_command(self, text, command_type='query', priority=0, context_id=''):
        """Send a text command."""
        msg = TextCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.command_text = text
        msg.source = 'user'
        msg.command_type = command_type
        msg.priority = priority
        msg.command_id = f'test_{time.time()}'
        msg.context_id = context_id
        
        self.text_command_pub.publish(msg)
    
    def send_voice_command(self, text, confidence=0.9, context_id=''):
        """Send a voice command."""
        msg = VoiceCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.recognized_text = text
        msg.confidence = confidence
        msg.command_id = f'test_{time.time()}'
        msg.context_id = context_id
        msg.speaker_id = 'test_speaker'
        
        self.voice_command_pub.publish(msg)
    
    def test_text_command_response(self):
        """Test that the dialog manager responds to text commands."""
        # Reset response
        self.text_response = None
        
        # Send a text command
        self.send_text_command("What time is it?")
        
        # Wait for a response
        if not self.spin_until(lambda: self.text_response is not None, 1.0):
            self.fail('Timed out waiting for text response')
        
        # Verify the response
        self.assertIsNotNone(self.text_response)
        self.assertIsNotNone(self.text_response.response_text)
        self.assertEqual(self.text_response.source, 'system')
    
    def test_voice_command_response(self):
        """Test that the dialog manager responds to voice commands."""
        # Reset response
        self.voice_response = None
        
        # Send a voice command
        self.send_voice_command("What is your name?")
        
        # Wait for a response
        if not self.spin_until(lambda: self.voice_response is not None, 1.0):
            self.fail('Timed out waiting for voice response')
        
        # Verify the response
        self.assertIsNotNone(self.voice_response)
        self.assertIsNotNone(self.voice_response.response_text)
        self.assertEqual(self.voice_response.source, 'system')
    
    def test_dialog_state_updates(self):
        """Test that the dialog state is updated correctly."""
        # Reset dialog state
        initial_state = self.dialog_state
        
        # Send a text command
        self.send_text_command("Hello")
        
        # Wait for dialog state to change
        def check_state_changed():
            return (self.dialog_state is not None and 
                    self.dialog_state.state != initial_state.state)
        
        if not self.spin_until(check_state_changed, 1.0):
            self.fail('Timed out waiting for dialog state to change')
        
        # Verify the dialog state changed
        self.assertNotEqual(self.dialog_state.state, initial_state.state)
        
        # Wait for dialog state to return to idle
        def check_state_idle():
            return (self.dialog_state is not None and 
                    self.dialog_state.state == 'idle')
        
        if not self.spin_until(check_state_idle, 3.0):
            self.fail('Timed out waiting for dialog state to return to idle')
        
        # Verify the dialog state returned to idle
        self.assertEqual(self.dialog_state.state, 'idle')
    
    def test_query_capabilities_service(self):
        """Test the query_capabilities service."""
        # Create a client for the query_capabilities service
        client = self.create_client(QueryCapabilities, '/query_capabilities')
        
        # Create a request
        request = QueryCapabilities.Request()
        request.query_type = 'all'
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)
        self.assertIsNotNone(response.capabilities)
        self.assertNotEqual(response.capabilities, '')
    
    def test_translate_command_service(self):
        """Test the translate_command service."""
        # Create a client for the translate_command service
        client = self.create_client(TranslateCommand, '/translate_command')
        
        # Create a request
        request = TranslateCommand.Request()
        request.natural_language_command = "Move forward one meter"
        request.context_id = ''
        
        # Send the request and wait for the response
        response = self.wait_for_service_response(client, request)
        
        # Verify the response
        self.assertTrue(response.success)
        self.assertEqual(response.command_type, 'navigation')
        self.assertIsNotNone(response.parameters)
        self.assertGreater(response.confidence, 0.0)
    
    def test_process_dialog_action(self):
        """Test the process_dialog action."""
        # Create an action client
        client = self.create_action_client(
            ProcessDialog,
            '/process_dialog'
        )
        
        # Create a goal
        goal_msg = ProcessDialog.Goal()
        goal_msg.initial_utterance = "Hello, how are you?"
        goal_msg.context_id = ''
        goal_msg.dialog_mode = 'conversation'
        goal_msg.timeout = 5.0
        
        # Send the goal and wait for the result
        result = self.wait_for_action_result(client, goal_msg)
        
        # Verify the result
        self.assertTrue(result.result.success)
        self.assertIsNotNone(result.result.dialog_summary)
        self.assertEqual(result.result.final_state, 'completed')
    
    def test_context_management(self):
        """Test that the dialog manager maintains context between commands."""
        # Send a text command with a context ID
        context_id = f'test_context_{time.time()}'
        self.send_text_command("My name is John", context_id=context_id)
        
        # Wait for a response
        if not self.spin_until(lambda: self.text_response is not None, 1.0):
            self.fail('Timed out waiting for text response')
        
        # Reset response
        self.text_response = None
        
        # Send a follow-up command with the same context ID
        self.send_text_command("What is my name?", context_id=context_id)
        
        # Wait for a response
        if not self.spin_until(lambda: self.text_response is not None, 1.0):
            self.fail('Timed out waiting for text response')
        
        # Verify the response contains the name
        self.assertIsNotNone(self.text_response)
        self.assertIn("John", self.text_response.response_text)
    
    def test_error_handling(self):
        """Test that the dialog manager handles errors correctly."""
        # Send an invalid text command
        msg = TextCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.command_text = ""  # Empty command
        msg.source = 'user'
        msg.command_type = 'invalid_type'
        msg.priority = 0
        msg.command_id = f'test_{time.time()}'
        msg.context_id = ''
        
        # Reset response
        self.text_response = None
        
        # Publish the invalid command
        self.text_command_pub.publish(msg)
        
        # Wait for a response
        if not self.spin_until(lambda: self.text_response is not None, 1.0):
            self.fail('Timed out waiting for text response')
        
        # Verify the response indicates an error
        self.assertIsNotNone(self.text_response)
        self.assertIn("error", self.text_response.response_text.lower())

if __name__ == '__main__':
    unittest.main()