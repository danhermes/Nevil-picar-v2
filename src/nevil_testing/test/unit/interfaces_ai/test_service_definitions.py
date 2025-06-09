#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_service

class TestServiceDefinitions(unittest.TestCase):
    """Test the service definitions in the nevil_interfaces_ai package."""

    def setUp(self):
        """Set up the test fixture."""
        # Get the service classes
        self.AIQuery = get_service('nevil_interfaces_ai/AIQuery')
        self.QueryCapabilities = get_service('nevil_interfaces_ai/QueryCapabilities')
        self.TranslateCommand = get_service('nevil_interfaces_ai/TranslateCommand')

    def test_ai_query_request_creation(self):
        """Test creating an AIQuery request."""
        # Create an AIQuery request
        request = self.AIQuery.Request()
        request.query = "What is the weather today?"

        # Verify the request fields
        self.assertEqual(request.query, "What is the weather today?")

    def test_ai_query_response_creation(self):
        """Test creating an AIQuery response."""
        # Create an AIQuery response
        response = self.AIQuery.Response()
        response.response = "The weather in New York is sunny with a high of 25°C."
        response.confidence = 0.95

        # Verify the response fields
        self.assertEqual(response.response, "The weather in New York is sunny with a high of 25°C.")
        self.assertEqual(response.confidence, 0.95)

    def test_query_capabilities_request_creation(self):
        """Test creating a QueryCapabilities request."""
        # Create a QueryCapabilities request
        request = self.QueryCapabilities.Request()
        request.query_type = "all"

        # Verify the request fields
        self.assertEqual(request.query_type, "all")

    def test_query_capabilities_response_creation(self):
        """Test creating a QueryCapabilities response."""
        # Create a QueryCapabilities response
        response = self.QueryCapabilities.Response()
        response.success = True
        response.message = "Capabilities retrieved successfully"
        response.capabilities = '{"speech_recognition": true, "speech_synthesis": true, "natural_language_processing": true}'

        # Verify the response fields
        self.assertEqual(response.success, True)
        self.assertEqual(response.message, "Capabilities retrieved successfully")
        self.assertEqual(response.capabilities, '{"speech_recognition": true, "speech_synthesis": true, "natural_language_processing": true}')

    def test_translate_command_request_creation(self):
        """Test creating a TranslateCommand request."""
        # Create a TranslateCommand request
        request = self.TranslateCommand.Request()
        request.natural_language_command = "Move forward one meter"
        request.context_id = "ctx_002"

        # Verify the request fields
        self.assertEqual(request.natural_language_command, "Move forward one meter")
        self.assertEqual(request.context_id, "ctx_002")

    def test_translate_command_response_creation(self):
        """Test creating a TranslateCommand response."""
        # Create a TranslateCommand response
        response = self.TranslateCommand.Response()
        response.success = True
        response.message = "Command translated successfully"
        response.command_type = "navigation"
        response.command_target = "motion_controller"
        response.command_action = "move"
        response.parameters = '{"distance": 1.0, "direction": "forward"}'
        response.confidence = 0.9
        response.alternatives = '[]'

        # Verify the response fields
        self.assertEqual(response.success, True)
        self.assertEqual(response.message, "Command translated successfully")
        self.assertEqual(response.command_type, "navigation")
        self.assertEqual(response.command_target, "motion_controller")
        self.assertEqual(response.command_action, "move")
        self.assertEqual(response.parameters, '{"distance": 1.0, "direction": "forward"}')
        self.assertEqual(response.confidence, 0.9)
        self.assertEqual(response.alternatives, '[]')

    def test_service_serialization(self):
        """Test serializing and deserializing service messages."""
        # Test AIQuery request serialization
        ai_query_request = self.AIQuery.Request()
        ai_query_request.query = "What is the weather today?"
        
        serialized_msg = serialize_message(ai_query_request)
        deserialized_msg = deserialize_message(serialized_msg, self.AIQuery.Request)
        
        self.assertEqual(deserialized_msg.query, "What is the weather today?")
        
        # Test AIQuery response serialization
        ai_query_response = self.AIQuery.Response()
        ai_query_response.response = "The weather is sunny."
        ai_query_response.confidence = 0.95
        
        serialized_msg = serialize_message(ai_query_response)
        deserialized_msg = deserialize_message(serialized_msg, self.AIQuery.Response)
        
        self.assertEqual(deserialized_msg.response, "The weather is sunny.")
        self.assertAlmostEqual(deserialized_msg.confidence, 0.95, places=5)
        
        # Test TranslateCommand request serialization
        translate_request = self.TranslateCommand.Request()
        translate_request.natural_language_command = "Move forward one meter"
        
        serialized_msg = serialize_message(translate_request)
        deserialized_msg = deserialize_message(serialized_msg, self.TranslateCommand.Request)
        
        self.assertEqual(deserialized_msg.natural_language_command, "Move forward one meter")
        
        # Test TranslateCommand response serialization
        translate_response = self.TranslateCommand.Response()
        translate_response.success = True
        translate_response.command_type = "navigation"
        translate_response.parameters = '{"distance": 1.0, "direction": "forward"}'
        
        serialized_msg = serialize_message(translate_response)
        deserialized_msg = deserialize_message(serialized_msg, self.TranslateCommand.Response)
        
        self.assertEqual(deserialized_msg.success, True)
        self.assertEqual(deserialized_msg.command_type, "navigation")
        self.assertEqual(deserialized_msg.parameters, '{"distance": 1.0, "direction": "forward"}')

    def test_error_handling(self):
        """Test error handling in service responses."""
        # Test QueryCapabilities error response
        query_capabilities_response = self.QueryCapabilities.Response()
        query_capabilities_response.success = False
        query_capabilities_response.message = "Internal server error"
        query_capabilities_response.capabilities = ""
        
        self.assertEqual(query_capabilities_response.success, False)
        self.assertEqual(query_capabilities_response.message, "Internal server error")
        self.assertEqual(query_capabilities_response.capabilities, "")
        
        # Test TranslateCommand error response
        translate_response = self.TranslateCommand.Response()
        translate_response.success = False
        translate_response.message = "Invalid command format"
        translate_response.command_type = ""
        translate_response.command_target = ""
        translate_response.command_action = ""
        translate_response.parameters = ""
        translate_response.confidence = 0.0
        
        self.assertEqual(translate_response.success, False)
        self.assertEqual(translate_response.message, "Invalid command format")
        self.assertEqual(translate_response.command_type, "")
        self.assertEqual(translate_response.parameters, "")
        self.assertEqual(translate_response.confidence, 0.0)

if __name__ == '__main__':
    unittest.main()