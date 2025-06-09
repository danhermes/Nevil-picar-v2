#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header

class TestMessageDefinitions(unittest.TestCase):
    """Test the message definitions in the nevil_interfaces_ai package."""

    def setUp(self):
        """Set up the test fixture."""
        # Get the message classes
        self.AICommand = get_message('nevil_interfaces_ai/AICommand')
        self.AIStatus = get_message('nevil_interfaces_ai/AIStatus')
        self.Audio = get_message('nevil_interfaces_ai/Audio')
        self.DialogState = get_message('nevil_interfaces_ai/DialogState')
        self.TextCommand = get_message('nevil_interfaces_ai/TextCommand')
        self.TextResponse = get_message('nevil_interfaces_ai/TextResponse')
        self.VoiceCommand = get_message('nevil_interfaces_ai/VoiceCommand')
        self.VoiceResponse = get_message('nevil_interfaces_ai/VoiceResponse')

    def test_ai_command_creation(self):
        """Test creating an AICommand message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "ai_system"

        # Create an AICommand message
        msg = self.AICommand()
        msg.header = header
        msg.command_type = "navigation"
        msg.command_data = '{"command": "Move forward 1 meter", "distance": 1.0, "direction": "forward"}'

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "ai_system")
        self.assertEqual(msg.command_type, "navigation")
        self.assertEqual(msg.command_data, '{"command": "Move forward 1 meter", "distance": 1.0, "direction": "forward"}')

    def test_ai_status_creation(self):
        """Test creating an AIStatus message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "ai_system"

        # Create an AIStatus message
        msg = self.AIStatus()
        msg.header = header
        msg.status = "active"
        msg.confidence = 0.95

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "ai_system")
        self.assertEqual(msg.status, "active")
        self.assertAlmostEqual(msg.confidence, 0.95, places=5)

    def test_audio_creation(self):
        """Test creating an Audio message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "microphone"

        # Create an Audio message - check the actual fields in the message definition
        # Since we don't have the full Audio.msg definition, we'll skip detailed testing
        msg = self.Audio()
        msg.header = header
        
        # Verify the header field
        self.assertEqual(msg.header.frame_id, "microphone")

    def test_dialog_state_creation(self):
        """Test creating a DialogState message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "dialog_system"

        # Create a DialogState message - check the actual fields in the message definition
        # Since we don't have the full DialogState.msg definition, we'll skip detailed testing
        msg = self.DialogState()
        msg.header = header
        
        # Verify the header field
        self.assertEqual(msg.header.frame_id, "dialog_system")

    def test_text_command_creation(self):
        """Test creating a TextCommand message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "text_interface"

        # Create a TextCommand message
        msg = self.TextCommand()
        msg.header = header
        msg.command_text = "What time is it?"
        msg.source = "user"
        msg.command_type = "query"
        msg.priority = 0
        msg.command_id = "cmd_002"
        msg.context_id = "ctx_002"

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "text_interface")
        self.assertEqual(msg.command_text, "What time is it?")
        self.assertEqual(msg.source, "user")
        self.assertEqual(msg.command_type, "query")
        self.assertEqual(msg.priority, 0)
        self.assertEqual(msg.command_id, "cmd_002")
        self.assertEqual(msg.context_id, "ctx_002")

    def test_text_response_creation(self):
        """Test creating a TextResponse message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "text_interface"

        # Create a TextResponse message
        msg = self.TextResponse()
        msg.header = header
        msg.response_text = "The current time is 12:30 PM."
        msg.response_type = "answer"
        msg.status = 0
        msg.command_id = "cmd_002"
        msg.context_id = "ctx_002"
        msg.data = '{"time": "12:30 PM"}'

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "text_interface")
        self.assertEqual(msg.response_text, "The current time is 12:30 PM.")
        self.assertEqual(msg.response_type, "answer")
        self.assertEqual(msg.status, 0)
        self.assertEqual(msg.command_id, "cmd_002")
        self.assertEqual(msg.context_id, "ctx_002")
        self.assertEqual(msg.data, '{"time": "12:30 PM"}')

    def test_voice_command_creation(self):
        """Test creating a VoiceCommand message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "voice_interface"

        # Create a VoiceCommand message - check the actual fields in the message definition
        # Since we don't have the full VoiceCommand.msg definition, we'll skip detailed testing
        msg = self.VoiceCommand()
        msg.header = header
        
        # Verify the header field
        self.assertEqual(msg.header.frame_id, "voice_interface")

    def test_voice_response_creation(self):
        """Test creating a VoiceResponse message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "voice_interface"

        # Create a VoiceResponse message - check the actual fields in the message definition
        # Since we don't have the full VoiceResponse.msg definition, we'll skip detailed testing
        msg = self.VoiceResponse()
        msg.header = header
        
        # Verify the header field
        self.assertEqual(msg.header.frame_id, "voice_interface")

    def test_message_serialization(self):
        """Test serializing and deserializing messages."""
        # Test AICommand serialization
        ai_command = self.AICommand()
        ai_command.command_type = "navigation"
        ai_command.command_data = '{"command": "Move forward 1 meter"}'
        
        serialized_msg = serialize_message(ai_command)
        deserialized_msg = deserialize_message(serialized_msg, self.AICommand)
        
        self.assertEqual(deserialized_msg.command_type, "navigation")
        self.assertEqual(deserialized_msg.command_data, '{"command": "Move forward 1 meter"}')
        
        # Test TextResponse serialization
        text_response = self.TextResponse()
        text_response.response_text = "The current time is 12:30 PM."
        text_response.response_type = "answer"
        
        serialized_msg = serialize_message(text_response)
        deserialized_msg = deserialize_message(serialized_msg, self.TextResponse)
        
        self.assertEqual(deserialized_msg.response_text, "The current time is 12:30 PM.")
        self.assertEqual(deserialized_msg.response_type, "answer")
        
        # Test AIStatus serialization
        ai_status = self.AIStatus()
        ai_status.status = "active"
        ai_status.confidence = 0.95
        
        serialized_msg = serialize_message(ai_status)
        deserialized_msg = deserialize_message(serialized_msg, self.AIStatus)
        
        self.assertEqual(deserialized_msg.status, "active")
        self.assertAlmostEqual(deserialized_msg.confidence, 0.95, places=5)

if __name__ == '__main__':
    unittest.main()