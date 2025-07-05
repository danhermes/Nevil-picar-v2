#!/usr/bin/env python3

"""
Integration test for speech recognition and synthesis coordination.
Tests that speech recognition pauses when speech synthesis is active.
"""

import os
import sys
import time
import threading
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from nevil_interfaces_ai_msgs.msg import DialogState, VoiceResponse, TextResponse

# Add the package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'nevil_interfaces_ai'))

from nevil_interfaces_ai.speech_recognition_node import SpeechRecognitionNode
from nevil_interfaces_ai.speech_synthesis_node import SpeechSynthesisNode


class TestSpeechCoordination(unittest.TestCase):
    """Test speech recognition and synthesis coordination."""
    
    @classmethod
    def setUpClass(cls):
        """Set up the test environment."""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up the test environment."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test."""
        self.executor = MultiThreadedExecutor()
        
        # Mock the audio hardware interface to avoid hardware dependencies
        with patch('nevil_interfaces_ai.speech_recognition_node.AudioHardwareInterface') as mock_audio_hw_rec:
            # Configure mock to return proper string values
            mock_audio_hw_rec.return_value.recognize_speech.return_value = "test speech"
            mock_audio_hw_rec.return_value.listen_for_speech.return_value = Mock()
            self.speech_recognition_node = SpeechRecognitionNode()
        
        with patch('nevil_interfaces_ai.speech_synthesis_node.AudioHardwareInterface') as mock_audio_hw_synth:
            # Configure mock for synthesis
            mock_audio_hw_synth.return_value.speak_text.return_value = None
            self.speech_synthesis_node = SpeechSynthesisNode()
        
        # Create a test publisher for speaking status
        self.test_node = Node('test_speech_coordination')
        self.speaking_status_pub = self.test_node.create_publisher(
            Bool,
            '/nevil/speaking_status',
            10
        )
        
        self.dialog_state_pub = self.test_node.create_publisher(
            DialogState,
            '/nevil/dialog_state',
            10
        )
        
        self.voice_response_pub = self.test_node.create_publisher(
            VoiceResponse,
            '/nevil/voice_response',
            10
        )
        
        # Add nodes to executor
        self.executor.add_node(self.speech_recognition_node)
        self.executor.add_node(self.speech_synthesis_node)
        self.executor.add_node(self.test_node)
        
        # Start executor in a separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        
        # Allow time for nodes to initialize
        time.sleep(0.5)
    
    def tearDown(self):
        """Clean up after each test."""
        self.speech_recognition_node.shutdown()
        self.speech_synthesis_node.shutdown()
        
        self.executor.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)
        
        self.speech_recognition_node.destroy_node()
        self.speech_synthesis_node.destroy_node()
        self.test_node.destroy_node()
    
    def test_speech_recognition_pauses_when_speaking(self):
        """Test that speech recognition pauses when speech synthesis is active."""
        # Initially, speech recognition should be listening
        self.assertTrue(self.speech_recognition_node.is_listening)
        
        # Publish speaking status = True (synthesis starts)
        speaking_msg = Bool()
        speaking_msg.data = True
        self.speaking_status_pub.publish(speaking_msg)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should now be paused
        self.assertFalse(self.speech_recognition_node.is_listening)
        
        # Publish speaking status = False (synthesis stops)
        speaking_msg.data = False
        self.speaking_status_pub.publish(speaking_msg)
        
        # Set dialog state to listening to enable resume
        dialog_state = DialogState()
        dialog_state.context_id = "test_context"
        dialog_state.state = "listening"
        dialog_state.mode = "command"
        dialog_state.dialog_history = "[]"
        dialog_state.turn_count = 0
        dialog_state.environment_context = "{}"
        self.dialog_state_pub.publish(dialog_state)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should resume listening
        self.assertTrue(self.speech_recognition_node.is_listening)
    
    def test_speech_synthesis_publishes_speaking_status(self):
        """Test that speech synthesis publishes speaking status correctly."""
        # Create a mock subscriber to track speaking status
        speaking_status_received = []
        
        def speaking_status_callback(msg):
            speaking_status_received.append(msg.data)
        
        speaking_status_sub = self.test_node.create_subscription(
            Bool,
            '/nevil/speaking_status',
            speaking_status_callback,
            10
        )
        
        # Trigger speech synthesis
        voice_response = VoiceResponse()
        voice_response.text_to_speak = "Hello, this is a test"
        voice_response.voice_id = ""
        voice_response.speaking_rate = 1.0
        voice_response.pitch = 1.0
        voice_response.volume = 1.0
        voice_response.command_id = "test_command"
        voice_response.context_id = "test_context"
        
        self.voice_response_pub.publish(voice_response)
        
        # Allow time for speech synthesis processing
        time.sleep(1.0)
        
        # Should have received at least one speaking status update
        self.assertGreater(len(speaking_status_received), 0)
        
        # Clean up subscription
        self.test_node.destroy_subscription(speaking_status_sub)
    
    def test_dialog_state_coordination(self):
        """Test that dialog state changes coordinate speech recognition properly."""
        # Set dialog state to speaking
        dialog_state = DialogState()
        dialog_state.context_id = "test_context"
        dialog_state.state = "speaking"
        dialog_state.mode = "command"
        dialog_state.dialog_history = "[]"
        dialog_state.turn_count = 0
        dialog_state.environment_context = "{}"
        self.dialog_state_pub.publish(dialog_state)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should be stopped
        self.assertFalse(self.speech_recognition_node.is_listening)
        
        # Set dialog state to listening
        dialog_state.state = "listening"
        self.dialog_state_pub.publish(dialog_state)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should be started
        self.assertTrue(self.speech_recognition_node.is_listening)
    
    def test_no_self_listening_during_speech(self):
        """Test that Nevil doesn't listen to himself while speaking."""
        # Start with listening enabled
        self.assertTrue(self.speech_recognition_node.is_listening)
        
        # Simulate speech synthesis starting
        speaking_msg = Bool()
        speaking_msg.data = True
        self.speaking_status_pub.publish(speaking_msg)
        
        # Set dialog state to speaking
        dialog_state = DialogState()
        dialog_state.context_id = "test_context"
        dialog_state.state = "speaking"
        dialog_state.mode = "command"
        dialog_state.dialog_history = "[]"
        dialog_state.turn_count = 0
        dialog_state.environment_context = "{}"
        self.dialog_state_pub.publish(dialog_state)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should be paused
        self.assertFalse(self.speech_recognition_node.is_listening)
        
        # Verify that the current dialog state is tracked
        self.assertEqual(self.speech_recognition_node.current_dialog_state, "speaking")
        
        # Simulate speech synthesis ending
        speaking_msg.data = False
        self.speaking_status_pub.publish(speaking_msg)
        
        # Set dialog state back to listening
        dialog_state.state = "listening"
        self.dialog_state_pub.publish(dialog_state)
        
        # Allow time for message processing
        time.sleep(0.2)
        
        # Speech recognition should resume
        self.assertTrue(self.speech_recognition_node.is_listening)
        self.assertEqual(self.speech_recognition_node.current_dialog_state, "listening")


class TestSpeechCoordinationRunner:
    """Test runner for speech coordination tests."""
    
    def run_tests(self):
        """Run all speech coordination tests."""
        print("Running speech coordination integration tests...")
        
        # Create test suite
        suite = unittest.TestLoader().loadTestsFromTestCase(TestSpeechCoordination)
        
        # Run tests
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Return success status
        return result.wasSuccessful()


def main():
    """Main test function."""
    test_runner = TestSpeechCoordinationRunner()
    success = test_runner.run_tests()
    
    if success:
        print("\n✅ All speech coordination tests passed!")
        return 0
    else:
        print("\n❌ Some speech coordination tests failed!")
        return 1


if __name__ == '__main__':
    exit(main())