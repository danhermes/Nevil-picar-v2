#!/usr/bin/env python3

"""
Test script for the Audio Hardware Interface.

This script tests the basic functionality of the Audio Hardware Interface,
including speech recognition and synthesis.
"""

import os
import time
import sys
import rclpy
from rclpy.node import Node

# Add the current directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the audio hardware interface directly
from audio_hardware_interface import AudioHardwareInterface

# Define a local get_env_var function
def get_env_var(name, default=None):
    """
    Get an environment variable, with fallback to a default value.
    
    Args:
        name: Name of the environment variable
        default: Default value if the environment variable is not set
        
    Returns:
        The value of the environment variable, or the default value
    """
    return os.environ.get(name, default)

class AudioHardwareTest(Node):
    """
    Test node for the Audio Hardware Interface.
    """
    
    def __init__(self):
        super().__init__('audio_hardware_test')
        
        # Initialize audio hardware interface
        self.get_logger().info('Initializing audio hardware interface...')
        self.audio_hw = AudioHardwareInterface(self)
        
        # Set speech recognition parameters
        self.audio_hw.set_speech_recognition_parameters(
            energy_threshold=int(get_env_var('SPEECH_RECOGNITION_ENERGY_THRESHOLD', 300)),
            pause_threshold=float(get_env_var('SPEECH_RECOGNITION_PAUSE_THRESHOLD', 0.5)),
            dynamic_energy=get_env_var('SPEECH_RECOGNITION_DYNAMIC_ENERGY', 'true').lower() in ['true', '1', 'yes']
        )
        
        self.get_logger().info('Audio hardware interface initialized')
    
    def run_tests(self):
        """Run a series of tests on the audio hardware interface."""
        try:
            # Print environment configuration
            self.get_logger().info('Environment configuration:')
            self.get_logger().info(f'  OPENAI_API_KEY: {"Set" if "OPENAI_API_KEY" in os.environ else "Not set"} (for language processing, not needed for Whisper)')
            self.get_logger().info(f'  SPEECH_RECOGNITION_LANGUAGE: {get_env_var("SPEECH_RECOGNITION_LANGUAGE", "en")}')
            self.get_logger().info(f'  SPEECH_RECOGNITION_ENERGY_THRESHOLD: {get_env_var("SPEECH_RECOGNITION_ENERGY_THRESHOLD", 300)}')
            self.get_logger().info(f'  SPEECH_SYNTHESIS_VOICE: {get_env_var("SPEECH_SYNTHESIS_VOICE", "onyx")}')
            self.get_logger().info(f'  WHISPER_MODEL: {get_env_var("WHISPER_MODEL", "small")} (offline, no API key needed)')
            
            # Test 1: Speech synthesis
            self.get_logger().info('Test 1: Speech synthesis')
            self.audio_hw.speak_text("Hello, I am Nevil. Testing audio hardware interface.")
            time.sleep(1.0)
            
            # Test 2: Speech recognition
            self.get_logger().info('Test 2: Speech recognition')
            self.get_logger().info('Please speak something...')
            audio = self.audio_hw.listen_for_speech(timeout=5.0, phrase_time_limit=5.0)
            
            if audio:
                # Test 3: Speech recognition with Whisper
                self.get_logger().info('Test 3: Speech recognition with Whisper')
                text = self.audio_hw.recognize_speech(audio, api='auto')
                self.get_logger().info(f'Recognized: {text}')
                
                # Test 4: Repeat what was heard
                self.get_logger().info('Test 4: Repeat what was heard')
                self.audio_hw.speak_text(f"You said: {text}")
            
            # Test 5: Audio recording
            self.get_logger().info('Test 5: Audio recording')
            self.get_logger().info('Recording for 3 seconds...')
            recording_path = os.path.join(os.getcwd(), 'test_recording.wav')
            self.audio_hw.record_audio(duration=3.0, file_path=recording_path)
            self.get_logger().info(f'Recording saved to {recording_path}')
            
            # Test 6: Audio playback
            self.get_logger().info('Test 6: Audio playback')
            self.get_logger().info('Playing back the recording...')
            if os.path.exists(recording_path):
                self.audio_hw.play_audio_file(recording_path)
            else:
                self.get_logger().error(f'Recording file not found: {recording_path}')
            
            self.get_logger().info('All tests completed')
            
        except Exception as e:
            self.get_logger().error(f'Error during tests: {e}')
        finally:
            # Clean up
            self.audio_hw.cleanup()
    
    def shutdown(self):
        """Clean up resources."""
        self.audio_hw.cleanup()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    test_node = AudioHardwareTest()
    
    try:
        test_node.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        test_node.shutdown()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()