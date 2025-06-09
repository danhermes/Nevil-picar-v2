#!/usr/bin/env python3

"""
Simple test script for the Whisper TTS functionality in the AudioHardwareInterface.
This script doesn't rely on ROS2 or any other dependencies that might not be installed.
"""

import os
import sys
import time
from dotenv import load_dotenv

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src/nevil_interfaces_ai/nevil_interfaces_ai'))

# Import the audio_hardware_interface module directly
try:
    import audio_hardware_interface
    print("Successfully imported audio_hardware_interface")
except ImportError as e:
    print(f"Error importing audio_hardware_interface: {e}")
    print("Python path:", sys.path)
    sys.exit(1)

def main():
    """Main function to test the Whisper TTS functionality."""
    print("Testing Whisper TTS functionality...")
    
    # Load environment variables
    load_dotenv()
    print(f"OPENAI_API_KEY: {'Set' if 'OPENAI_API_KEY' in os.environ else 'Not set'}")
    
    # Create a mock logger
    class MockLogger:
        def info(self, msg):
            print(f"INFO: {msg}")
        def debug(self, msg):
            print(f"DEBUG: {msg}")
        def warn(self, msg):
            print(f"WARN: {msg}")
        def error(self, msg):
            print(f"ERROR: {msg}")
    
    # Create a mock node
    class MockNode:
        def __init__(self):
            self.logger = MockLogger()
        def get_logger(self):
            return self.logger
    
    # Create the audio hardware interface
    print("Creating AudioHardwareInterface...")
    audio_hw = audio_hardware_interface.AudioHardwareInterface(MockNode())
    
    # Test speech synthesis with Whisper TTS
    print("Testing speech synthesis with Whisper TTS...")
    audio_hw.speak_text("Hello, I am Nevil. This is a test of the Whisper text to speech system with the Onyx voice model.")
    
    print("Test completed.")

if __name__ == "__main__":
    main()