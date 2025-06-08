#!/usr/bin/env python3

"""
Test script for environment variable loading.

This script tests the loading of environment variables from the .env file.
"""

import os
import sys
import rclpy
from rclpy.node import Node
from pathlib import Path

# Define local environment variable utilities
def load_env_file(env_file_path=None):
    """
    Load environment variables from a .env file.
    
    Args:
        env_file_path: Path to the .env file (default: project root .env)
        
    Returns:
        dict: Dictionary of environment variables loaded from the file
    """
    # If no path is provided, look for .env in the project root
    if env_file_path is None:
        # Try to find the project root by looking for .env file
        current_dir = Path.cwd()
        while current_dir != current_dir.parent:
            env_path = current_dir / '.env'
            if env_path.exists():
                env_file_path = env_path
                break
            current_dir = current_dir.parent
    
    # If still None, use the current directory
    if env_file_path is None:
        env_file_path = Path.cwd() / '.env'
    
    # Convert to Path object if it's a string
    if isinstance(env_file_path, str):
        env_file_path = Path(env_file_path)
    
    # Load environment variables from the .env file
    env_vars = {}
    if env_file_path.exists():
        print(f'Loading environment variables from {env_file_path}')
        with open(env_file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                key, value = line.split('=', 1)
                env_vars[key.strip()] = value.strip()
                # Also set in the environment
                os.environ[key.strip()] = value.strip()
    else:
        print(f'.env file not found at {env_file_path}')
    
    return env_vars

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

class EnvLoadingTest(Node):
    """
    Test node for environment variable loading.
    """
    
    def __init__(self):
        super().__init__('env_loading_test')
        
        # Load environment variables
        self.get_logger().info('Loading environment variables...')
        env_vars = load_env_file()
        
        # Print environment variables
        self.get_logger().info('Environment variables loaded:')
        self.get_logger().info(f'  OPENAI_API_KEY: {"Set" if "OPENAI_API_KEY" in os.environ else "Not set"} (for language processing, not needed for Whisper)')
        self.get_logger().info(f'  SPEECH_RECOGNITION_LANGUAGE: {get_env_var("SPEECH_RECOGNITION_LANGUAGE", "en")}')
        self.get_logger().info(f'  SPEECH_RECOGNITION_ENERGY_THRESHOLD: {get_env_var("SPEECH_RECOGNITION_ENERGY_THRESHOLD", 300)}')
        self.get_logger().info(f'  SPEECH_RECOGNITION_PAUSE_THRESHOLD: {get_env_var("SPEECH_RECOGNITION_PAUSE_THRESHOLD", 0.5)}')
        self.get_logger().info(f'  SPEECH_RECOGNITION_DYNAMIC_ENERGY: {get_env_var("SPEECH_RECOGNITION_DYNAMIC_ENERGY", "true")}')
        self.get_logger().info(f'  SPEECH_SYNTHESIS_VOICE: {get_env_var("SPEECH_SYNTHESIS_VOICE", "onyx")}')
        self.get_logger().info(f'  SPEECH_SYNTHESIS_RATE: {get_env_var("SPEECH_SYNTHESIS_RATE", 200)}')
        self.get_logger().info(f'  SPEECH_SYNTHESIS_VOLUME: {get_env_var("SPEECH_SYNTHESIS_VOLUME", 1.0)}')
        self.get_logger().info(f'  WHISPER_MODEL: {get_env_var("WHISPER_MODEL", "small")} (offline, no API key needed)')
        
        # Check if OpenAI API key is set
        if "OPENAI_API_KEY" in os.environ:
            self.get_logger().info('OpenAI API key is set for language processing (not needed for Whisper)')
        else:
            self.get_logger().info('OpenAI API key is not set (only needed for language processing, not for Whisper)')
        
        self.get_logger().info('Environment variable loading test completed')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    test_node = EnvLoadingTest()
    
    try:
        rclpy.spin_once(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()