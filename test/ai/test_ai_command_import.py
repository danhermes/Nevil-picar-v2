#!/usr/bin/env python3

import sys
import os

# Test AI command message import
print("Testing AI command message import...")

try:
    from nevil_interfaces_ai_msgs.msg import AICommand
    print("SUCCESS: AICommand imported successfully!")
    print(f"AICommand class: {AICommand}")
    
    # Test creating an instance
    cmd = AICommand()
    cmd.command_type = "test"
    cmd.command_data = '{"test": "data"}'
    print(f"Created AICommand instance: {cmd}")
    
except ImportError as e:
    print(f"FAILED: Could not import AICommand: {e}")
    print("This means the ROS2 environment is not properly sourced")
    print("Run: source install/setup.bash")

print("\nTesting other AI messages...")
try:
    from nevil_interfaces_ai_msgs.msg import AIStatus, TextCommand, VoiceCommand
    print("SUCCESS: Other AI messages imported successfully!")
except ImportError as e:
    print(f"FAILED: Could not import other AI messages: {e}")

print("\nPython path:")
for path in sys.path:
    print(f"  {path}")

print(f"\nCurrent working directory: {os.getcwd()}")