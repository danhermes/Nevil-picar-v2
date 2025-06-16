#!/usr/bin/env python3
"""
Minimal AI Interface Test - Debug message reception
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'nevil_interfaces_ai'))

class MinimalAITest(Node):
    """Minimal test to debug message reception"""
    
    def __init__(self):
        super().__init__('minimal_ai_test')
        
        print("🧪 Starting Minimal AI Test...")
        
        # Create subscribers for both topics
        self.text_command_sub = self.create_subscription(
            String,
            '/nevil/text_command',
            self.handle_text_command,
            10
        )
        
        self.speech_command_sub = self.create_subscription(
            String,
            '/nevil/speech_command',
            self.handle_speech_command,
            10
        )
        
        # Create publisher for responses
        self.response_pub = self.create_publisher(
            String,
            '/nevil/text_response',
            10
        )
        
        self.message_count = 0
        
        print("✅ Minimal AI Test ready!")
        print("📡 Listening on /nevil/text_command")
        print("📡 Listening on /nevil/speech_command")
        print("📢 Publishing on /nevil/text_response")
        print("🔍 Waiting for messages...")
    
    def handle_text_command(self, msg):
        """Handle text command messages"""
        self.message_count += 1
        print(f"🎉 TEXT COMMAND RECEIVED #{self.message_count}!")
        print(f"📨 Topic: /nevil/text_command")
        print(f"📨 Content: '{msg.data}'")
        
        # Send immediate response
        response = String()
        response.data = f"Received text command: {msg.data}"
        self.response_pub.publish(response)
        print(f"📤 Response sent: '{response.data}'")
        print("-" * 50)
    
    def handle_speech_command(self, msg):
        """Handle speech command messages"""
        self.message_count += 1
        print(f"🎉 SPEECH COMMAND RECEIVED #{self.message_count}!")
        print(f"📨 Topic: /nevil/speech_command")
        print(f"📨 Content: '{msg.data}'")
        
        # Send immediate response
        response = String()
        response.data = f"Received speech command: {msg.data}"
        self.response_pub.publish(response)
        print(f"📤 Response sent: '{response.data}'")
        print("-" * 50)

def main():
    """Main function"""
    rclpy.init()
    
    try:
        test_node = MinimalAITest()
        print("🚀 Spinning node - send messages to test...")
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()