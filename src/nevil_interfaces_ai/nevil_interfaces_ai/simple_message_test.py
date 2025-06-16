#!/usr/bin/env python3
"""
Simple message test to debug ROS2 communication
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleMessageTest(Node):
    """Simple test node to verify message reception"""
    
    def __init__(self):
        super().__init__('simple_message_test')
        
        print("🧪 Creating Simple Message Test Node...")
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            '/nevil/speech_command',
            self.message_callback,
            10
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            '/nevil/text_response',
            10
        )
        
        self.message_count = 0
        print("✅ Simple Message Test Node ready!")
        print("📡 Listening on /nevil/speech_command")
        print("📢 Publishing on /nevil/text_response")
    
    def message_callback(self, msg):
        """Handle incoming messages"""
        self.message_count += 1
        print(f"🎉 MESSAGE RECEIVED #{self.message_count}!")
        print(f"📨 Content: '{msg.data}'")
        
        # Send immediate response
        response = String()
        response.data = f"Echo: {msg.data} (message #{self.message_count})"
        self.publisher.publish(response)
        print(f"📤 Response sent: '{response.data}'")

def main():
    """Main function"""
    rclpy.init()
    
    try:
        node = SimpleMessageTest()
        print("🚀 Starting to spin...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()