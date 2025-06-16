#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class DebugMessageReception(Node):
    def __init__(self):
        super().__init__('debug_message_reception')
        
        # Create subscriptions to both topics
        self.text_command_sub = self.create_subscription(
            String,
            '/nevil/text_command',
            self.text_command_callback,
            10
        )
        
        self.speech_command_sub = self.create_subscription(
            String,
            '/nevil/speech_command', 
            self.speech_command_callback,
            10
        )
        
        # Create publisher for responses
        self.text_response_pub = self.create_publisher(
            String,
            '/nevil/text_response',
            10
        )
        
        self.get_logger().info('🔄 Debug Message Reception Node Started')
        self.get_logger().info(f'📡 Subscribed to: /nevil/text_command')
        self.get_logger().info(f'📡 Subscribed to: /nevil/speech_command')
        self.get_logger().info(f'📤 Publishing to: /nevil/text_response')
        
        # Start a timer to show we're alive
        self.alive_timer = self.create_timer(5.0, self.alive_callback)
        self.message_count = 0
        
    def alive_callback(self):
        self.get_logger().info(f'💓 Node alive - received {self.message_count} messages so far')
        
    def text_command_callback(self, msg):
        self.message_count += 1
        self.get_logger().info(f'✅ RECEIVED TEXT COMMAND: "{msg.data}"')
        
        # Send immediate response
        response = String()
        response.data = f"ACK: Received text command '{msg.data}'"
        self.text_response_pub.publish(response)
        self.get_logger().info(f'📤 SENT RESPONSE: "{response.data}"')
        
    def speech_command_callback(self, msg):
        self.message_count += 1
        self.get_logger().info(f'✅ RECEIVED SPEECH COMMAND: "{msg.data}"')
        
        # Send immediate response
        response = String()
        response.data = f"ACK: Received speech command '{msg.data}'"
        self.text_response_pub.publish(response)
        self.get_logger().info(f'📤 SENT RESPONSE: "{response.data}"')

def main():
    print('🚀 Starting Debug Message Reception Test...')
    
    rclpy.init()
    
    try:
        node = DebugMessageReception()
        print('✅ Node created successfully')
        
        # Spin the node
        print('🔄 Starting to spin node...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n🛑 Shutting down...')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print('✅ Shutdown complete')

if __name__ == '__main__':
    main()