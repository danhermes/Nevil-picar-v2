#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SimpleAIInterface(Node):
    def __init__(self):
        super().__init__('simple_ai_interface')
        
        # Load environment variables
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
        dotenv_path = os.path.join(project_root, '.env')
        load_dotenv(dotenv_path=dotenv_path)
        
        # QoS profile for ai messages
        self.ai_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        ) 

        # Publishers
        self.text_response_pub = self.create_publisher(String, '/nevil/text_response', qos_profile=self.ai_qos)
        
        # Subscribers
        self.text_command_sub = self.create_subscription(
            String,
            '/nevil/text_command',
            self.handle_text_command,
            qos_profile=self.ai_qos
        )
        
        # OpenAI API configuration
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if self.openai_api_key:
            self.get_logger().info('OpenAI API key loaded from environment')
        else:
            self.get_logger().warning('OpenAI API key not set, using fallback responses')
        
        self.get_logger().info('Simple AI Interface initialized')
    
    def handle_text_command(self, msg):
        """Handle incoming text commands from speech recognition"""
        self.get_logger().info(f'Received text command: {msg.data}')
        
        # Process the command with OpenAI or use a fallback
        response = self.process_with_openai(msg.data)
        
        # Publish the response
        response_msg = String()
        response_msg.data = response
        self.text_response_pub.publish(response_msg)
        self.get_logger().info(f'Published AI response: {response}')
    
    def process_with_openai(self, text):
        """Process text with OpenAI API and return response"""
        try:
            if not self.openai_api_key:
                return self.generate_fallback_response(text)
            
            # Import here to avoid dependency issues if OpenAI is not installed
            from openai import OpenAI
            
            # Initialize the client
            client = OpenAI(api_key=self.openai_api_key)
            
            # Call OpenAI API
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for a robot named Nevil. Keep responses concise and clear."},
                    {"role": "user", "content": text}
                ],
                max_tokens=150
            )
            
            response_text = response.choices[0].message.content
            self.get_logger().info(f'OpenAI response: {response_text}')
            return response_text
        
        except ImportError:
            self.get_logger().error('OpenAI package not installed')
            return self.generate_fallback_response(text)
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
            return self.generate_fallback_response(text)
    
    def generate_fallback_response(self, text):
        """Generate a fallback response when OpenAI is unavailable"""
        text_lower = text.lower()
        
        if 'hello' in text_lower or 'hi' in text_lower:
            return "Hello! I'm Nevil. How can I help you?"
        
        if 'name' in text_lower:
            return "My name is Nevil. I'm a robot assistant."
        
        if 'help' in text_lower:
            return "I can help you with information, navigation, or just have a conversation."
        
        if 'thank' in text_lower:
            return "You're welcome! Is there anything else I can help with?"
        
        if 'move' in text_lower or 'go' in text_lower:
            return "I understand you want me to move, but I need more specific instructions."
        
        if 'stop' in text_lower:
            return "Stopping now."
        
        # Default response
        return "I'm not sure how to respond to that. Could you rephrase your request?"

def main(args=None):
    rclpy.init(args=args)
    
    simple_ai_interface = SimpleAIInterface()
    
    try:
        rclpy.spin(simple_ai_interface)
    except KeyboardInterrupt:
        pass
    finally:
        simple_ai_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()