#!/usr/bin/env python3
"""
Test OpenAI API call and message receipt integration
Tests the complete flow: ROS2 message → OpenAI API → Response
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import sys
import os

# Add the package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'nevil_interfaces_ai'))

class OpenAIMessageFlowTest(Node):
    """Test node to verify OpenAI integration with ROS2 messages"""
    
    def __init__(self):
        super().__init__('openai_message_flow_test')
        
        print("🧪 Initializing OpenAI Message Flow Test...")
        
        # Test OpenAI first
        self.test_openai_direct()
        
        # Import and test the integrated AI interface
        from integrated_ai_interface import IntegratedAIInterface
        
        # Create AI interface
        self.ai_interface = IntegratedAIInterface()
        
        # Create test publisher and subscriber
        self.command_pub = self.create_publisher(
            String,
            '/nevil/speech_command',
            10
        )
        
        self.response_sub = self.create_subscription(
            String,
            '/nevil/text_response',
            self.response_callback,
            10
        )
        
        self.received_responses = []
        self.test_messages = [
            "Hello, can you hear me?",
            "What is your name?",
            "How are you doing today?",
            "Tell me a joke",
            "Thank you"
        ]
        
        print("✅ OpenAI Message Flow Test initialized")
    
    def test_openai_direct(self):
        """Test OpenAI API directly"""
        print("\n=== TESTING OPENAI API DIRECTLY ===")
        
        try:
            import openai
            print("✅ OpenAI library imported")
            
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                print("❌ No OPENAI_API_KEY found")
                return False
            
            client = openai.OpenAI(api_key=api_key)
            print("✅ OpenAI client created")
            
            # Test API call
            print("🔄 Making direct OpenAI API call...")
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are Nevil, a helpful robot assistant. Keep responses short."},
                    {"role": "user", "content": "Hello, this is a test message"}
                ],
                max_tokens=100,
                temperature=0.7
            )
            
            ai_response = response.choices[0].message.content.strip()
            print(f"✅ Direct OpenAI Response: '{ai_response}'")
            print("✅ Direct OpenAI API test PASSED!")
            return True
            
        except Exception as e:
            print(f"❌ Direct OpenAI API test FAILED: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def response_callback(self, msg):
        """Handle received responses"""
        response = msg.data
        self.received_responses.append(response)
        print(f"📨 Received AI response: '{response}'")
    
    def send_test_message(self, message):
        """Send a test message"""
        print(f"📤 Sending message: '{message}'")
        msg = String()
        msg.data = message
        self.command_pub.publish(msg)
    
    def run_message_flow_test(self):
        """Run the complete message flow test"""
        print("\n=== TESTING MESSAGE FLOW WITH OPENAI ===")
        
        # Wait for everything to initialize
        time.sleep(2)
        
        for i, message in enumerate(self.test_messages):
            print(f"\n📋 Test {i+1}/{len(self.test_messages)}: '{message}'")
            
            initial_count = len(self.received_responses)
            
            # Send message
            self.send_test_message(message)
            
            # Wait for response
            start_time = time.time()
            timeout = 15  # Longer timeout for OpenAI API calls
            
            while len(self.received_responses) <= initial_count and time.time() - start_time < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if len(self.received_responses) > initial_count:
                latest_response = self.received_responses[-1]
                print(f"✅ Test {i+1} PASSED - Response: '{latest_response}'")
            else:
                print(f"❌ Test {i+1} FAILED - No response received within {timeout}s")
            
            # Wait between tests
            time.sleep(3)
        
        # Summary
        print(f"\n📊 MESSAGE FLOW TEST SUMMARY:")
        print(f"   Messages sent: {len(self.test_messages)}")
        print(f"   Responses received: {len(self.received_responses)}")
        
        if len(self.received_responses) >= len(self.test_messages):
            print("✅ MESSAGE FLOW TEST PASSED!")
            return True
        else:
            print("❌ MESSAGE FLOW TEST FAILED!")
            return False
    
    def cleanup(self):
        """Clean up resources"""
        if hasattr(self, 'ai_interface'):
            self.ai_interface.cleanup()
            self.ai_interface.destroy_node()

def main():
    """Main test function"""
    rclpy.init()
    
    try:
        test_node = OpenAIMessageFlowTest()
        
        print("🚀 Starting OpenAI Message Flow Test")
        print("=" * 60)
        
        # Run the message flow test
        success = test_node.run_message_flow_test()
        
        if success:
            print("\n🎉 ALL TESTS PASSED!")
            print("✅ OpenAI API integration working correctly")
            print("✅ Message flow working correctly")
        else:
            print("\n⚠️ SOME TESTS FAILED!")
            print("❌ Check OpenAI API key and network connection")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'test_node' in locals():
            test_node.cleanup()
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()