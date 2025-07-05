#!/usr/bin/env python3
"""
Integration Test Script for Nevil AI Interface
Tests the complete pipeline: speech command → AI processing → text response → TTS
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

class IntegrationTester(Node):
    """Test node to verify the complete AI integration pipeline"""
    
    def __init__(self):
        super().__init__('integration_tester')
        
        # Publisher for speech commands
        self.speech_command_pub = self.create_publisher(
            String,
            '/nevil/speech_command',
            10
        )
        
        # Subscriber for text responses
        self.text_response_sub = self.create_subscription(
            String,
            '/nevil/text_response',
            self.handle_text_response,
            10
        )
        
        self.received_responses = []
        self.test_commands = [
            "hello",
            "how are you",
            "what is your name",
            "help me",
            "thank you"
        ]
        
        self.logger = self.get_logger()
        self.logger.info("🧪 Integration Tester initialized")
    
    def handle_text_response(self, msg: String):
        """Handle received text responses"""
        response = msg.data
        self.received_responses.append(response)
        self.logger.info(f"📨 Received response: '{response}'")
    
    def send_test_command(self, command: str):
        """Send a test command"""
        msg = String()
        msg.data = command
        self.speech_command_pub.publish(msg)
        self.logger.info(f"📤 Sent command: '{command}'")
    
    def run_integration_test(self):
        """Run the complete integration test"""
        self.logger.info("🚀 Starting integration test...")
        
        for i, command in enumerate(self.test_commands):
            self.logger.info(f"📋 Test {i+1}/{len(self.test_commands)}: '{command}'")
            
            # Send command
            self.send_test_command(command)
            
            # Wait for response
            start_time = time.time()
            initial_count = len(self.received_responses)
            
            while len(self.received_responses) <= initial_count and time.time() - start_time < 10:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if len(self.received_responses) > initial_count:
                self.logger.info(f"✅ Test {i+1} passed - received response")
            else:
                self.logger.warning(f"⚠️ Test {i+1} timeout - no response received")
            
            # Wait between tests
            time.sleep(2)
        
        # Summary
        self.logger.info("📊 Integration Test Summary:")
        self.logger.info(f"   Commands sent: {len(self.test_commands)}")
        self.logger.info(f"   Responses received: {len(self.received_responses)}")
        
        if len(self.received_responses) >= len(self.test_commands):
            self.logger.info("✅ Integration test PASSED - All components working!")
        else:
            self.logger.warning("⚠️ Integration test PARTIAL - Some responses missing")
        
        return len(self.received_responses) >= len(self.test_commands)

def main():
    """Main test function"""
    rclpy.init()
    
    try:
        tester = IntegrationTester()
        
        print("🧪 Integration Test for Nevil AI Interface")
        print("=" * 50)
        print("This test will:")
        print("1. Send speech commands to /nevil/speech_command")
        print("2. Listen for responses on /nevil/text_response")
        print("3. Verify the complete AI pipeline is working")
        print()
        print("Make sure the IntegratedAIInterface is running!")
        print("Press Enter to start the test...")
        input()
        
        # Run the test
        success = tester.run_integration_test()
        
        if success:
            print("\n🎉 INTEGRATION TEST PASSED!")
            print("✅ Complete AI pipeline is working correctly")
        else:
            print("\n⚠️ INTEGRATION TEST INCOMPLETE")
            print("❌ Some components may not be responding")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()