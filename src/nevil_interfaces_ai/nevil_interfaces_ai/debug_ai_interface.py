#!/usr/bin/env python3
"""
Debug version of Integrated AI Interface
Shows verbose logging to debug message reception issues
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from typing import Optional, Dict, Any
import logging
import time

# Import the fixed audio hardware interface
from audio_hardware_interface import AudioHardwareInterface, OPENAI_AVAILABLE

# OpenAI imports (conditional)
if OPENAI_AVAILABLE:
    import openai

class DebugAIInterface(Node):
    """Debug version of AI Interface with verbose logging"""
    
    def __init__(self):
        super().__init__('debug_ai_interface')
        
        # Initialize logging
        self.logger = self.get_logger()
        self.logger.info("🐛 DEBUG: Initializing Debug AI Interface...")
        
        # Initialize OpenAI client if available
        self.openai_client = None
        if OPENAI_AVAILABLE:
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.openai_client = openai.OpenAI(api_key=api_key)
                self.logger.info("🐛 DEBUG: OpenAI client initialized")
            else:
                self.logger.warning("🐛 DEBUG: OPENAI_API_KEY not found in environment")
        else:
            self.logger.warning("🐛 DEBUG: OpenAI not available - using fallback responses")
        
        # Initialize the fixed audio hardware interface
        try:
            self.audio_hw = AudioHardwareInterface(self)
            self.logger.info(f"🐛 DEBUG: Audio Hardware Interface initialized (simulation_mode: {self.audio_hw.simulation_mode})")
        except Exception as e:
            self.logger.error(f"🐛 DEBUG: Failed to initialize AudioHardwareInterface: {e}")
            self.audio_hw = None
        
        # ROS2 Publishers and Subscribers using std_msgs/String
        self.logger.info("🐛 DEBUG: Creating publishers and subscribers...")
        
        self.text_response_pub = self.create_publisher(
            String, 
            '/nevil/text_response', 
            10
        )
        self.logger.info("🐛 DEBUG: Created text_response publisher on /nevil/text_response")
        
        self.speech_command_sub = self.create_subscription(
            String,
            '/nevil/speech_command',
            self.handle_speech_command,
            10
        )
        self.logger.info("🐛 DEBUG: Created speech_command subscriber on /nevil/speech_command")
        
        # Message counter for debugging
        self.message_count = 0
        
        self.logger.info("🐛 DEBUG: Debug AI Interface ready!")
        self.logger.info("🐛 DEBUG: Waiting for messages on /nevil/speech_command...")
    
    def handle_speech_command(self, msg: String) -> None:
        """Handle incoming speech commands with debug logging"""
        self.message_count += 1
        self.logger.info(f"🐛 DEBUG: MESSAGE #{self.message_count} RECEIVED!")
        self.logger.info(f"🐛 DEBUG: Raw message: {msg}")
        self.logger.info(f"🐛 DEBUG: Message data: '{msg.data}'")
        
        try:
            command_text = msg.data.strip()
            if not command_text:
                self.logger.warning("🐛 DEBUG: Empty command text received")
                return
            
            self.logger.info(f"🐛 DEBUG: Processing command: '{command_text}'")
            
            # Generate AI response
            self.logger.info("🐛 DEBUG: Generating AI response...")
            response_text = self.generate_ai_response(command_text)
            self.logger.info(f"🐛 DEBUG: Generated response: '{response_text}'")
            
            # Publish the response
            self.logger.info("🐛 DEBUG: Publishing response...")
            response_msg = String()
            response_msg.data = response_text
            self.text_response_pub.publish(response_msg)
            self.logger.info(f"🐛 DEBUG: Response published successfully!")
            
            # Convert response to speech using the fixed audio hardware interface
            if self.audio_hw:
                self.logger.info("🐛 DEBUG: Converting to speech...")
                self.audio_hw.text_to_speech(response_text)
                self.logger.info("🐛 DEBUG: Speech conversion completed!")
            else:
                self.logger.warning("🐛 DEBUG: No audio hardware available for TTS")
            
        except Exception as e:
            self.logger.error(f"🐛 DEBUG: Error handling speech command: {e}")
            import traceback
            self.logger.error(f"🐛 DEBUG: Traceback: {traceback.format_exc()}")
            
            # Send error response
            error_msg = String()
            error_msg.data = "Sorry, I encountered an error processing your request."
            self.text_response_pub.publish(error_msg)
    
    def generate_ai_response(self, user_input: str) -> str:
        """Generate an AI response with debug logging"""
        self.logger.info(f"🐛 DEBUG: Generating response for: '{user_input}'")
        
        try:
            if self.openai_client:
                self.logger.info("🐛 DEBUG: Using OpenAI for response")
                return self._generate_openai_response(user_input)
            else:
                self.logger.info("🐛 DEBUG: Using fallback response")
                return self._generate_fallback_response(user_input)
        except Exception as e:
            self.logger.error(f"🐛 DEBUG: Error generating AI response: {e}")
            return "I'm sorry, I'm having trouble processing your request right now."
    
    def _generate_fallback_response(self, user_input: str) -> str:
        """Generate fallback response with debug logging"""
        self.logger.info(f"🐛 DEBUG: Generating fallback response for: '{user_input}'")
        
        user_lower = user_input.lower()
        
        # Simple rule-based responses
        if any(greeting in user_lower for greeting in ['hello', 'hi', 'hey']):
            response = "Hello! I'm Nevil, your robot assistant. How can I help you today?"
        elif any(question in user_lower for question in ['how are you', 'how do you feel']):
            response = "I'm doing great! My systems are running smoothly and I'm ready to help."
        elif any(name in user_lower for name in ['your name', 'who are you']):
            response = "I'm Nevil, a Raspberry Pi robot car with AI capabilities. Nice to meet you!"
        elif any(help_word in user_lower for help_word in ['help', 'assist', 'support']):
            response = "I can help you with navigation, answer questions, and have conversations. What would you like to do?"
        else:
            response = f"I heard you say '{user_input}'. I'm still learning, but I'm here to help however I can!"
        
        self.logger.info(f"🐛 DEBUG: Generated fallback response: '{response}'")
        return response
    
    def _generate_openai_response(self, user_input: str) -> str:
        """Generate OpenAI response with debug logging"""
        self.logger.info(f"🐛 DEBUG: Calling OpenAI API for: '{user_input}'")
        
        try:
            # Create system message for Nevil
            system_message = {
                "role": "system",
                "content": "You are Nevil, a helpful AI assistant for a Raspberry Pi robot car. "
                          "Keep responses concise and friendly. You can help with navigation, "
                          "information, and general conversation."
            }
            
            # Prepare messages for OpenAI
            messages = [
                system_message,
                {"role": "user", "content": user_input}
            ]
            
            # Call OpenAI API
            self.logger.info("🐛 DEBUG: Making OpenAI API call...")
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=messages,
                max_tokens=150,
                temperature=0.7
            )
            
            ai_response = response.choices[0].message.content.strip()
            self.logger.info(f"🐛 DEBUG: OpenAI response: '{ai_response}'")
            return ai_response
            
        except Exception as e:
            self.logger.error(f"🐛 DEBUG: OpenAI API error: {e}")
            return self._generate_fallback_response(user_input)
    
    def cleanup(self):
        """Clean up resources with debug logging"""
        self.logger.info("🐛 DEBUG: Cleaning up resources...")
        try:
            if self.audio_hw:
                self.audio_hw.cleanup()
            self.logger.info("🐛 DEBUG: Debug AI Interface cleaned up")
        except Exception as e:
            self.logger.error(f"🐛 DEBUG: Error during cleanup: {e}")

def main(args=None):
    """Main entry point with debug logging"""
    rclpy.init(args=args)
    
    try:
        ai_interface = DebugAIInterface()
        
        print("🐛 DEBUG AI Interface running...")
        print("📡 Listening for speech commands on /nevil/speech_command")
        print("📢 Publishing responses on /nevil/text_response")
        print("🎵 Using fixed AudioHardwareInterface for TTS")
        print("🐛 Debug mode: Verbose logging enabled")
        print("Press Ctrl+C to stop")
        
        # Spin with debug info
        print("🐛 DEBUG: Starting to spin...")
        rclpy.spin(ai_interface)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Debug AI Interface...")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        print(f"🐛 DEBUG: Traceback: {traceback.format_exc()}")
    finally:
        if 'ai_interface' in locals():
            ai_interface.cleanup()
            ai_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()