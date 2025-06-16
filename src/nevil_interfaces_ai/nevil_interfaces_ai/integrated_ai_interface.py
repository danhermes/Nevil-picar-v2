#!/usr/bin/env python3
"""
Integrated AI Interface for Nevil-picar v2.0
Combines AI processing with the fixed audio hardware interface
Uses standardized std_msgs/String for all ROS2 communication
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from typing import Optional, Dict, Any
import logging

# Import the fixed audio hardware interface
from audio_hardware_interface import AudioHardwareInterface, OPENAI_AVAILABLE

# OpenAI imports (conditional)
if OPENAI_AVAILABLE:
    import openai

class IntegratedAIInterface(Node):
    """
    Integrated AI Interface that combines:
    1. Fixed AudioHardwareInterface (robot_hat + OpenAI TTS)
    2. OpenAI GPT for intelligent responses
    3. Standardized ROS2 communication using std_msgs/String
    """
    
    def __init__(self):
        super().__init__('integrated_ai_interface')
        
        # Initialize logging
        self.logger = self.get_logger()
        self.logger.info("🤖 Initializing Integrated AI Interface...")
        
        # Initialize OpenAI client if available
        self.openai_client = None
        if OPENAI_AVAILABLE:
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.openai_client = openai.OpenAI(api_key=api_key)
                self.logger.info("✅ OpenAI client initialized")
            else:
                self.logger.warning("⚠️ OPENAI_API_KEY not found in environment")
        else:
            self.logger.warning("⚠️ OpenAI not available - using fallback responses")
        
        # Initialize the fixed audio hardware interface
        try:
            self.audio_hw = AudioHardwareInterface(self)
            self.logger.info(f"✅ Audio Hardware Interface initialized (simulation_mode: {self.audio_hw.simulation_mode})")
        except Exception as e:
            self.logger.error(f"❌ Failed to initialize AudioHardwareInterface: {e}")
            self.audio_hw = None
        
        # ROS2 Publishers and Subscribers using std_msgs/String (per architecture docs)
        self.text_response_pub = self.create_publisher(
            String,
            '/nevil/text_response',
            10
        )
        
        # Subscribe to the correct topic per architecture: /nevil/text_command
        self.text_command_sub = self.create_subscription(
            String,
            '/nevil/text_command',
            self.handle_text_command,
            10
        )
        
        # Also subscribe to speech_command for backward compatibility
        self.speech_command_sub = self.create_subscription(
            String,
            '/nevil/speech_command',
            self.handle_text_command,
            10
        )
        
        # AI conversation context
        self.conversation_history = []
        self.max_history = 10
        
        self.logger.info("🚀 Integrated AI Interface ready!")
    
    def handle_text_command(self, msg: String) -> None:
        """
        Handle incoming text commands and generate AI responses
        
        Args:
            msg: ROS2 String message containing the text command
        """
        self.logger.info(f"🎤 CALLBACK TRIGGERED! Received message: {msg}")
        self.logger.info(f"🎤 Message data: '{msg.data}'")
        
        try:
            command_text = msg.data.strip()
            if not command_text:
                self.logger.warning("⚠️ Empty command text received")
                return
            
            self.logger.info(f"🎤 Processing speech command: '{command_text}'")
            
            # Generate AI response
            self.logger.info("🤖 Generating AI response...")
            response_text = self.generate_ai_response(command_text)
            self.logger.info(f"🤖 Generated response: '{response_text}'")
            
            # Publish the response
            self.logger.info("📤 Publishing response...")
            response_msg = String()
            response_msg.data = response_text
            self.text_response_pub.publish(response_msg)
            self.logger.info(f"📤 Response published: '{response_text}'")
            
            # Convert response to speech using the fixed audio hardware interface
            if self.audio_hw:
                self.logger.info("🎵 Converting to speech...")
                self.audio_hw.speak_text(response_text)
                self.logger.info("🎵 Speech conversion completed")
            else:
                self.logger.warning("⚠️ No audio hardware available for TTS")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling speech command: {e}")
            import traceback
            self.logger.error(f"❌ Traceback: {traceback.format_exc()}")
            # Send error response
            error_msg = String()
            error_msg.data = "Sorry, I encountered an error processing your request."
            self.text_response_pub.publish(error_msg)
    
    def generate_ai_response(self, user_input: str) -> str:
        """
        Generate an AI response using OpenAI GPT or fallback
        
        Args:
            user_input: The user's input text
            
        Returns:
            AI-generated response text
        """
        try:
            if self.openai_client:
                return self._generate_openai_response(user_input)
            else:
                return self._generate_fallback_response(user_input)
        except Exception as e:
            self.logger.error(f"❌ Error generating AI response: {e}")
            return "I'm sorry, I'm having trouble processing your request right now."
    
    def _generate_openai_response(self, user_input: str) -> str:
        """
        Generate response using OpenAI GPT
        
        Args:
            user_input: The user's input text
            
        Returns:
            OpenAI-generated response
        """
        try:
            # Add user input to conversation history
            self.conversation_history.append({"role": "user", "content": user_input})
            
            # Limit conversation history
            if len(self.conversation_history) > self.max_history * 2:
                self.conversation_history = self.conversation_history[-self.max_history:]
            
            # Create system message for Nevil
            system_message = {
                "role": "system",
                "content": "You are Nevil, a helpful AI assistant for a Raspberry Pi robot car. "
                          "Keep responses concise and friendly. You can help with navigation, "
                          "information, and general conversation."
            }
            
            # Prepare messages for OpenAI
            messages = [system_message] + self.conversation_history
            
            # Call OpenAI API
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=messages,
                max_tokens=150,
                temperature=0.7
            )
            
            ai_response = response.choices[0].message.content.strip()
            
            # Add AI response to conversation history
            self.conversation_history.append({"role": "assistant", "content": ai_response})
            
            return ai_response
            
        except Exception as e:
            self.logger.error(f"❌ OpenAI API error: {e}")
            return self._generate_fallback_response(user_input)
    
    def _generate_fallback_response(self, user_input: str) -> str:
        """
        Generate fallback response when OpenAI is not available
        
        Args:
            user_input: The user's input text
            
        Returns:
            Simple rule-based response
        """
        user_lower = user_input.lower()
        
        # Simple rule-based responses
        if any(greeting in user_lower for greeting in ['hello', 'hi', 'hey']):
            return "Hello! I'm Nevil, your robot assistant. How can I help you today?"
        
        elif any(question in user_lower for question in ['how are you', 'how do you feel']):
            return "I'm doing great! My systems are running smoothly and I'm ready to help."
        
        elif any(name in user_lower for name in ['your name', 'who are you']):
            return "I'm Nevil, a Raspberry Pi robot car with AI capabilities. Nice to meet you!"
        
        elif any(help_word in user_lower for help_word in ['help', 'assist', 'support']):
            return "I can help you with navigation, answer questions, and have conversations. What would you like to do?"
        
        elif any(move in user_lower for move in ['move', 'drive', 'go', 'navigate']):
            return "I'm ready to move! Please specify where you'd like me to go or what direction to take."
        
        elif any(thanks in user_lower for thanks in ['thank', 'thanks']):
            return "You're welcome! I'm always happy to help."
        
        elif any(bye in user_lower for bye in ['bye', 'goodbye', 'see you']):
            return "Goodbye! It was nice talking with you. See you next time!"
        
        else:
            return f"I heard you say '{user_input}'. I'm still learning, but I'm here to help however I can!"
    
    def cleanup(self):
        """Clean up resources"""
        try:
            if self.audio_hw:
                self.audio_hw.cleanup()
            self.logger.info("🧹 Integrated AI Interface cleaned up")
        except Exception as e:
            self.logger.error(f"❌ Error during cleanup: {e}")

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        ai_interface = IntegratedAIInterface()
        
        print("🤖 Integrated AI Interface running...")
        print("📡 Listening for speech commands on /nevil/speech_command")
        print("📢 Publishing responses on /nevil/text_response")
        print("🎵 Using fixed AudioHardwareInterface for TTS")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(ai_interface)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Integrated AI Interface...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ai_interface' in locals():
            ai_interface.cleanup()
            ai_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()