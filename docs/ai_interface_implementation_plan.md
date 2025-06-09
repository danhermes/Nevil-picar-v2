# AI Interface Implementation Plan

## Overview

This document outlines the implementation plan for the AI interface system in the Nevil robot. The AI interface serves as the central component for natural language processing, connecting speech recognition (STT) with speech synthesis (TTS) and enabling conversational interaction with the robot.

## Current Status

The AI interface node has been implemented with the following features:
- Subscription to `/nevil/text_command` topic to receive text from STT
- Processing of text using OpenAI API with fallback mechanism
- Publication of responses to `/nevil/text_response` topic for TTS
- Implementation of ProcessDialog action server for complex dialog interactions

However, there is an issue with the entry point configuration in setup.py that needs to be addressed.

## Required Dependencies

1. **ROS2 Dependencies**
   - rclpy
   - std_msgs
   - action_msgs
   - nevil_interfaces (for custom message and action definitions)

2. **External Dependencies**
   - python-dotenv (for environment variable management)
   - openai (for API integration)

## Implementation Details

### 1. AI Interface Node Class Structure

```python
class AIInterfaceNode(Node):
    def __init__(self):
        super().__init__('ai_interface')
        
        # Publishers
        self.text_response_pub = self.create_publisher(String, '/nevil/text_response', 10)
        self.status_pub = self.create_publisher(String, 'ai_status', 10)
        
        # Subscribers
        self.text_command_sub = self.create_subscription(
            String,
            '/nevil/text_command',
            self.handle_text_command,
            10
        )
        
        # Action server
        self._action_server = ActionServer(
            self,
            ProcessDialog,
            'process_dialog',
            self.execute_dialog_callback
        )
        
        # OpenAI API configuration
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
```

### 2. Text Command Handler

```python
def handle_text_command(self, msg):
    """Handle incoming text commands from speech recognition"""
    self.get_logger().info(f'Received text command: {msg.data}')
    
    # Process the command with OpenAI or use a fallback
    response = self.process_with_openai(msg.data)
    
    # Publish the response
    response_msg = String()
    response_msg.data = response
    self.text_response_pub.publish(response_msg)
    
    # Also publish status update
    status_msg = String()
    status_msg.data = 'AI system processed command'
    self.status_pub.publish(status_msg)
```

### 3. Dialog Processing Action Server

```python
def execute_dialog_callback(self, goal_handle):
    """Handle dialog processing action requests"""
    goal = goal_handle.request
    
    # Initialize feedback
    feedback_msg = ProcessDialog.Feedback()
    feedback_msg.current_state = "processing"
    feedback_msg.last_utterance = goal.initial_utterance
    
    # Process the dialog with OpenAI
    response = self.process_with_openai(goal.initial_utterance)
    
    # Update feedback
    feedback_msg.current_state = "completed"
    feedback_msg.last_utterance = response
    goal_handle.publish_feedback(feedback_msg)
    
    # Initialize result
    result = ProcessDialog.Result()
    result.success = True
    result.message = "Dialog processed successfully"
    result.final_state = "completed"
    
    # Publish the response for TTS
    response_msg = String()
    response_msg.data = response
    self.text_response_pub.publish(response_msg)
    
    goal_handle.succeed(result)
    return result
```

### 4. OpenAI Integration

```python
def process_with_openai(self, text):
    """Process text with OpenAI API and return response"""
    try:
        if not self.openai_api_key:
            return self.generate_fallback_response(text)
        
        # Import here to avoid dependency issues if OpenAI is not installed
        import openai
        openai.api_key = self.openai_api_key
        
        # Call OpenAI API
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant for a robot named Nevil."},
                {"role": "user", "content": text}
            ]
        )
        
        # Extract and return the response text
        return response.choices[0].message.content
    
    except Exception as e:
        self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
        return self.generate_fallback_response(text)
```

### 5. Fallback Response Generator

```python
def generate_fallback_response(self, text):
    """Generate a fallback response when OpenAI is unavailable"""
    # Simple keyword-based responses
    text_lower = text.lower()
    
    if 'hello' in text_lower or 'hi' in text_lower:
        return "Hello! I'm Nevil. How can I help you?"
    
    if 'name' in text_lower:
        return "My name is Nevil. I'm a robot assistant."
    
    # Default response
    return "I'm not sure how to respond to that. Could you rephrase your request?"
```

## Implementation Steps

1. **Fix Entry Point Configuration**

   The current entry point in setup.py is:
   ```python
   'ai_interface_node = nevil_interfaces_ai.scripts.ai_interface_node:main',
   ```

   This should be modified to:
   ```python
   'ai_interface_node = scripts.ai_interface_node:main',
   ```

   Or alternatively, move the script to the Python package structure:
   ```
   nevil_interfaces_ai/
   └── nevil_interfaces_ai/
       └── ai_interface_node.py
   ```

2. **Ensure Script Permissions**

   Make sure the script is executable:
   ```bash
   chmod +x src/nevil_interfaces_ai/scripts/ai_interface_node.py
   ```

3. **Update Package Dependencies**

   Ensure all required dependencies are listed in package.xml and setup.py.

4. **Build and Install**

   Build and install the package:
   ```bash
   colcon build --packages-select nevil_interfaces_ai
   source install/setup.bash
   ```

5. **Test the Node**

   Test the node individually:
   ```bash
   ros2 run nevil_interfaces_ai ai_interface_node
   ```

6. **Test with Launch File**

   Test the node with the launch file:
   ```bash
   ros2 launch nevil_interfaces_ai nevil_interfaces_ai.launch.py
   ```

7. **Test Full Integration**

   Test the full integration with the physical robot:
   ```bash
   ros2 launch nevil_bringup physical_robot.launch.py
   ```

## Error Handling and Robustness

1. **API Key Handling**
   - Check for API key availability at startup
   - Provide clear error messages when API key is missing
   - Fall back to basic responses when API is unavailable

2. **Exception Handling**
   - Catch and log exceptions during API calls
   - Provide fallback responses when errors occur
   - Ensure the node continues to function even when components fail

3. **Message Validation**
   - Validate incoming messages before processing
   - Handle empty or malformed messages gracefully

## Future Improvements

1. **Context Management**
   - Maintain conversation history for more coherent interactions
   - Implement session management for multi-turn conversations

2. **Intent Recognition**
   - Add intent classification to better understand user commands
   - Route commands to appropriate subsystems based on intent

3. **Local Processing**
   - Implement local processing options for basic commands
   - Reduce dependency on external API for common interactions

4. **Command Routing**
   - Develop a system to route commands to appropriate robot subsystems
   - Implement feedback mechanisms for command execution status

5. **Personalization**
   - Add user recognition and personalized responses
   - Implement preference learning for better interactions over time