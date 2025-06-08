# Nevil Interfaces AI

This package provides text and voice interfaces for Nevil-picar v2.0 that enable natural language interaction capabilities for controlling and communicating with the robot.

## Overview

The `nevil_interfaces_ai` package implements:

- Text-based command processing
- Speech recognition and synthesis
- Dialog management for maintaining conversation context
- Integration with the navigation API

It works with both the physical robot and the simulation environment.

## Components

### ROS2 Interfaces

- **Messages**:
  - `TextCommand`: Text commands from various sources
  - `TextResponse`: Text responses to commands
  - `VoiceCommand`: Voice commands from speech recognition
  - `VoiceResponse`: Voice responses for speech synthesis
  - `DialogState`: Current state of the dialog system

- **Services**:
  - `QueryCapabilities`: Query the robot's capabilities
  - `TranslateCommand`: Translate natural language to structured commands

- **Actions**:
  - `ProcessDialog`: Process a multi-turn dialog

### Nodes

- **Text Command Processor**: Interprets natural language commands and converts them to robot actions
- **Speech Recognition**: Converts spoken commands to text
- **Speech Synthesis**: Converts responses to speech
- **Dialog Manager**: Maintains conversation context and manages multi-turn dialogs

### Hardware Interfaces

- **Audio Hardware Interface**: Provides thread-safe access to microphone and speaker hardware
  - Supports speech recognition with OpenAI Whisper integration
  - Supports speech synthesis with robot_hat TTS integration
  - Handles audio recording and playback with proper mutex handling
  - Includes simulation mode for development without hardware
  - Provides fallback mechanisms when hardware or libraries are not available
  - Thread-safe design for real-time performance
  - For detailed documentation, see [AudioHardwareInterface Documentation](nevil_interfaces_ai/README_audio_hardware_interface.md)

## Usage

### Launch Files

- **nevil_interfaces_ai.launch.py**: Launch the AI interfaces
  ```
  ros2 launch nevil_interfaces_ai nevil_interfaces_ai.launch.py
  ```

- **nevil_interfaces_ai_with_simulation.launch.py**: Launch the AI interfaces with the simulation
  ```
  ros2 launch nevil_interfaces_ai nevil_interfaces_ai_with_simulation.launch.py
  ```

### Launch Parameters

- `use_sim`: Whether to use simulation (true) or physical hardware (false)
- `use_online_recognition`: Whether to use online speech recognition (true) or offline (false)
- `use_online_tts`: Whether to use online text-to-speech (true) or offline (false)
- `use_cloud_ai`: Whether to use cloud AI for natural language processing (true) or local (false)
- `environment`: Simulation environment to use (empty, maze, obstacle_course)

### Environment Variables

The package can also be configured using environment variables in a `.env` file at the root of the project. This is especially useful for storing sensitive information like API keys.

Example `.env` file:

```
# OpenAI API Keys (for language processing, not needed for Whisper)
OPENAI_API_KEY=your_openai_api_key_here

# Speech Recognition Settings
SPEECH_RECOGNITION_LANGUAGE=en
SPEECH_RECOGNITION_ENERGY_THRESHOLD=300
SPEECH_RECOGNITION_PAUSE_THRESHOLD=0.5
SPEECH_RECOGNITION_DYNAMIC_ENERGY=true

# Speech Synthesis Settings
SPEECH_SYNTHESIS_VOICE=onyx
SPEECH_SYNTHESIS_RATE=200
SPEECH_SYNTHESIS_VOLUME=1.0

# Whisper Settings (offline speech-to-text, no API key needed)
WHISPER_MODEL=small  # Options: tiny, base, small, medium, large
```

For a complete list of available configuration options, see the [AudioHardwareInterface Documentation](nevil_interfaces_ai/README_audio_hardware_interface.md).

For detailed information on building and using environment variables, see the [Build Environment Variables Documentation](../../docs/BUILD_ENVIRONMENT_VARIABLES.md).

## Examples

### Basic Command and Control via Text

```python
# Publish a text command
from nevil_interfaces_ai.msg import TextCommand
import rclpy
from rclpy.node import Node

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(TextCommand, '/nevil/text_command', 10)
        self.send_command("Move forward 1 meter")
        
    def send_command(self, command_text):
        msg = TextCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_text = command_text
        msg.source = 'api'
        msg.command_type = 'navigation'
        msg.priority = 100
        msg.command_id = 'cmd_001'
        msg.context_id = 'ctx_001'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command_text}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Voice Interaction

Voice interaction is handled automatically by the speech recognition and synthesis nodes. Simply launch the system and speak to the robot.

### Multi-turn Conversations

```python
# Start a dialog
from nevil_interfaces_ai.action import ProcessDialog
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class DialogClient(Node):
    def __init__(self):
        super().__init__('dialog_client')
        self._action_client = ActionClient(self, ProcessDialog, '/nevil/process_dialog')
        
    def send_goal(self):
        goal_msg = ProcessDialog.Goal()
        goal_msg.initial_utterance = "Hello, what can you do?"
        goal_msg.dialog_mode = "conversation"
        goal_msg.timeout = 60.0
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Dialog completed: {result.success}')
        self.get_logger().info(f'Summary: {result.dialog_summary}')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current state: {feedback.current_state}')
        self.get_logger().info(f'Last utterance: {feedback.last_utterance}')

def main(args=None):
    rclpy.init(args=args)
    action_client = DialogClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Dependencies

- ROS2 Humble
- Python 3.8+
- SpeechRecognition
- PyAudio
- robot_hat (for hardware access)
- pyttsx3 (fallback for speech synthesis)
- OpenAI Whisper (optional, for improved offline speech recognition - no API key needed)
- OpenAI API (optional, for natural language processing - requires API key)
- python-dotenv (for loading environment variables from .env file)

## License

Apache License 2.0