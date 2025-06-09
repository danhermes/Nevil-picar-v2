# Nevil-picar v2.0: User Guide

This guide provides instructions for using the Nevil-picar v2.0 system, including basic operations, voice commands, text commands, and system modes.

## Table of Contents

- [Getting Started](#getting-started)
- [Basic Operations](#basic-operations)
- [Voice Interface](#voice-interface)
- [Text Interface](#text-interface)
- [System Modes](#system-modes)
- [Navigation](#navigation)
- [Environmental Learning](#environmental-learning)
- [Customization](#customization)
- [Maintenance](#maintenance)
- [Troubleshooting](#troubleshooting)

## Getting Started

### Initial Setup

Before using Nevil-picar v2.0, ensure that you have completed the installation and setup process as described in the [Installation and Setup Guide](2_installation_setup.md).

### Powering On

1. Ensure the battery is charged and properly connected
2. Press the power button on the Raspberry Pi
3. Wait for the system to boot (approximately 30-60 seconds)
4. The system will announce "Nevil is ready" when startup is complete

### Checking System Status

You can check the system status using the following command:

```bash
ros2 topic echo /system_status
```

This will display the current status of the system, including:

- System mode
- Battery level
- Connection status
- Error messages (if any)

## Basic Operations

### Movement Controls

Nevil-picar v2.0 can be controlled using voice commands, text commands, or programmatically through ROS2 topics.

#### Manual Control

To manually control the robot using the keyboard, run:

```bash
ros2 run nevil_core teleop_keyboard
```

This will start a keyboard teleop node with the following controls:

- `w`: Move forward
- `s`: Move backward
- `a`: Turn left
- `d`: Turn right
- `x`: Stop
- `q`: Increase speed
- `z`: Decrease speed
- `e`: Toggle turbo mode

#### Programmatic Control

To control the robot programmatically, publish messages to the `/cmd_vel` topic:

```bash
# Move forward at 0.5 m/s
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left at 1.0 rad/s
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Stop
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Stopping in an Emergency

In case of emergency, you can stop the robot using any of the following methods:

1. Voice command: "Nevil, stop immediately"
2. Text command: "stop"
3. Keyboard: Press `x` in teleop mode
4. ROS2 command:
   ```bash
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```
5. Physical power button (press and hold for 3 seconds)

## Voice Interface

### Activating Voice Recognition

Nevil-picar v2.0 uses voice activity detection to automatically detect when you are speaking to it. To activate voice recognition:

1. Say "Hey Nevil" or "Nevil" to get the robot's attention
2. Wait for the acknowledgment sound or light
3. Speak your command clearly

### Basic Voice Commands

| Command | Description | Example |
|---------|-------------|---------|
| Move | Control movement | "Move forward", "Go backward", "Turn left" |
| Stop | Stop movement | "Stop", "Halt", "Freeze" |
| Speed | Control speed | "Speed up", "Slow down", "Set speed to 50%" |
| Mode | Change system mode | "Enter conversation mode", "Switch to play mode" |
| Information | Get information | "What's your status?", "Where are you?", "What can you see?" |
| Behaviors | Perform behaviors | "Wave hello", "Nod", "Shake head" |

### Advanced Voice Commands

| Command | Description | Example |
|---------|-------------|---------|
| Navigation | Navigate to locations | "Go to the kitchen", "Navigate to the charging station" |
| Learning | Learn environment | "Remember this location as kitchen", "Learn this room" |
| Interaction | Conversational interaction | "Tell me a joke", "What's the weather like?", "Who are you?" |
| System | System control | "Restart navigation system", "Enable offline mode", "Update your map" |

### Voice Command Examples

Here are some examples of voice commands and their expected responses:

- **User**: "Hey Nevil, move forward for 2 meters"
  - **Nevil**: "Moving forward for 2 meters" (robot moves forward)

- **User**: "Nevil, what can you see?"
  - **Nevil**: "I can see a chair, a table, and a person in front of me"

- **User**: "Nevil, remember this location as living room"
  - **Nevil**: "I've remembered this location as living room"

- **User**: "Hey Nevil, tell me a joke"
  - **Nevil**: "Why don't scientists trust atoms? Because they make up everything!"

## Text Interface

### Accessing the Text Interface

You can interact with Nevil-picar v2.0 using the text interface through the ROS2 command line:

```bash
ros2 run nevil_interfaces_ai text_command_client
```

This will start an interactive text command client where you can type commands and receive responses.

### Text Command Format

Text commands follow a simple format:

```
[command] [parameters]
```

For example:
- `move forward 1.0` - Move forward 1 meter
- `turn left 90` - Turn left 90 degrees
- `mode conversation` - Switch to conversation mode

### Basic Text Commands

| Command | Parameters | Description | Example |
|---------|------------|-------------|---------|
| move | direction, distance | Move in a direction | `move forward 1.0` |
| turn | direction, angle | Turn in a direction | `turn left 90` |
| stop | none | Stop movement | `stop` |
| speed | value | Set movement speed | `speed 0.5` |
| mode | mode_name | Change system mode | `mode conversation` |
| status | none | Get system status | `status` |

### Advanced Text Commands

| Command | Parameters | Description | Example |
|---------|------------|-------------|---------|
| navigate | location | Navigate to a location | `navigate kitchen` |
| remember | name | Remember current location | `remember kitchen` |
| learn | none | Learn current environment | `learn` |
| find | object | Find an object | `find chair` |
| say | text | Speak text | `say Hello, how are you?` |
| execute | behavior | Execute a behavior | `execute wave` |

### Text Command Examples

Here are some examples of text commands and their expected responses:

```
> move forward 1.0
Moving forward 1.0 meters

> turn left 90
Turning left 90 degrees

> status
System Status:
- Mode: conversation
- Battery: 78%
- Position: x=1.2, y=3.4, theta=0.5
- Connected to network: Yes
- AI Mode: Online (OpenAI)

> navigate kitchen
Navigating to kitchen...
```

## System Modes

Nevil-picar v2.0 operates in several modes, each with specific behaviors and capabilities.

### Available Modes

| Mode | Description | Activation Command |
|------|-------------|-------------------|
| Conversation | Focus on interaction with users | `mode conversation` |
| Play | Autonomous exploration and interaction | `mode play` |
| Autonomous | Independent operation and mapping | `mode autonomous` |
| Sleep | Power conservation with minimal awareness | `mode sleep` |

### Conversation Mode

In conversation mode, Nevil focuses on interaction with users:

- Active listening for voice commands
- Natural language conversation
- Reduced movement to minimize noise
- Focus on user engagement

To activate conversation mode:
- Voice: "Nevil, enter conversation mode"
- Text: `mode conversation`

### Play Mode

In play mode, Nevil explores and interacts with its environment:

- Autonomous exploration
- Object detection and tracking
- Environmental learning
- Playful behaviors and responses

To activate play mode:
- Voice: "Nevil, switch to play mode"
- Text: `mode play`

### Autonomous Mode

In autonomous mode, Nevil operates independently:

- SLAM-based navigation
- Environmental mapping
- Object recognition and categorization
- Self-directed behavior

To activate autonomous mode:
- Voice: "Nevil, enter autonomous mode"
- Text: `mode autonomous`

### Sleep Mode

In sleep mode, Nevil conserves power while maintaining awareness:

- Reduced sensor processing
- Minimal movement
- Low-power listening for wake commands
- Periodic environment checks

To activate sleep mode:
- Voice: "Nevil, go to sleep"
- Text: `mode sleep`

To wake from sleep mode:
- Voice: "Nevil, wake up"
- Physical: Tap the top of the robot twice

## Navigation

### Basic Navigation

Nevil-picar v2.0 can navigate to specific locations using either coordinates or named locations.

#### Navigating to Coordinates

To navigate to specific coordinates:

- Voice: "Nevil, navigate to coordinates X=1.5, Y=2.3"
- Text: `navigate coordinates 1.5 2.3`

#### Navigating to Named Locations

To navigate to a named location (that has been previously learned):

- Voice: "Nevil, go to the kitchen"
- Text: `navigate kitchen`

### Learning Locations

Nevil can learn and remember locations:

1. Position the robot at the desired location
2. Use one of the following commands:
   - Voice: "Nevil, remember this location as kitchen"
   - Text: `remember kitchen`
3. The robot will confirm that it has remembered the location

### Viewing the Map

To view the current map:

```bash
ros2 run nevil_navigation map_viewer
```

This will open a visualization of the current map with:
- Walls and obstacles
- Named locations
- Current robot position
- Planned paths (if navigating)

### Path Planning

Nevil uses advanced path planning algorithms to navigate around obstacles:

1. When given a navigation command, Nevil will:
   - Determine its current position
   - Plan a path to the destination
   - Execute the path while avoiding obstacles
   - Announce arrival at the destination

2. If an obstacle blocks the planned path, Nevil will:
   - Stop and reassess the situation
   - Plan an alternative path if possible
   - Notify the user if navigation is not possible

## Environmental Learning

### Room Recognition

Nevil can learn and recognize different rooms:

1. Position the robot in a room
2. Use the command:
   - Voice: "Nevil, learn this room"
   - Text: `learn room`
3. The robot will scan the room and create a signature
4. Later, the robot can recognize the room:
   - Voice: "Nevil, what room is this?"
   - Text: `identify room`

### Object Recognition

Nevil can recognize and track objects:

1. To identify objects in view:
   - Voice: "Nevil, what can you see?"
   - Text: `identify objects`
2. The robot will list the objects it recognizes

3. To find a specific object:
   - Voice: "Nevil, find the chair"
   - Text: `find chair`
4. The robot will search for the object and navigate to it if found

### User Recognition

Nevil can recognize different users:

1. To register a new user:
   - Voice: "Nevil, remember me as [name]"
   - Text: `register user [name]`
2. The robot will capture facial and voice characteristics

3. Later, the robot can recognize the user:
   - Voice: "Nevil, who am I?"
   - Text: `identify user`

## Customization

### Changing Voice Settings

You can customize Nevil's voice:

1. Edit the voice configuration file:
   ```bash
   nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_interfaces_ai/config/voice_config.yaml
   ```

2. Adjust the parameters:
   ```yaml
   voice:
     engine: "openai"  # Options: "openai", "pyttsx3"
     voice_id: "alloy"  # For OpenAI: "alloy", "echo", "fable", "onyx", "nova", "shimmer"
     rate: 175  # Speech rate (words per minute)
     volume: 1.0  # Volume (0.0 to 1.0)
   ```

3. Restart the voice control node:
   ```bash
   ros2 service call /system_manager/restart_node std_srvs/srv/Trigger "{node_name: 'voice_control'}"
   ```

### Customizing Behaviors

You can customize Nevil's behaviors:

1. Edit the behavior configuration file:
   ```bash
   nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_core/config/behaviors.yaml
   ```

2. Modify existing behaviors or add new ones:
   ```yaml
   behaviors:
     wave:
       sequence:
         - {action: "servo", position: 45, duration: 0.5}
         - {action: "servo", position: -45, duration: 0.5}
         - {action: "servo", position: 0, duration: 0.5}
       sound: "hello.wav"
     
     dance:
       sequence:
         - {action: "move", linear: 0.2, angular: 0.0, duration: 1.0}
         - {action: "move", linear: 0.0, angular: 3.14, duration: 2.0}
         - {action: "move", linear: 0.2, angular: 0.0, duration: 1.0}
       sound: "dance.wav"
   ```

3. Restart the system manager node:
   ```bash
   ros2 service call /system_manager/restart std_srvs/srv/Trigger "{}"
   ```

### Configuring AI Settings

You can configure the AI settings:

1. Edit the AI configuration file:
   ```bash
   nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_interfaces_ai/config/ai_config.yaml
   ```

2. Adjust the parameters:
   ```yaml
   ai:
     online:
       provider: "openai"
       model: "gpt-4"
       temperature: 0.7
       max_tokens: 150
     
     offline:
       provider: "gemma"  # Options: "gemma", "tinyllama"
       model_path: "/path/to/model"
       temperature: 0.8
       max_tokens: 100
     
     switching:
       prefer_online: true
       offline_threshold_ms: 500
       network_check_interval: 60
   ```

3. Restart the AI processing node:
   ```bash
   ros2 service call /system_manager/restart_node std_srvs/srv/Trigger "{node_name: 'ai_processing'}"
   ```

## Maintenance

### Battery Management

Nevil-picar v2.0 includes battery management features:

1. Checking battery level:
   - Voice: "Nevil, what's your battery level?"
   - Text: `battery status`
   - ROS2: `ros2 topic echo /battery_status`

2. Low battery behavior:
   - At 20% battery, Nevil will warn you
   - At 10% battery, Nevil will automatically return to the charging station if known
   - At 5% battery, Nevil will enter emergency power saving mode

3. Charging:
   - Place Nevil on its charging station
   - The robot will announce when charging begins and completes
   - You can check charging status:
     ```bash
     ros2 topic echo /charging_status
     ```

### Software Updates

To update the Nevil-picar v2.0 software:

1. Pull the latest changes:
   ```bash
   cd ~/nevil_ws/src/nevil-picar-v2
   git pull
   ```

2. Rebuild the workspace:
   ```bash
   cd ~/nevil_ws
   colcon build
   source install/setup.bash
   ```

3. Restart the system:
   ```bash
   ros2 service call /system_manager/restart std_srvs/srv/Trigger "{}"
   ```

### Sensor Calibration

Periodically calibrate the sensors for optimal performance:

1. Ultrasonic sensor calibration:
   ```bash
   ros2 run nevil_perception calibrate_ultrasonic
   ```
   Follow the on-screen instructions

2. Camera calibration:
   ```bash
   ros2 run nevil_perception calibrate_camera
   ```
   Follow the on-screen instructions

3. IMU calibration (if installed):
   ```bash
   ros2 run nevil_perception calibrate_imu
   ```
   Follow the on-screen instructions

### Log Management

Nevil-picar v2.0 maintains logs for debugging and analysis:

1. View real-time logs:
   ```bash
   ros2 run nevil_core log_viewer
   ```

2. Save logs to a file:
   ```bash
   ros2 run nevil_core log_saver --output ~/nevil_logs_$(date +%Y%m%d).txt
   ```

3. Clear old logs:
   ```bash
   ros2 run nevil_core log_cleaner --older-than 30
   ```
   This removes logs older than 30 days

## Troubleshooting

### Common Issues

#### Robot Doesn't Respond to Voice Commands

1. Check if the microphone is properly connected
2. Verify that the voice control node is running:
   ```bash
   ros2 node list | grep voice_control
   ```
3. Test the microphone:
   ```bash
   ros2 run nevil_interfaces_ai test_microphone
   ```
4. Restart the voice control node:
   ```bash
   ros2 service call /system_manager/restart_node std_srvs/srv/Trigger "{node_name: 'voice_control'}"
   ```

#### Robot Doesn't Move

1. Check if the motors are properly connected
2. Verify that the motion control node is running:
   ```bash
   ros2 node list | grep motion_control
   ```
3. Test the motors:
   ```bash
   ros2 run nevil_navigation test_motors
   ```
4. Check for obstacles that might be blocking movement
5. Restart the motion control node:
   ```bash
   ros2 service call /system_manager/restart_node std_srvs/srv/Trigger "{node_name: 'motion_control'}"
   ```

#### Camera Not Working

1. Check if the camera is properly connected
2. Verify that the camera vision node is running:
   ```bash
   ros2 node list | grep camera_vision
   ```
3. Test the camera:
   ```bash
   ros2 run nevil_perception test_camera
   ```
4. Restart the camera vision node:
   ```bash
   ros2 service call /system_manager/restart_node std_srvs/srv/Trigger "{node_name: 'camera_vision'}"
   ```

#### System Crashes or Freezes

1. Check the system logs:
   ```bash
   ros2 run nevil_core log_viewer --level error
   ```
2. Restart the system manager:
   ```bash
   ros2 service call /system_manager/restart std_srvs/srv/Trigger "{}"
   ```
3. If the issue persists, reboot the Raspberry Pi:
   ```bash
   sudo reboot
   ```

### Diagnostic Tools

Nevil-picar v2.0 includes several diagnostic tools:

1. System diagnostics:
   ```bash
   ros2 run nevil_core system_diagnostics
   ```
   This checks all nodes and reports their status

2. Hardware diagnostics:
   ```bash
   ros2 run nevil_core hardware_diagnostics
   ```
   This tests all hardware components

3. Network diagnostics:
   ```bash
   ros2 run nevil_core network_diagnostics
   ```
   This checks network connectivity and API access

4. Performance monitoring:
   ```bash
   ros2 run nevil_core performance_monitor
   ```
   This displays real-time performance metrics

### Getting Help

If you encounter issues that you cannot resolve:

1. Check the [Troubleshooting Guide](7_troubleshooting.md) for more detailed information
2. Visit the project repository for known issues and solutions
3. Contact the development team for support

## Conclusion

This user guide provides the essential information for operating Nevil-picar v2.0. For more detailed information on specific topics, refer to the other documentation files:

- [Project Overview](1_overview_project.md)
- [Installation and Setup](2_installation_setup.md)
- [Core Concepts](3_core_concepts.md)
- [API Reference](5_api_reference.md)
- [Developer Guide](6_developer_guide.md)
- [Troubleshooting](7_troubleshooting.md)