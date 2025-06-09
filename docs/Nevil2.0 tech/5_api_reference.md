# Nevil-picar v2.0: API Reference

This document provides detailed documentation of the APIs available in the Nevil-picar v2.0 system. It covers ROS2 interfaces, Python APIs, and configuration options.

## Table of Contents

- [ROS2 Interfaces](#ros2-interfaces)
  - [Topics](#topics)
  - [Services](#services)
  - [Actions](#actions)
  - [Parameters](#parameters)
- [Python APIs](#python-apis)
  - [Navigation API](#navigation-api)
  - [Vision API](#vision-api)
  - [Voice API](#voice-api)
  - [AI API](#ai-api)
  - [System Management API](#system-management-api)
- [Configuration Files](#configuration-files)
  - [Hardware Configuration](#hardware-configuration)
  - [ROS2 Parameters](#ros2-parameters)
  - [AI Configuration](#ai-configuration)
  - [Simulation Configuration](#simulation-configuration)
- [Command Line Tools](#command-line-tools)
  - [System Tools](#system-tools)
  - [Diagnostic Tools](#diagnostic-tools)
  - [Development Tools](#development-tools)

## ROS2 Interfaces

### Topics

#### Motion Control Topics

| Topic | Type | Description | Publishers | Subscribers |
|-------|------|-------------|------------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for robot movement | Navigation, Obstacle Avoidance | Motion Control |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry data for robot position tracking | Motion Control | Navigation |
| `/motor_status` | `nevil_interfaces/msg/MotorStatus` | Status of motor controllers | Motion Control | System Manager |

Example usage:

```python
# Publishing to /cmd_vel
from geometry_msgs.msg import Twist
from rclpy.node import Node

class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
```

#### Sensor Topics

| Topic | Type | Description | Publishers | Subscribers |
|-------|------|-------------|------------|-------------|
| `/ultrasonic_data` | `sensor_msgs/msg/Range` | Distance sensor readings | Sensor Driver | Obstacle Avoidance |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Raw camera images | Camera Driver | Camera Vision |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU sensor data | IMU Driver | Navigation |

Example usage:

```python
# Subscribing to /ultrasonic_data
from sensor_msgs.msg import Range
from rclpy.node import Node

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            'ultrasonic_data',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Distance: {msg.range} meters')
```

#### Vision Topics

| Topic | Type | Description | Publishers | Subscribers |
|-------|------|-------------|------------|-------------|
| `/object_detections` | `vision_msgs/msg/Detection2DArray` | Detected objects | Camera Vision | Navigation, AI Processing |
| `/face_detections` | `vision_msgs/msg/Detection2DArray` | Detected faces | Camera Vision | AI Processing |
| `/aruco_markers` | `vision_msgs/msg/Detection2DArray` | Detected ArUco markers | Camera Vision | Navigation |

#### System Topics

| Topic | Type | Description | Publishers | Subscribers |
|-------|------|-------------|------------|-------------|
| `/system_status` | `nevil_interfaces/msg/SystemStatus` | Overall system status | System Manager | All Nodes |
### Services

#### Navigation Services

| Service | Type | Description | Server | Clients |
|---------|------|-------------|--------|---------|
| `/check_obstacle` | `nevil_interfaces/srv/CheckObstacle` | Check if an obstacle is present | Obstacle Avoidance | Any Node |
| `/set_navigation_mode` | `nevil_interfaces/srv/SetNavigationMode` | Set navigation mode (normal, cautious, etc.) | Navigation | System Manager, AI Processing |
| `/clear_map` | `std_srvs/srv/Trigger` | Clear the current map | Navigation | System Manager |

Example service definition (`CheckObstacle.srv`):

```
float32 direction  # Direction to check in radians
---
bool obstacle_present
float32 distance
```

Example usage:

```python
# Service client for /check_obstacle
from nevil_interfaces.srv import CheckObstacle
import rclpy
from rclpy.node import Node

class ObstacleCheckClient(Node):
    def __init__(self):
        super().__init__('obstacle_check_client')
        self.client = self.create_client(CheckObstacle, 'check_obstacle')
        
    async def check_obstacle(self, direction):
        request = CheckObstacle.Request()
        request.direction = float(direction)
        
        future = self.client.call_async(request)
        await future
        
        return future.result()
```

#### Vision Services

| Service | Type | Description | Server | Clients |
|---------|------|-------------|--------|---------|
| `/detect_objects` | `nevil_interfaces/srv/DetectObjects` | Detect objects in the current camera view | Camera Vision | Any Node |
| `/recognize_face` | `nevil_interfaces/srv/RecognizeFace` | Recognize a face in the current camera view | Camera Vision | AI Processing |
| `/capture_image` | `nevil_interfaces/srv/CaptureImage` | Capture a single image | Camera Vision | Any Node |

#### AI Services

| Service | Type | Description | Server | Clients |
|---------|------|-------------|--------|---------|
| `/translate_command` | `nevil_interfaces_ai/srv/TranslateCommand` | Convert natural language to system commands | AI Processing | Voice Control |
| `/query_capabilities` | `nevil_interfaces_ai/srv/QueryCapabilities` | Get information about system capabilities | AI Processing | Any Node |
| `/switch_ai_mode` | `nevil_interfaces_ai/srv/SwitchAIMode` | Switch between online and offline AI modes | AI Processing | System Manager |

Example service definition (`TranslateCommand.srv`):

```
string text_command  # Natural language command
---
string translated_command  # System command
float32 confidence  # Confidence level (0.0-1.0)
```

#### System Services

| Service | Type | Description | Server | Clients |
|---------|------|-------------|--------|---------|
| `/system_manager/restart` | `std_srvs/srv/Trigger` | Restart the entire system | System Manager | Any Node |
| `/system_manager/restart_node` | `nevil_interfaces/srv/RestartNode` | Restart a specific node | System Manager | Any Node |
| `/system_manager/set_mode` | `nevil_interfaces/srv/SetMode` | Set the system mode | System Manager | Any Node |
| `/system_mode` | `std_msgs/msg/String` | Current system mode | System Manager | All Nodes |
### Actions

#### Navigation Actions

| Action | Type | Description | Server | Clients |
|--------|------|-------------|--------|---------|
| `/navigate_to_point` | `nevil_interfaces/action/NavigateToPoint` | Navigate to a specific location | Navigation | System Manager, AI Processing |
| `/follow_path` | `nav_msgs/action/FollowPath` | Follow a specified path | Navigation | System Manager |
| `/explore_area` | `nevil_interfaces/action/ExploreArea` | Explore an unknown area | Navigation | System Manager |

Example action definition (`NavigateToPoint.action`):

```
# Goal
float32 x
float32 y
float32 theta
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining
float32 estimated_time_remaining
```

Example usage:

```python
# Action client for /navigate_to_point
from nevil_interfaces.action import NavigateToPoint
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPoint, 'navigate_to_point')
        
    async def navigate_to(self, x, y, theta):
        goal_msg = NavigateToPoint.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.theta = float(theta)
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
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
        self.get_logger().info(f'Result: {result.success}, {result.message}')
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
```

#### Behavior Actions

| Action | Type | Description | Server | Clients |
|--------|------|-------------|--------|---------|
| `/perform_behavior` | `nevil_interfaces/action/PerformBehavior` | Execute an expressive behavior | Motion Control | System Manager, AI Processing |
| `/track_object` | `nevil_interfaces/action/TrackObject` | Track a detected object | Motion Control, Camera Vision | AI Processing |
| `/patrol_area` | `nevil_interfaces/action/PatrolArea` | Patrol a specified area | Navigation | System Manager |

#### AI Actions

| Action | Type | Description | Server | Clients |
|--------|------|-------------|--------|---------|
| `/process_dialog` | `nevil_interfaces_ai/action/ProcessDialog` | Handle a conversation | AI Processing | Voice Control |
| `/learn_environment` | `nevil_interfaces_ai/action/LearnEnvironment` | Learn and map the current environment | AI Processing, Navigation | System Manager |
| `/recognize_user` | `nevil_interfaces_ai/action/RecognizeUser` | Recognize and remember a user | AI Processing, Camera Vision | System Manager |

Example action definition (`ProcessDialog.action`):

```
# Goal
string initial_utterance
bool continue_conversation
---
# Result
string final_response
### Parameters

#### Motion Control Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_speed` | double | 0.5 | Maximum linear speed in m/s |
| `max_angular_speed` | double | 1.0 | Maximum angular speed in rad/s |
| `control_frequency` | double | 20.0 | Control loop frequency in Hz |
| `acceleration_limit` | double | 0.5 | Maximum acceleration in m/s² |
| `deceleration_limit` | double | 1.0 | Maximum deceleration in m/s² |

#### Obstacle Avoidance Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_distance` | double | 0.2 | Minimum safe distance in meters |
| `stop_distance` | double | 0.1 | Emergency stop distance in meters |
| `avoidance_strategy` | string | "turn_and_go" | Avoidance strategy to use |
| `sensor_angle` | double | 0.0 | Sensor mounting angle in radians |
| `max_detection_range` | double | 2.0 | Maximum detection range in meters |

#### Navigation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `path_planning_algorithm` | string | "a_star" | Path planning algorithm to use |
| `map_resolution` | double | 0.05 | Map resolution in meters per cell |
| `update_frequency` | double | 5.0 | Map update frequency in Hz |
| `goal_tolerance_position` | double | 0.1 | Position tolerance for goals in meters |
| `goal_tolerance_orientation` | double | 0.1 | Orientation tolerance for goals in radians |

#### Camera Vision Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `object_detection_model` | string | "yolov5" | Object detection model to use |
| `confidence_threshold` | double | 0.5 | Minimum confidence for detections |
| `detection_frequency` | double | 10.0 | Detection frequency in Hz |
| `image_width` | int | 640 | Camera image width in pixels |
| `image_height` | int | 480 | Camera image height in pixels |

#### Voice Control Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `listening_timeout` | double | 5.0 | Timeout for listening in seconds |
| `voice_activity_threshold` | double | 0.3 | Threshold for voice activity detection |
| `tts_engine` | string | "openai" | Text-to-speech engine to use |
| `wake_word` | string | "nevil" | Wake word for voice activation |
| `language` | string | "en-US" | Language for speech recognition |

#### AI Processing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `online_model` | string | "gpt-4" | Online AI model to use |
| `offline_model` | string | "gemma-2b" | Offline AI model to use |
| `temperature` | double | 0.7 | Temperature for text generation |
| `max_tokens` | int | 150 | Maximum tokens for text generation |
| `context_window_size` | int | 10 | Number of conversation turns to keep in context |

#### System Manager Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `default_mode` | string | "conversation" | Default system mode |
| `startup_delay` | double | 5.0 | Delay before starting all nodes in seconds |
| `watchdog_timeout` | double | 10.0 | Watchdog timeout in seconds |
| `low_battery_threshold` | double | 0.2 | Low battery warning threshold (0.0-1.0) |
| `critical_battery_threshold` | double | 0.1 | Critical battery threshold (0.0-1.0) |
bool conversation_complete
---
# Feedback
string current_state
## Python APIs

### Navigation API

The Navigation API provides programmatic access to the robot's navigation capabilities.

#### Core Navigation API

```python
from nevil_navigation.nevil_navigation_api.core import NavigationAPI

# Create a navigation API instance
nav_api = NavigationAPI()

# Navigate to a point
result = nav_api.navigate_to_point(x=1.0, y=2.0, theta=0.0)

# Check if path is clear
is_clear = nav_api.check_path_clear(x=1.0, y=2.0)

# Get current position
position = nav_api.get_current_position()

# Stop navigation
nav_api.stop()
```

#### Async Navigation API

```python
import asyncio
from nevil_navigation.nevil_navigation_api.async_api import AsyncNavigationAPI

async def navigation_example():
    # Create an async navigation API instance
    nav_api = AsyncNavigationAPI()
    
    # Navigate to a point
    result = await nav_api.navigate_to_point(x=1.0, y=2.0, theta=0.0)
    
    # Follow a path
    path_points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    result = await nav_api.follow_path(path_points)
    
    # Explore an area
    result = await nav_api.explore_area(center_x=1.0, center_y=1.0, radius=2.0)

# Run the async example
asyncio.run(navigation_example())
```

#### Real-Time Integration

```python
from nevil_navigation.nevil_navigation_api.rt_integration import RTNavigationAPI

# Create a real-time navigation API instance
rt_nav_api = RTNavigationAPI(priority=80)

# Set real-time parameters
rt_nav_api.set_rt_parameters(
    control_frequency=50.0,
    max_latency_ms=10.0,
    watchdog_timeout_ms=100.0
)

# Start real-time navigation
rt_nav_api.start()

# Navigate to a point with real-time guarantees
result = rt_nav_api.navigate_to_point_rt(x=1.0, y=2.0, theta=0.0)

# Stop and clean up
rt_nav_api.stop()
```

#### Navigation API Examples

See the [examples directory](../src/nevil_navigation/nevil_navigation/nevil_navigation_api/examples/) for more examples:

- [Basic Movement](../src/nevil_navigation/nevil_navigation/nevil_navigation_api/examples/basic_movement.py)
- [Complex Navigation](../src/nevil_navigation/nevil_navigation/nevil_navigation_api/examples/complex_navigation.py)
- [Error Handling](../src/nevil_navigation/nevil_navigation/nevil_navigation_api/examples/error_handling.py)
- [Sensor Integration](../src/nevil_navigation/nevil_navigation/nevil_navigation_api/examples/sensor_integration.py)

### Vision API

The Vision API provides programmatic access to the robot's vision capabilities.

#### Core Vision API

```python
from nevil_perception.camera_vision import VisionAPI

# Create a vision API instance
vision_api = VisionAPI()

# Capture an image
image = vision_api.capture_image()

# Detect objects
objects = vision_api.detect_objects(image)

# Recognize faces
faces = vision_api.recognize_faces(image)

# Detect ArUco markers
markers = vision_api.detect_aruco_markers(image)
```

#### Advanced Vision Features

```python
# Object tracking
tracker_id = vision_api.start_object_tracking(object_id=5)
tracking_info = vision_api.get_tracking_info(tracker_id)
vision_api.stop_tracking(tracker_id)

# Room recognition
room_signature = vision_api.create_room_signature()
vision_api.save_room_signature("living_room", room_signature)
recognized_room = vision_api.recognize_room()

# Visual SLAM
vision_api.start_visual_slam()
current_map = vision_api.get_current_map()
vision_api.stop_visual_slam()
```

### Voice API

The Voice API provides programmatic access to the robot's voice capabilities.

#### Speech Recognition

```python
from nevil_interfaces_ai.speech_recognition_node import SpeechRecognitionAPI

# Create a speech recognition API instance
speech_api = SpeechRecognitionAPI()

# Start listening
speech_api.start_listening()

# Get recognized speech
text = speech_api.get_recognized_speech(timeout=5.0)

# Stop listening
speech_api.stop_listening()
```

#### Text-to-Speech

```python
from nevil_interfaces_ai.speech_synthesis_node import SpeechSynthesisAPI

# Create a speech synthesis API instance
tts_api = SpeechSynthesisAPI()

# Speak text
tts_api.speak("Hello, I am Nevil.")

# Speak text with parameters
tts_api.speak(
    "Hello, I am Nevil.",
    voice="alloy",
    rate=1.2,
    pitch=1.0,
    volume=0.8
)

# Get available voices
voices = tts_api.get_available_voices()
```

### AI API

The AI API provides programmatic access to the robot's AI capabilities.

#### Text Processing

```python
from nevil_interfaces_ai.text_command_processor import TextCommandProcessor

# Create a text command processor instance
processor = TextCommandProcessor()

# Process a text command
result = processor.process_command("move forward 1 meter")

# Check if text is a command
is_command = processor.is_command("move forward 1 meter")

# Get command intent
intent, parameters = processor.extract_intent("move forward 1 meter")
```

#### Online AI

```python
from nevil_interfaces_ai.ai_processing_node import OnlineAIAPI

# Create an online AI API instance
online_ai = OnlineAIAPI()

# Generate text
response = online_ai.generate_text(
    "What is the capital of France?",
    max_tokens=100,
    temperature=0.7
)

# Chat completion
response = online_ai.chat_completion([
    {"role": "system", "content": "You are Nevil, a helpful robot assistant."},
    {"role": "user", "content": "What can you do?"}
])
```

#### Offline AI

```python
from nevil_interfaces_ai.ai_processing_node import OfflineAIAPI

# Create an offline AI API instance
offline_ai = OfflineAIAPI()

# Generate text
response = offline_ai.generate_text(
    "What is the capital of France?",
    max_tokens=50,
    temperature=0.8
)

# Chat completion
response = offline_ai.chat_completion([
    {"role": "system", "content": "You are Nevil, a helpful robot assistant."},
    {"role": "user", "content": "What can you do?"}
])
```

### System Management API

The System Management API provides programmatic access to the robot's system management capabilities.

#### Node Management

```python
from nevil_core.system_manager import SystemManagerAPI

# Create a system manager API instance
system_api = SystemManagerAPI()

# Restart a node
success = system_api.restart_node("camera_vision")

# Check node status
status = system_api.get_node_status("camera_vision")

# Get all node statuses
all_statuses = system_api.get_all_node_statuses()
```

#### Mode Management

```python
# Set system mode
system_api.set_mode("conversation")

# Get current mode
current_mode = system_api.get_current_mode()

# Register mode change callback
def mode_change_callback(new_mode):
    print(f"Mode changed to: {new_mode}")
## Configuration Files

### Hardware Configuration

The hardware configuration file (`hardware_config.yaml`) defines the hardware components and their parameters:

```yaml
hardware:
  motors:
    left_pin: 27
    right_pin: 17
    pwm_frequency: 1000
    direction_pins:
      left_forward: 22
      left_backward: 23
      right_forward: 24
      right_backward: 25
  
  servo:
    pin: 23
    min_angle: -45
    max_angle: 45
    center_offset: 0
  
  ultrasonic:
    trigger_pin: 23
    echo_pin: 24
    max_distance: 4.0
  
  camera:
    resolution:
      width: 640
      height: 480
    framerate: 30
    auto_exposure: true
  
  imu:
    i2c_bus: 1
    i2c_address: 0x68
    update_rate: 100
```

### ROS2 Parameters

The ROS2 parameters file (`ros_params.yaml`) defines parameters for ROS2 nodes:

```yaml
/**:
  ros__parameters:
    use_sim_time: false

motion_control_node:
  ros__parameters:
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    control_frequency: 20.0
    acceleration_limit: 0.5
    deceleration_limit: 1.0

obstacle_avoidance_node:
  ros__parameters:
    min_distance: 0.2
    stop_distance: 0.1
    avoidance_strategy: "turn_and_go"
    sensor_angle: 0.0
    max_detection_range: 2.0

navigation_node:
  ros__parameters:
    path_planning_algorithm: "a_star"
    map_resolution: 0.05
    update_frequency: 5.0
    goal_tolerance_position: 0.1
    goal_tolerance_orientation: 0.1

camera_vision_node:
  ros__parameters:
    object_detection_model: "yolov5"
    confidence_threshold: 0.5
    detection_frequency: 10.0
    image_width: 640
    image_height: 480

voice_control_node:
  ros__parameters:
    listening_timeout: 5.0
    voice_activity_threshold: 0.3
    tts_engine: "openai"
    wake_word: "nevil"
    language: "en-US"
```

### AI Configuration

The AI configuration file (`ai_config.yaml`) defines parameters for AI processing:

```yaml
ai:
  online:
    provider: "openai"
    model: "gpt-4"
    api_key_env: "OPENAI_API_KEY"
    temperature: 0.7
    max_tokens: 150
    top_p: 1.0
    frequency_penalty: 0.0
    presence_penalty: 0.0
  
  offline:
    provider: "gemma"  # Options: "gemma", "tinyllama"
    model_path: "/path/to/model"
    temperature: 0.8
    max_tokens: 100
    top_k: 40
    top_p: 0.9
  
  switching:
    prefer_online: true
    offline_threshold_ms: 500
    network_check_interval: 60
    auto_switch_on_failure: true
  
  voice:
    tts_provider: "openai"  # Options: "openai", "pyttsx3"
    openai_voice: "alloy"  # Options: "alloy", "echo", "fable", "onyx", "nova", "shimmer"
    stt_provider: "openai"  # Options: "openai", "vosk"
    language: "en-US"
    voice_activity_threshold: 0.3
    silence_duration: 1.0
```

### Simulation Configuration

The simulation configuration file (`simulation_config.yaml`) defines parameters for the digital twin simulation:

```yaml
simulation:
  physics:
    update_rate: 100  # Hz
    gravity: 9.81  # m/s²
    friction_coefficient: 0.7
    restitution_coefficient: 0.5
  
  robot:
    mass: 1.2  # kg
    wheel_radius: 0.034  # m
    wheel_separation: 0.14  # m
    max_wheel_speed: 10.0  # rad/s
    motor_torque: 0.5  # N·m
  
  sensors:
    ultrasonic:
      noise_mean: 0.0
      noise_stddev: 0.01
      update_rate: 20  # Hz
      max_range: 4.0  # m
      min_range: 0.02  # m
      field_of_view: 15  # degrees
    
    camera:
      resolution:
        width: 640
        height: 480
      field_of_view: 60  # degrees
      update_rate: 30  # Hz
      noise_factor: 0.01
    
    imu:
      noise_mean: 0.0
      noise_stddev: 0.01
      update_rate: 100  # Hz
  
  environments:
    default: "empty"
    available:
      - "empty"
      - "maze"
      - "obstacle_course"
      - "room"
```

## Command Line Tools

### System Tools

#### nevil_core_launcher

Launches the core system components:

```bash
ros2 run nevil_core nevil_core_launcher [--simulation] [--rt]
```

Options:
- `--simulation`: Launch in simulation mode
- `--rt`: Launch with real-time priorities

#### nevil_mode_switcher

Changes the system mode:

```bash
ros2 run nevil_core nevil_mode_switcher [mode]
```

Arguments:
- `mode`: System mode (conversation, play, autonomous, sleep)

#### nevil_status

Displays the current system status:

```bash
ros2 run nevil_core nevil_status [--watch]
```

Options:
- `--watch`: Continuously update the status display

### Diagnostic Tools

#### nevil_diagnostics

Runs diagnostic tests on the system:

```bash
ros2 run nevil_core nevil_diagnostics [--hardware] [--software] [--network]
```

Options:
- `--hardware`: Run hardware diagnostics
- `--software`: Run software diagnostics
- `--network`: Run network diagnostics

#### nevil_performance_monitor

Monitors system performance:

```bash
ros2 run nevil_core nevil_performance_monitor [--interval INTERVAL]
```

Options:
- `--interval INTERVAL`: Update interval in seconds (default: 1.0)

#### nevil_log_viewer

Views system logs:

```bash
ros2 run nevil_core nevil_log_viewer [--level LEVEL] [--node NODE]
```

Options:
- `--level LEVEL`: Log level (debug, info, warn, error)
- `--node NODE`: Filter logs by node name

### Development Tools

#### nevil_topic_monitor

Monitors ROS2 topics:

```bash
ros2 run nevil_core nevil_topic_monitor [topic] [--rate RATE]
```

Arguments:
- `topic`: Topic to monitor (default: all)

Options:
- `--rate RATE`: Update rate in Hz (default: 1.0)

#### nevil_service_tester

Tests ROS2 services:

```bash
ros2 run nevil_core nevil_service_tester [service] [params]
```

Arguments:
- `service`: Service to test
- `params`: Service parameters as JSON

#### nevil_action_tester

Tests ROS2 actions:

```bash
ros2 run nevil_core nevil_action_tester [action] [goal]
```

Arguments:
- `action`: Action to test
- `goal`: Action goal as JSON

system_api.register_mode_change_callback(mode_change_callback)
```

#### Resource Management

```python
# Get system resources
cpu_usage = system_api.get_cpu_usage()
memory_usage = system_api.get_memory_usage()
disk_usage = system_api.get_disk_usage()

# Get battery status
battery_status = system_api.get_battery_status()

# Set power mode
system_api.set_power_mode("performance")  # Options: "performance", "balanced", "power_saving"
```
#### Dialog Management

```python
from nevil_interfaces_ai.dialog_manager_node import DialogManagerAPI

# Create a dialog manager API instance
dialog_api = DialogManagerAPI()

# Start a conversation
conversation_id = dialog_api.start_conversation()

# Process user input
response = dialog_api.process_user_input(
    conversation_id,
    "What can you do?"
)

# End the conversation
dialog_api.end_conversation(conversation_id)
```
string thinking
```
| `/battery_status` | `sensor_msgs/msg/BatteryState` | Battery level and charging status | System Manager | All Nodes |