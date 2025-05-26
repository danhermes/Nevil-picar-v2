# Nevil-picar v2.0: Project Overview

## Introduction

Nevil-picar v2.0 is a multi-threaded, real-time robotic system built on the PiCar-X platform that integrates ROS2 with PREEMPT-RT for deterministic performance. It combines conversational AI, autonomous navigation, computer vision, and environmental mapping into a cohesive robotic companion capable of simultaneous operations across multiple processing threads.

This document provides a comprehensive overview of the Nevil-picar v2.0 project, including its goals, architecture, key features, and components.

## Table of Contents

- [Project Goals](#project-goals)
- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
- [ROS2 Node Structure](#ros2-node-structure)
- [Multi-Threading Architecture](#multi-threading-architecture)
- [Digital Twin Simulation](#digital-twin-simulation)
- [Text and Voice Interface](#text-and-voice-interface)
- [Improvements Over v1.0](#improvements-over-v10)

## Project Goals

Nevil-picar v2.0 aims to create a fully functional, responsive robotic companion that can:

- Navigate autonomously while maintaining conversational interaction
- Map and learn its environment through SLAM functionality
- Recognize and track users and objects using both online and offline vision processing
- Operate in various modes (conversation, play, autonomous, sleep) with seamless transitions
- Maintain real-time responsiveness through multi-threaded execution and PREEMPT-RT integration
- Function both with a physical robot and through a digital twin simulation
- Provide offline and online functionality and cognition

## System Architecture

Nevil-picar v2.0 implements a layered architecture that integrates hardware, middleware, ROS2 nodes, AI processing, and digital twin simulation:

### Architecture Layers

1. **Hardware Layer**: Physical components including Raspberry Pi, PiCar-X platform, sensors, and actuators
2. **OS & Middleware Layer**: PREEMPT-RT Linux kernel, ROS2 Humble, and Cyclone DDS
3. **ROS2 Nodes Layer**: Specialized nodes for different functionalities with priority-based execution
4. **AI & Processing Layer**: Cloud and local AI models for cognition and decision-making
5. **Digital Twin Layer**: Simulation environment for development and testing

### System Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     Hardware Layer                          │
│  ┌─────────┐  ┌──────────┐  ┌────────┐  ┌────────────────┐  │
│  │Raspberry│  │PiCar-X   │  │Sensors │  │Microphone/     │  │
│  │Pi 4/5   │  │Platform  │  │        │  │Speaker         │  │
│  └─────────┘  └──────────┘  └────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────────┐
│                 OS & Middleware Layer                       │
│  ┌─────────────┐  ┌──────────┐  ┌────────────────────────┐  │
│  │PREEMPT-RT   │  │ROS2      │  │Cyclone DDS            │  │
│  │Linux Kernel │  │Humble    │  │                       │  │
│  └─────────────┘  └──────────┘  └────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Nodes Layer                         │
│  ┌─────────┐  ┌──────────┐  ┌────────┐  ┌────────────────┐  │
│  │Motion   │  │Obstacle  │  │Naviga- │  │Camera Vision   │  │
│  │Control  │  │Avoidance │  │tion    │  │                │  │
│  └─────────┘  └──────────┘  └────────┘  └────────────────┘  │
│                                                             │
│  ┌─────────┐  ┌──────────┐  ┌────────────────────────────┐  │
│  │Voice    │  │AI        │  │System Manager              │  │
│  │Control  │  │Processing│  │                            │  │
│  └─────────┘  └──────────┘  └────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────────┐
│                  AI & Processing Layer                      │
│  ┌─────────┐  ┌──────────┐  ┌────────┐  ┌────────────────┐  │
│  │OpenAI   │  │Local AI  │  │SLAM    │  │Object          │  │
│  │API      │  │Models    │  │Process │  │Recognition     │  │
│  └─────────┘  └──────────┘  └────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────────┐
│                  Digital Twin Layer                         │
│  ┌─────────┐  ┌──────────┐  ┌────────┐  ┌────────────────┐  │
│  │Simulation│ │Hardware  │  │Sensor  │  │Physics Engine  │  │
│  │Environment││Abstraction│ │Simulation│ │               │  │
│  └─────────┘  └──────────┘  └────────┘  └────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Key Features

### Multi-Threaded Architecture
- True parallel processing with ROS2's MultiThreadedExecutor
- Priority-based scheduling for critical operations
- Deterministic performance with PREEMPT-RT integration

### Real-Time Performance
- Sub-millisecond latency for critical operations
- Deterministic execution with PREEMPT-RT
- Isolated execution with priority scheduling

### Autonomous Navigation
- Path planning and execution
- Obstacle detection and avoidance
- Environmental mapping with SLAM

### Hybrid AI Processing
- Cloud-based processing with OpenAI API
- Local processing with Gemma 2 or TinyLlama
- Seamless switching between online and offline modes

### Multi-Modal Interaction
- Voice interface with speech recognition and synthesis
- Text interface for command input and output
- Visual processing for object and user recognition

### Digital Twin Simulation
- Accurate physics simulation of PiCar-X platform
- Simulated sensors with realistic behavior
- Seamless switching between simulation and physical hardware

## Hardware Components

| Component | Description | Interactions |
|-----------|-------------|--------------|
| Raspberry Pi 4/5 | Main controller running ROS2 and PREEMPT-RT | Interfaces with all hardware components |
| PiCar-X Platform | Physical chassis with motors and servos | Controlled by Motion Control Node |
| Camera Module | Vision input for object detection and tracking | Feeds data to Camera Vision Node |
| Ultrasonic Sensor | Distance measurement for obstacle detection | Feeds data to Obstacle Avoidance Node |
| Microphone/Speaker | Audio I/O for voice interaction | Interfaces with Voice Control Node |
| Optional IMU | Motion stabilization and orientation data | Provides data to Navigation Node |

## Software Components

| Component | Description | Interactions |
|-----------|-------------|--------------|
| ROS2 Humble | Middleware for inter-node communication | Connects all nodes via topics, services, and actions |
| PREEMPT-RT Kernel | Real-time Linux kernel for deterministic performance | Provides scheduling guarantees for critical nodes |
| Cyclone DDS | Low-latency messaging system | Handles ROS2 message transport |
| OpenCV + YOLO | Vision processing libraries | Used by Camera Vision Node |
| OpenAI API | Cloud-based AI processing | Used by AI Processing Node and Voice Control Node |
| Gemma 2/TinyLlama | Local AI models for offline operation | Used by AI Processing Node |
| SLAM Algorithms | Mapping and localization | Used by Navigation Node |

## ROS2 Node Structure

Nevil-picar v2.0 is organized into several ROS2 nodes, each responsible for specific functionality:

### High Priority Nodes
- **Motion Control Node**: Controls the motors and servos for movement
- **Obstacle Avoidance Node**: Processes sensor data to detect and avoid obstacles

### Medium Priority Nodes
- **Navigation Node**: Handles path planning and execution
- **Camera Vision Node**: Processes camera input for object detection and tracking

### Lower Priority Nodes
- **Voice Control Node**: Manages speech recognition and synthesis
- **AI Processing Node**: Handles AI processing and decision-making
- **System Manager Node**: Coordinates overall system behavior and mode switching

### Node Interactions

The nodes communicate through ROS2 topics, services, and actions:

| Interface Type | Name | Description | Publishers | Subscribers |
|----------------|------|-------------|------------|-------------|
| Topic | `/cmd_vel` | Velocity commands | Navigation, Obstacle Avoidance | Motion Control |
| Topic | `/ultrasonic_data` | Distance sensor readings | Sensor Driver | Obstacle Avoidance |
| Topic | `/camera/image_raw` | Raw camera images | Camera Driver | Camera Vision |
| Topic | `/object_detections` | Detected objects | Camera Vision | Navigation, AI Processing |
| Topic | `/system_mode` | Current system mode | System Manager | All Nodes |
| Service | `/check_obstacle` | Obstacle check service | Obstacle Avoidance | Any Node |
| Action | `/navigate_to_point` | Navigation action | Navigation | System Manager, AI Processing |
| Action | `/perform_behavior` | Expressive behavior action | Motion Control | System Manager, AI Processing |

## Multi-Threading Architecture

Nevil-picar v2.0 leverages ROS2's MultiThreadedExecutor to enable true parallel processing across nodes:

### ROS2 MultiThreadedExecutor

The system uses ROS2's MultiThreadedExecutor to run multiple nodes concurrently:

```python
def main():
    rclpy.init()
    motion = MotionControl()
    obstacle = ObstacleAvoidance()
    navigation = Navigation()
    vision = CameraVision()
    voice = VoiceControl()
    ai = AIProcessing()
    system = SystemManager()

    executor = MultiThreadedExecutor()
    executor.add_node(motion)
    executor.add_node(obstacle)
    executor.add_node(navigation)
    executor.add_node(vision)
    executor.add_node(voice)
    executor.add_node(ai)
    executor.add_node(system)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Cleanup nodes
    rclpy.shutdown()
```

### PREEMPT-RT Integration

The PREEMPT-RT patched kernel enables deterministic, low-latency performance:

1. **Kernel Configuration**:
   - PREEMPT-RT patched kernel installed on Raspberry Pi
   - Real-time scheduling policies (SCHED_FIFO) enabled
   - Power-saving features disabled to prevent latency spikes

2. **Priority-Based Scheduling**:
   - Critical nodes (obstacle avoidance, motion control) run at highest priority
   - Medium-priority nodes (navigation, vision) run at intermediate priority
   - Non-critical nodes (voice, AI) run at lower priority

3. **Launch with Priority**:
   ```python
   nodes = [
       ("nevil", "motion_control", 90),      # Highest priority
       ("nevil", "obstacle_avoidance", 85),
       ("nevil", "navigation", 80),
       ("nevil", "camera_vision", 70),
       ("nevil", "voice_control", 60),
       ("nevil", "ai_processing", 50),
       ("nevil", "system_manager", 40)       # Lowest priority
   ]

   for package, executable, priority in nodes:
       cmd = ["sudo", "chrt", "-f", str(priority), "ros2", "run", package, executable]
       subprocess.Popen(cmd)
   ```

### Performance Metrics

- **Latency Requirements**:
  - Obstacle detection and avoidance: < 50ms
  - Motion control command execution: < 20ms
  - Voice command recognition: < 1s
  - System mode switching: < 500ms

## Digital Twin Simulation

Nevil-picar v2.0 includes a digital twin simulation that provides a virtual representation of the physical robot:

### Simulation Architecture

The digital twin simulation is based on the ARCHES-PiCar-X project and includes:

1. **URDF Model**: Accurate representation of PiCar-X physical dimensions and properties
2. **Physics Simulation**: Realistic motion dynamics and interactions
3. **Sensor Simulation**: Virtual ultrasonic, camera, and IMU sensors
4. **Environment Simulation**: Configurable test environments for navigation and SLAM testing

### Hardware Abstraction Layer

A key component of the digital twin architecture is the hardware abstraction layer that allows seamless switching between simulation and physical hardware:

```python
class HardwareInterface:
    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        if simulation_mode:
            self.interface = SimulationInterface()
        else:
            self.interface = PhysicalHardwareInterface()
    
    def set_motor_speed(self, left, right):
        return self.interface.set_motor_speed(left, right)
    
    def get_distance(self):
        return self.interface.get_distance()
    
    def get_camera_image(self):
        return self.interface.get_camera_image()
    
    # Additional hardware methods...
```

## Text and Voice Interface

Nevil-picar v2.0 provides multi-modal interaction through text and voice interfaces:

### Voice Control Architecture

The voice control system includes:

1. **Voice Activity Detection**: Detects when someone is speaking to the robot
2. **Speech-to-Text**: Converts spoken language to text
3. **Intent Recognition**: Identifies the user's intent from the text
4. **Command Execution**: Executes the appropriate action
5. **Text-to-Speech**: Converts the robot's response to spoken language

### Hybrid AI Processing

Nevil v2.0 implements a hybrid approach to AI processing:

1. **Cloud-Based Processing (Online)**:
   - OpenAI API for advanced language understanding
   - High-quality text-to-speech synthesis
   - Complex reasoning and conversation

2. **Local Processing (Offline)**:
   - Gemma 2 or TinyLlama for basic cognition
   - Local speech recognition for command processing
   - Object recognition using optimized models

3. **Seamless Switching**:
   - Automatic fallback to local processing when offline
   - Prioritization based on task complexity
   - Caching of common responses for reduced latency

### Conversation and Context Management

The system maintains conversation context to provide coherent interactions:

```python
class ContextManager:
    def __init__(self):
        self.conversation_history = []
        self.environment_context = {}
        self.user_profiles = {}
        self.current_mode = "conversation"
    
    def add_user_utterance(self, user_id, text):
        self.conversation_history.append({"role": "user", "id": user_id, "content": text})
    
    def add_system_response(self, text):
        self.conversation_history.append({"role": "assistant", "content": text})
    
    def update_environment(self, location, objects, room_type):
        self.environment_context.update({
            "location": location,
            "visible_objects": objects,
            "room_type": room_type
        })
    
    def get_context_for_ai(self):
        # Format context for AI processing
        return {
            "conversation": self.conversation_history[-10:],  # Last 10 exchanges
            "environment": self.environment_context,
            "mode": self.current_mode
        }
```

## Improvements Over v1.0

Nevil-picar v2.0 represents a significant advancement over v1.0:

### Multi-Threading Enhancements

| v1.0 Limitation | v2.0 Improvement |
|-----------------|------------------|
| Single-threaded execution | True parallel processing with MultiThreadedExecutor |
| Sequential processing of sensors and actions | Concurrent processing with priority-based scheduling |
| Blocking operations during speech processing | Non-blocking callbacks with real-time guarantees |
| Limited responsiveness during complex tasks | Deterministic performance with PREEMPT-RT |

### Real-Time Performance

| v1.0 Limitation | v2.0 Improvement |
|-----------------|------------------|
| Unpredictable latency for critical operations | Sub-millisecond latency for obstacle avoidance |
| System jitter affecting motion control | Deterministic execution with PREEMPT-RT |
| Interference between subsystems | Isolated execution with priority scheduling |
| Limited performance monitoring | Comprehensive latency tracking and reporting |

### Modularity and Extensibility

| v1.0 Limitation | v2.0 Improvement |
|-----------------|------------------|
| Monolithic code structure | Modular ROS2 node architecture |
| Tight coupling between components | Well-defined interfaces using ROS2 communication patterns |
| Limited abstraction for hardware | Hardware abstraction layer for simulation and physical hardware |
| Fixed behavior patterns | Configurable behaviors through ROS2 parameters |

## Conclusion

Nevil-picar v2.0 represents a significant architectural advancement, implementing a multi-threaded ROS2 architecture with PREEMPT-RT integration for real-time performance. The system is designed to provide a responsive, intelligent robotic companion capable of simultaneous operations across multiple processing threads.

The architecture addresses the limitations of v1.0 by enabling true parallel processing, deterministic real-time performance, and seamless integration between physical hardware and simulation. The hybrid AI approach provides both advanced cloud-based capabilities and robust offline functionality.