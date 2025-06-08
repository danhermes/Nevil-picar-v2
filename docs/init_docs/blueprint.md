# Nevil-picar v2.0: Project Blueprint

## Project Overview

Nevil-picar v2.0 is a comprehensive upgrade to the existing Nevil 1.0 platform, implementing a multi-threaded ROS2 architecture with PREEMPT-RT integration for real-time performance. This blueprint outlines the technical requirements, architecture, and implementation plan for developing this next-generation robotic companion.

## Core Requirements

1. **Multi-Threaded Architecture**
   - Implement ROS2 MultiThreadedExecutor for parallel processing
   - Prioritize critical nodes (obstacle avoidance, motion control) with real-time scheduling
   - Enable simultaneous operation of movement, vision, voice, and AI processing

2. **PREEMPT-RT Integration**
   - Install and configure PREEMPT-RT kernel on Raspberry Pi
   - Implement priority-based scheduling for ROS2 nodes
   - Configure system for deterministic, low-latency performance

3. **Digital Twin Simulation**
   - Adapt ARCHES-PiCar-X project for digital twin functionality
   - Implement text and voice interfaces for simulation
   - Create abstraction layer for seamless switching between simulation and physical robot

4. **Hybrid AI Processing**
   - Integrate OpenAI for cloud-based processing
   - Implement local models (Gemma 2 or TinyLlama) for offline cognition
   - Add offline object recognition using PiCar object recognition model

5. **SLAM and Environmental Mapping**
   - Implement SLAM functionality for mapping and navigation
   - Create persistent storage for environmental data
   - Develop location, room, and object recognition capabilities

6. **Multi-Modal Interaction**
   - Implement conversational interface with context awareness
   - Develop various operational modes (conversation, play, sleep, etc.)
   - Create command system for mode switching and task execution

## System Architecture

### Hardware Components
- Raspberry Pi 4/5 (main controller)
- PiCar-X platform (motors, servos, chassis)
- Camera module (vision input)
- Ultrasonic sensor (obstacle detection)
- Microphone and speaker (audio I/O)
- Optional IMU (motion stabilization)

### Software Stack
- ROS2 Humble (middleware)
- PREEMPT-RT patched Linux kernel
- Cyclone DDS (low-latency messaging)
- OpenCV + YOLO (vision processing)
- OpenAI API (cloud AI)
- Gemma 2 or TinyLlama (local AI)
- Context7 (documentation and knowledge base)

### ROS2 Node Structure
```
ros2_ws/src/nevil/
│── launch/            # Launch files with priority settings
│── scripts/           # Python node implementations
│   ├── motion_control.py     # Motor and servo control
│   ├── obstacle_avoidance.py # Ultrasonic sensor processing
│   ├── navigation.py         # Path planning and SLAM
│   ├── camera_vision.py      # Computer vision processing
│   ├── voice_control.py      # Speech recognition and synthesis
│   ├── ai_processing.py      # AI integration (cloud and local)
│   ├── system_manager.py     # Mode management and coordination
│── config/            # Configuration files
│── urdf/              # Robot description for simulation
│── msg/               # Custom message definitions
│── srv/               # Custom service definitions
│── action/            # Custom action definitions
│── CMakeLists.txt     # Build configuration
│── package.xml        # Package metadata
```

## Implementation Plan

### Phase 1: Environment Setup
1. Install and configure ROS2 Humble
2. Install and configure PREEMPT-RT kernel
3. Set up development environment and build system
4. Configure digital twin simulation environment

### Phase 2: Core Functionality
1. Implement basic ROS2 nodes (motion, obstacle, camera)
2. Integrate action_helper.py navigation API
3. Implement multi-threaded executor with priority scheduling
4. Test basic movement and obstacle avoidance

### Phase 3: AI and Interaction
1. Implement voice control and text interfaces
2. Integrate OpenAI for cloud processing
3. Set up local AI models for offline operation
4. Develop mode switching and command system

### Phase 4: Advanced Features
1. Implement SLAM functionality
2. Develop environmental mapping and learning
3. Create user and object recognition system
4. Implement room and location awareness

### Phase 5: Integration and Testing
1. Integrate all components into cohesive system
2. Perform comprehensive testing in simulation
3. Deploy and test on physical hardware
4. Optimize performance and fix issues

## Testing Strategy

1. **Unit Testing**
   - Test individual ROS2 nodes in isolation
   - Verify correct behavior of specific functions

2. **Integration Testing**
   - Test interaction between multiple nodes
   - Verify message passing and callback execution

3. **Simulation Testing**
   - Test complete system in digital twin environment
   - Verify behavior in various scenarios

4. **Hardware Testing**
   - Test on physical PiCar-X platform
   - Verify real-world performance and behavior

5. **Performance Testing**
   - Measure latency and jitter with cyclictest
   - Verify real-time performance under load

## Success Criteria

1. Nevil can navigate autonomously while maintaining conversation
2. System responds to obstacles in real-time without interrupting other functions
3. All modes (conversation, play, sleep, etc.) function as expected
4. SLAM functionality successfully maps and remembers environment
5. Digital twin accurately simulates physical robot behavior
6. System operates effectively both online and offline
7. Voice and text interfaces provide natural interaction