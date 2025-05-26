# Nevil-picar v2.0: Project Requirements Document (PRD)

## 1. Introduction

### 1.1 Purpose
This document defines the comprehensive requirements for Nevil-picar v2.0, a multi-threaded ROS2-based robotic system with PREEMPT-RT integration. It serves as the authoritative reference for development, testing, and validation.

### 1.2 Scope
Nevil-picar v2.0 is a complete upgrade to the existing Nevil 1.0 platform, implementing advanced robotics capabilities including autonomous navigation, environmental mapping, multi-modal interaction, and hybrid AI processing. The system will function both as a physical robot and through a digital twin simulation.

### 1.3 Definitions and Acronyms
- **ROS2**: Robot Operating System 2
- **PREEMPT-RT**: Preemptible Real-Time Linux kernel patch
- **SLAM**: Simultaneous Localization and Mapping
- **DDS**: Data Distribution Service
- **IMU**: Inertial Measurement Unit
- **STT**: Speech-to-Text
- **TTS**: Text-to-Speech

## 2. System Overview

### 2.1 System Context
Nevil-picar v2.0 operates as an autonomous robotic companion capable of navigation, conversation, environmental learning, and multi-modal interaction. It integrates hardware components (PiCar-X platform, sensors, etc.) with a sophisticated software stack (ROS2, PREEMPT-RT, AI models) to create a responsive, intelligent system.

### 2.2 User Characteristics
- Primary user: Dan (main operator)
- Secondary users: Danielle and others in the household
- The system should recognize and adapt to different users

### 2.3 Assumptions and Dependencies
- Raspberry Pi 4/5 with sufficient processing power
- PiCar-X hardware in working condition
- Internet connectivity for cloud AI (with offline fallback)
- ROS2 Humble and compatible libraries
- PREEMPT-RT kernel compatibility with Raspberry Pi

## 3. Functional Requirements

### 3.1 Multi-Threaded Architecture

#### 3.1.1 ROS2 Node Structure
- **FR-1.1**: Implement a MultiThreadedExecutor to run multiple nodes concurrently
- **FR-1.2**: Create separate nodes for motion control, obstacle avoidance, navigation, vision, voice, and AI processing
- **FR-1.3**: Establish appropriate topic, service, and action interfaces between nodes
- **FR-1.4**: Implement priority-based scheduling for critical nodes

#### 3.1.2 Thread Management
- **FR-1.5**: Configure thread priorities according to criticality (obstacle avoidance > motion control > navigation > vision > voice > AI)
- **FR-1.6**: Implement thread synchronization mechanisms where necessary
- **FR-1.7**: Monitor thread performance and resource usage

### 3.2 PREEMPT-RT Integration

#### 3.2.1 Kernel Configuration
- **FR-2.1**: Install PREEMPT-RT patched kernel on Raspberry Pi
- **FR-2.2**: Configure kernel parameters for optimal real-time performance
- **FR-2.3**: Set up real-time scheduling policies (SCHED_FIFO)
- **FR-2.4**: Disable power-saving features that impact real-time performance

#### 3.2.2 Real-Time Performance
- **FR-2.5**: Achieve sub-millisecond latency for critical operations
- **FR-2.6**: Implement latency monitoring and reporting
- **FR-2.7**: Optimize critical code paths for deterministic execution

### 3.3 Digital Twin Simulation

#### 3.3.1 Simulation Environment
- **FR-3.1**: Adapt ARCHES-PiCar-X project for digital twin functionality
- **FR-3.2**: Create accurate physics simulation of PiCar-X platform
- **FR-3.3**: Simulate sensors (camera, ultrasonic, etc.) with realistic behavior
- **FR-3.4**: Implement environment simulation for testing navigation and SLAM

#### 3.3.2 Hardware Abstraction
- **FR-3.5**: Develop abstraction layer for hardware components
- **FR-3.6**: Implement seamless switching between simulation and physical hardware
- **FR-3.7**: Ensure consistent behavior across simulation and physical robot

### 3.4 Navigation and Mapping

#### 3.4.1 Basic Movement
- **FR-4.1**: Integrate action_helper.py for proven navigation capabilities
- **FR-4.2**: Implement forward, backward, turning, and stopping functions
- **FR-4.3**: Calibrate movement parameters for accurate control
- **FR-4.4**: Implement speed control and acceleration profiles

#### 3.4.2 Obstacle Avoidance
- **FR-4.5**: Process ultrasonic sensor data for obstacle detection
- **FR-4.6**: Implement real-time obstacle avoidance algorithms
- **FR-4.7**: Create different avoidance strategies based on obstacle type and context
- **FR-4.8**: Integrate obstacle avoidance with path planning

#### 3.4.3 SLAM Implementation
- **FR-4.9**: Implement SLAM algorithms for mapping and localization
- **FR-4.10**: Create persistent storage for map data
- **FR-4.11**: Develop map update and refinement capabilities
- **FR-4.12**: Implement path planning using map data

### 3.5 AI and Cognition

#### 3.5.1 Cloud AI Integration
- **FR-5.1**: Integrate OpenAI API for advanced processing
- **FR-5.2**: Implement efficient API usage to minimize latency and costs
- **FR-5.3**: Develop fallback mechanisms for connectivity issues
- **FR-5.4**: Create context management for conversational continuity

#### 3.5.2 Local AI Processing
- **FR-5.5**: Implement Gemma 2 or TinyLlama for offline cognition
- **FR-5.6**: Optimize local models for Raspberry Pi performance
- **FR-5.7**: Develop seamless switching between cloud and local processing
- **FR-5.8**: Implement offline object recognition using PiCar model

#### 3.5.3 Environmental Learning
- **FR-5.9**: Develop capabilities to learn and remember locations
- **FR-5.10**: Implement room and area classification
- **FR-5.11**: Create user recognition and tracking
- **FR-5.12**: Develop object recognition and categorization

### 3.6 Interaction Capabilities

#### 3.6.1 Voice Interface
- **FR-6.1**: Implement speech recognition for command input
- **FR-6.2**: Develop text-to-speech for robot responses
- **FR-6.3**: Create voice activity detection for conversation initiation
- **FR-6.4**: Implement voice characteristics recognition for user identification

#### 3.6.2 Text Interface
- **FR-6.5**: Develop text-based command interface
- **FR-6.6**: Implement text output for responses and status
- **FR-6.7**: Create logging system for conversation history
- **FR-6.8**: Develop command syntax and parsing

#### 3.6.3 Behavioral Modes
- **FR-6.9**: Implement conversation mode for interactive dialogue
- **FR-6.10**: Develop play mode for autonomous exploration and interaction
- **FR-6.11**: Create sleep mode for power conservation
- **FR-6.12**: Implement shutdown procedure for safe system termination
- **FR-6.13**: Develop mode switching commands and transitions

## 4. Non-Functional Requirements

### 4.1 Performance

#### 4.1.1 Latency
- **NFR-1.1**: Obstacle detection and avoidance response < 50ms
- **NFR-1.2**: Motion control command execution < 20ms
- **NFR-1.3**: Voice command recognition < 1s
- **NFR-1.4**: System mode switching < 500ms

#### 4.1.2 Resource Usage
- **NFR-1.5**: CPU usage < 80% under normal operation
- **NFR-1.6**: Memory usage < 2GB
- **NFR-1.7**: Storage requirements < 16GB
- **NFR-1.8**: Battery life > 2 hours of continuous operation

### 4.2 Reliability

- **NFR-2.1**: System uptime > 99% during active use
- **NFR-2.2**: Graceful degradation when resources are constrained
- **NFR-2.3**: Automatic recovery from non-critical failures
- **NFR-2.4**: Safe shutdown on critical failures

### 4.3 Usability

- **NFR-3.1**: Voice commands recognized with > 90% accuracy
- **NFR-3.2**: Natural language understanding for common instructions
- **NFR-3.3**: Intuitive mode switching and command structure
- **NFR-3.4**: Clear feedback for command recognition and execution

### 4.4 Maintainability

- **NFR-4.1**: Modular code structure with clear separation of concerns
- **NFR-4.2**: Comprehensive documentation for all components
- **NFR-4.3**: Logging system for debugging and performance analysis
- **NFR-4.4**: Version control for all code and configuration

## 5. System Interfaces

### 5.1 Hardware Interfaces

- **SI-1.1**: PiCar-X motor and servo control
- **SI-1.2**: Camera module for vision input
- **SI-1.3**: Ultrasonic sensor for distance measurement
- **SI-1.4**: Microphone for audio input
- **SI-1.5**: Speaker for audio output
- **SI-1.6**: Optional IMU for motion data

### 5.2 Software Interfaces

- **SI-2.1**: ROS2 middleware for inter-node communication
- **SI-2.2**: Linux kernel interfaces for real-time scheduling
- **SI-2.3**: OpenAI API for cloud AI processing
- **SI-2.4**: Local AI model interfaces
- **SI-2.5**: Context7 for documentation access

### 5.3 Communication Interfaces

- **SI-3.1**: DDS for ROS2 message transport
- **SI-3.2**: HTTP/HTTPS for API communication
- **SI-3.3**: Audio I/O for voice interaction
- **SI-3.4**: Video streaming for remote monitoring (optional)

## 6. Implementation Plan

### 6.1 Development Phases

#### 6.1.1 Phase 1: Environment Setup (Weeks 1-2)
- Set up ROS2 Humble environment
- Install and configure PREEMPT-RT kernel
- Configure development tools and build system
- Set up digital twin simulation environment

#### 6.1.2 Phase 2: Core Functionality (Weeks 3-5)
- Implement basic ROS2 nodes
- Integrate action_helper.py navigation API
- Implement multi-threaded executor
- Test basic movement and obstacle avoidance

#### 6.1.3 Phase 3: AI and Interaction (Weeks 6-8)
- Implement voice and text interfaces
- Integrate OpenAI for cloud processing
- Set up local AI models
- Develop mode switching system

#### 6.1.4 Phase 4: Advanced Features (Weeks 9-11)
- Implement SLAM functionality
- Develop environmental mapping
- Create user and object recognition
- Implement room awareness

#### 6.1.5 Phase 5: Integration and Testing (Weeks 12-14)
- Integrate all components
- Perform comprehensive testing
- Deploy to physical hardware
- Optimize and fix issues

### 6.2 Testing Strategy

#### 6.2.1 Unit Testing
- Test individual ROS2 nodes
- Verify component functionality

#### 6.2.2 Integration Testing
- Test node interactions
- Verify system behavior

#### 6.2.3 Simulation Testing
- Test in digital twin environment
- Verify behavior in various scenarios

#### 6.2.4 Hardware Testing
- Test on physical PiCar-X platform
- Verify real-world performance

#### 6.2.5 Performance Testing
- Measure latency and jitter
- Verify real-time capabilities

## 7. Appendices

### 7.1 Reference Documents
- Nevil_ROS2_PREEMPT-RT_Spec_and_Code_Samples.md
- Nevil_ROS2_Architecture.md
- Nevil_PREEMPT_RT_Integration.md
- PiCar-X documentation (docs-sunfounder-com-picar-x-en-latest.pdf)
- ARCHES-PiCar-X-main project

### 7.2 Glossary
- **ROS2**: Robot Operating System 2, a flexible framework for writing robot software
- **PREEMPT-RT**: A set of patches for the Linux kernel that allows for better real-time performance
- **SLAM**: Simultaneous Localization and Mapping, a technique for creating maps while tracking location
- **Digital Twin**: A virtual representation of a physical object or system
- **MultiThreadedExecutor**: A ROS2 component that allows multiple callbacks to run concurrently