# Nevil-picar v2.0 Documentation

Welcome to the official documentation for Nevil-picar v2.0, a multi-threaded, real-time robotic system built on the PiCar-X platform that integrates ROS2 with PREEMPT-RT for deterministic performance.

## Quick Launch

```bash
# Launch the complete physical robot system
./nevil launch nevil_bringup physical_robot.launch.py

# Shutdown all nodes
pkill -f "nevil"
```

For detailed launch and system management instructions, see the [Launch and System Management Guide](launch_system_guide.md).

## Documentation Structure

This documentation is organized into several sections to help you understand, use, and develop with Nevil-picar v2.0:

### Core Documentation
1. [Project Overview](Nevil2.0%20tech/1_overview_project.md) - Introduction to Nevil-picar v2.0, its goals, and capabilities
2. [Installation and Setup](Nevil2.0%20tech/2_installation_setup.md) - Instructions for installing and configuring the system
3. [Core Concepts](Nevil2.0%20tech/3_core_concepts.md) - Explanation of the key architectural components and design principles
4. [User Guide](Nevil2.0%20tech/4_user_guide.md) - Instructions for using Nevil-picar v2.0
5. [API Reference](Nevil2.0%20tech/5_api_reference.md) - Detailed documentation of the system's APIs
6. [Developer Guide](Nevil2.0%20tech/6_developer_guide.md) - Information for developers who want to extend or modify the system
7. [Troubleshooting](Nevil2.0%20tech/7_troubleshooting.md) - Solutions to common problems and issues

### System Operations
- **[Launch and System Management Guide](launch_system_guide.md)** - Comprehensive guide for starting, monitoring, and shutting down the system
- [Build Status](build/BUILD_STATUS.md) - Current build status and known issues
- [AI Interface Architecture](ai_interface_architecture.md) - AI system design and implementation

### Architecture Documentation
- [Nevil v2.0 Architectural Overview](Nevil%202.0%20architecture/Nevil_v2.0_Architectural_Overview.md)
- [ROS2 Architecture](Nevil%202.0%20architecture/Nevil_ROS2_Architecture.md)
- [PREEMPT-RT Integration](init_docs/Nevil_PREEMPT_RT_Integration.md)

## Quick Start

To get started with Nevil-picar v2.0:

1. **Launch the System**: Use the [Launch Guide](launch_system_guide.md) for step-by-step instructions
2. **Understand the Architecture**: Read the [Core Concepts](Nevil2.0%20tech/3_core_concepts.md) to understand the system design
3. **Monitor System Status**: Follow the monitoring procedures in the [Launch Guide](launch_system_guide.md#system-monitoring)
4. **Troubleshoot Issues**: Refer to the [Troubleshooting](Nevil2.0%20tech/7_troubleshooting.md) guide for common problems

## System Overview

Nevil-picar v2.0 is a sophisticated robotic system that combines:

- **Multi-threaded ROS2 Architecture**: Enables true parallel processing across multiple nodes
- **PREEMPT-RT Integration**: Provides deterministic, low-latency performance for critical operations
- **Digital Twin Simulation**: Allows development and testing without physical hardware
- **Hybrid AI Processing**: Combines cloud-based and local AI models for robust operation
- **Multi-modal Interaction**: Supports voice, text, and visual interfaces

## Hardware Requirements

- Raspberry Pi 4/5
- PiCar-X platform
- Camera module
- Ultrasonic sensor
- Microphone/Speaker
- Optional IMU

## Software Requirements

- ROS2 Humble
- PREEMPT-RT patched Linux kernel
- Python 3.8+
- OpenAI API (for cloud-based AI)
- Local AI models (Gemma 2 or TinyLlama)

## License

[License information]

## Contributing

See the [Developer Guide](6_developer_guide.md) for information on how to contribute to Nevil-picar v2.0.

## Acknowledgments

- SunFounder for the PiCar-X platform
- ROS2 community
- PREEMPT-RT developers
- OpenAI for API access
- Contributors to the ARCHES-PiCar-X project
