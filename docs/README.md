# Nevil-picar v2.0 Documentation

Welcome to the official documentation for Nevil-picar v2.0, a multi-threaded, real-time robotic system built on the PiCar-X platform that integrates ROS2 with PREEMPT-RT for deterministic performance.

## Documentation Structure

This documentation is organized into several sections to help you understand, use, and develop with Nevil-picar v2.0:

1. [Project Overview](1_overview_project.md) - Introduction to Nevil-picar v2.0, its goals, and capabilities
2. [Installation and Setup](2_installation_setup.md) - Instructions for installing and configuring the system
3. [Core Concepts](3_core_concepts.md) - Explanation of the key architectural components and design principles
4. [User Guide](4_user_guide.md) - Instructions for using Nevil-picar v2.0
5. [API Reference](5_api_reference.md) - Detailed documentation of the system's APIs
6. [Developer Guide](6_developer_guide.md) - Information for developers who want to extend or modify the system
7. [Troubleshooting](7_troubleshooting.md) - Solutions to common problems and issues

## Quick Start

To get started with Nevil-picar v2.0:

1. Follow the [Installation and Setup](2_installation_setup.md) guide to install the necessary software and configure your system
2. Read the [Core Concepts](3_core_concepts.md) to understand the system architecture
3. Follow the [User Guide](4_user_guide.md) to learn how to use Nevil-picar v2.0

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
