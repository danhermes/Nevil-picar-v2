# Nevil v3.0 - Lightweight Robot Framework

## Overview

Nevil v3.0 is a lightweight, custom framework designed to replace the complex ROS2 architecture of v2.0 with a simple, elegant, and maintainable solution. The framework preserves the proven working audio approaches from v1.0 while implementing a clean node-based architecture with threading, messaging, and launch capabilities.

**Core Philosophy**: "Simple architecture = working robot"

## 🎯 Design Goals

- **Simplicity**: Minimal dependencies, clear interfaces, easy to understand
- **Reliability**: Proven v1.0 components with robust error handling
- **Maintainability**: Clean code structure with comprehensive documentation
- **Performance**: Efficient resource usage with real-time capabilities
- **Extensibility**: Easy to add new nodes and features

## 📋 Specification Documents

This specification consists of 10 comprehensive documents covering all aspects of Nevil v3.0:

### Core Architecture
1. **[Technical Architecture Specification](01_technical_architecture_specification.md)**
   - System overview and design principles
   - Component relationships and interfaces
   - Performance requirements and constraints

2. **[Node Structure and Threading Model](02_node_structure_threading_model.md)**
   - Base node class architecture
   - Threading patterns and synchronization
   - Inter-process communication design

3. **[Configuration File Formats](03_configuration_file_formats.md)**
   - `.nodes` file format and validation
   - `.messages` file format for each node
   - Environment variable management

### System Components
4. **[Launch System Architecture](04_launch_system_architecture.md)**
   - Node lifecycle management
   - Dependency resolution and startup ordering
   - Health monitoring and recovery

5. **[Logging Architecture](05_logging_architecture.md)**
   - Structured logging with multiple levels
   - Log rotation and archiving
   - Real-time monitoring and alerting

6. **[Framework Core Components Pseudocode](06_framework_core_components_pseudocode.md)**
   - Message bus implementation
   - Configuration manager
   - Node registry and error handling

### Implementation Details
7. **[Node Implementation Pseudocode](07_node_implementation_pseudocode.md)**
   - Speech recognition node
   - Speech synthesis node
   - AI cognition node

8. **[Audio Integration Strategy](08_audio_integration_strategy.md)**
   - v1.0 audio component preservation
   - Hardware abstraction layer
   - Testing and validation approach

### Documentation and Deployment
9. **[Documentation Folder Structure](09_docs_folder_structure.md)**
   - Complete documentation organization
   - Standards and templates
   - Maintenance procedures

10. **[Integration Plan with v1.0 Components](10_integration_plan_v1_components.md)**
    - Phase-by-phase migration strategy
    - Risk mitigation and rollback plans
    - Success criteria and timeline

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Nevil v3.0 Framework                     │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Speech          │ │ Speech          │ │ AI Cognition    ││
│  │ Recognition     │ │ Synthesis       │ │ Node            ││
│  │ Node            │ │ Node            │ │                 ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Message Bus     │ │ Node Manager    │ │ Config Manager  ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Logging System  │ │ Error Handler   │ │ Health Monitor  ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                    v1.0 Audio Components                    │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Audio Input     │ │ Audio Output    │ │ Hardware        ││
│  │ (Microphone)    │ │ (Speaker)       │ │ Abstraction     ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

## 🔧 Key Features

### Framework Features
- **Node-based Architecture**: Autonomous processes with clear responsibilities
- **Message Bus**: Simple publish/subscribe communication
- **Configuration-Driven**: YAML-based setup with hot-reloading
- **Comprehensive Logging**: Structured logs with automatic rotation
- **Health Monitoring**: Real-time system health and performance tracking
- **Error Recovery**: Automatic restart and recovery mechanisms

### Audio Features (v1.0 Proven)
- **Speech Recognition**: OpenAI Whisper with optimized parameters
- **Speech Synthesis**: OpenAI TTS with HiFiBerry DAC output
- **Hardware Integration**: USB microphone and I2S DAC support
- **Real-time Processing**: Low-latency audio pipeline
- **Quality Assurance**: Proven audio quality and reliability

### AI Features
- **Conversation Management**: Context-aware dialogue system
- **Command Processing**: Natural language understanding
- **Response Generation**: Intelligent and contextual responses
- **Fallback Capabilities**: Offline operation when API unavailable

## 🚀 Quick Start

### Prerequisites
- Raspberry Pi 4/5 with Raspberry Pi OS
- PiCar-X hardware platform
- USB microphone and HiFiBerry DAC
- OpenAI API key

### Installation
```bash
# Clone the repository
git clone https://github.com/your-org/nevil-v3.git
cd nevil-v3

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Validate configuration
./nevil validate

# Start the system
./nevil start
```

### Basic Usage
```bash
# Check system status
./nevil status

# View logs
./nevil logs

# Stop the system
./nevil stop
```

## 📁 Project Structure

```
nevil_v3/
├── .nodes                          # Main configuration file
├── .env.example                    # Environment template
├── nevil                          # CLI script
├── nevil_framework/               # Core framework
│   ├── base_node.py              # Base node class
│   ├── message_bus.py            # Message system
│   ├── launcher.py               # System launcher
│   ├── config_manager.py         # Configuration management
│   ├── log_manager.py            # Logging system
│   └── error_handler.py          # Error handling
├── nodes/                        # Node implementations
│   ├── speech_recognition/       # Voice input processing
│   ├── speech_synthesis/         # Voice output processing
│   └── ai_cognition/            # AI conversation management
├── audio/                        # v1.0 Audio components
│   ├── audio_input.py           # Microphone handling
│   ├── audio_output.py          # Speaker handling
│   └── hardware_abstraction.py  # Hardware management
├── logs/                         # System and node logs
├── tests/                        # Test suite
└── docs/                         # Documentation
    ├── nevil_v3/                # v3.0 specifications
    ├── user_guide/              # User documentation
    ├── developer_guide/         # Developer documentation
    └── api_reference/           # API documentation
```

## 🔄 Message Flow

```
┌─────────────────┐    voice_command    ┌─────────────────┐
│ Speech          │ ──────────────────> │ AI Cognition    │
│ Recognition     │                     │ Node            │
└─────────────────┘                     └─────────────────┘
         │                                       │
         │ speaking_status                       │ text_response
         │                                       │
         v                                       v
┌─────────────────┐                     ┌─────────────────┐
│ Speech          │ <─────────────────── │ Message Bus     │
│ Synthesis       │                     │                 │
└─────────────────┘                     └─────────────────┘
```

## ⚙️ Configuration Example

```yaml
# .nodes - Main configuration file
version: "3.0"
description: "Nevil v3.0 lightweight framework"

nodes:
  speech_recognition:
    status: live                    # live | muted | disabled
    priority: high                  # high | medium | low
    restart_policy: always          # always | on_failure | never
    environment:
      OPENAI_API_KEY: "${OPENAI_API_KEY}"
      ENERGY_THRESHOLD: "300"
      MICROPHONE_DEVICE_INDEX: "1"
  
  speech_synthesis:
    status: live
    priority: high
    restart_policy: always
    environment:
      OPENAI_API_KEY: "${OPENAI_API_KEY}"
      TTS_VOICE: "onyx"
      VOLUME_DB: "6"
  
  ai_cognition:
    status: live
    priority: medium
    restart_policy: on_failure
    environment:
      OPENAI_API_KEY: "${OPENAI_API_KEY}"
      GPT_MODEL: "gpt-3.5-turbo"

launch:
  startup_order: [speech_recognition, speech_synthesis, ai_cognition]
  wait_for_healthy: true
  ready_timeout: 30.0
```

## 🧪 Testing

### Running Tests
```bash
# Run all tests
python -m pytest tests/

# Run specific test categories
python -m pytest tests/test_audio.py      # Audio integration tests
python -m pytest tests/test_nodes.py      # Node functionality tests
python -m pytest tests/test_framework.py  # Framework core tests

# Run performance tests
python -m pytest tests/test_performance.py
```

### Test Coverage
- Unit tests for all framework components
- Integration tests for node communication
- Audio hardware tests with real devices
- Performance benchmarking
- Stress testing with multiple nodes

## 🔧 Development

### Creating a New Node
```bash
# Generate node template
./nevil node create my_new_node

# Edit the generated files
# - nodes/my_new_node/my_new_node_node.py
# - nodes/my_new_node/.messages

# Add to .nodes configuration
# Update launch order if needed
```

### Debugging
```bash
# Enable debug logging
export LOG_LEVEL=DEBUG

# View real-time logs
./nevil logs -f

# View specific node logs
./nevil logs speech_recognition -f

# Check system health
./nevil status
```

## 📊 Performance Characteristics

### Target Performance
- **Voice Recognition Latency**: < 2 seconds
- **Speech Synthesis Latency**: < 3 seconds
- **Inter-node Message Delivery**: < 100ms
- **System Startup Time**: < 30 seconds
- **Memory Usage per Node**: < 200MB
- **System Uptime**: > 99% with auto-recovery

### Resource Usage
- **Total Memory**: ~500MB (3 nodes + framework)
- **CPU Usage**: < 50% on Raspberry Pi 4
- **Disk Space**: < 1GB including logs
- **Network**: Local IPC only (no external dependencies)

## 🛡️ Reliability Features

### Error Handling
- Automatic node restart on failure
- Circuit breaker patterns to prevent cascading failures
- Graceful degradation when components fail
- Comprehensive error logging and analysis

### Recovery Mechanisms
- Audio system recovery from hardware failures
- API timeout handling with local fallbacks
- Configuration hot-reloading without restart
- Automatic log rotation and cleanup

### Monitoring
- Real-time health monitoring
- Performance metrics collection
- Alert system for critical issues
- Resource usage tracking

## 🔄 Migration from Previous Versions

### From v1.0
- **Audio Compatibility**: 100% preserved
- **Configuration**: Automatic migration
- **Performance**: Equal or better
- **Features**: All v1.0 features plus framework benefits

### From v2.0
- **Simplified Architecture**: No ROS2 complexity
- **Better Reliability**: Fewer dependencies and failure points
- **Easier Maintenance**: Clear, understandable codebase
- **Preserved Functionality**: All working features maintained

## 📚 Documentation

### For Users
- [Installation Guide](../user_guide/installation.md)
- [Quick Start Guide](../user_guide/quick_start.md)
- [Configuration Guide](../user_guide/configuration.md)
- [Troubleshooting](../user_guide/troubleshooting.md)

### For Developers
- [Getting Started](../developer_guide/getting_started.md)
- [Creating Nodes](../developer_guide/creating_nodes.md)
- [API Reference](../api_reference/README.md)
- [Testing Guide](../developer_guide/testing_guide.md)

### For System Administrators
- [Deployment Guide](../deployment/installation_guide.md)
- [Monitoring Guide](../deployment/monitoring.md)
- [Security Guide](../deployment/security.md)
- [Backup and Recovery](../deployment/backup_recovery.md)

## 🤝 Contributing

We welcome contributions to Nevil v3.0! Please see our [Contributing Guide](../developer_guide/contributing.md) for details on:

- Code style and standards
- Testing requirements
- Documentation standards
- Pull request process

## 📄 License

[License information to be added]

## 🙏 Acknowledgments

- **v1.0 Foundation**: The proven audio pipeline and core functionality
- **SunFounder**: PiCar-X hardware platform
- **OpenAI**: API services for speech and AI processing
- **Python Community**: Excellent libraries and tools
- **Contributors**: Everyone who helped make this possible

## 📞 Support

- **Documentation**: Complete specification in this folder
- **Issues**: GitHub issues for bug reports and feature requests
- **Discussions**: GitHub discussions for questions and ideas
- **Community**: Join our community for support and collaboration

---

## Quick Reference

### Essential Commands
```bash
./nevil start          # Start the system
./nevil stop           # Stop the system
./nevil status         # Check system status
./nevil logs           # View system logs
./nevil validate       # Validate configuration
```

### Key Files
- `.nodes` - Main system configuration
- `nodes/*/messages` - Node message interfaces
- `logs/system.log` - Main system log
- `.env` - Environment variables

### Important Concepts
- **Nodes**: Autonomous processes with specific responsibilities
- **Messages**: Typed communication between nodes
- **Topics**: Named channels for message delivery
- **Configuration**: YAML-based system and node setup
- **Logging**: Structured, multi-level logging system

---

*Nevil v3.0 - Where simplicity meets reliability* 🤖