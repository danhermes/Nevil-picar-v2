# Nevil v3.0 Technical Architecture Specification

## Executive Summary

Nevil v3.0 is a lightweight, custom framework designed to replace the complex ROS2 architecture of v2.0 with a simple, elegant, and maintainable solution. The framework preserves the proven working audio approaches from v1.0 while implementing a clean node-based architecture with threading, messaging, and launch capabilities.

**Core Philosophy**: "Simple architecture = working robot"

## Comprehensive System Overview

Nevil v3.0 represents a complete architectural redesign focused on declarative configuration, process isolation, and bounded resource management. The system automatically discovers, creates, and manages robotic components through YAML configuration files, eliminating complex manual setup while ensuring reliable, predictable operation.

### Core Architecture Philosophy

**Declarative Everything**: Rather than imperative programming, Nevil v3.0 uses declarative configuration files (`.nodes` and `.messages`) to define system behavior, node relationships, and communication patterns. This approach ensures predictable, reproducible deployments and eliminates the complexity of manual node management.

**Configuration-Driven Discovery**: The system automatically discovers individual node configurations and builds dependency graphs, startup sequences, and communication topologies without hardcoded relationships. This enables truly modular robotics development where components can be added, removed, or modified through simple configuration changes.

**Process Isolation with Message Passing**: Each node runs in its own isolated Python process, communicating through a robust message bus. This design provides fault isolation, independent scaling, and clear separation of concerns while maintaining high-performance inter-node communication.

**Bounded Resource Management**: The system implements strict resource controls including log file limits (`KEEP_LOG_FILES=3`), memory bounds, and predictable disk usage, ensuring the robot operates reliably within known resource constraints.

**Real-world robot action with no Simulations**: The system is at all times real-world, IRL, keepin it real, just actual robot audio, real robot motion, and actual reality with all the features.  NO SIMULATIONS. EVER. UNDER ANY CIRCUMSTANCES. Nevil v2 used simulations. This new and improved Nevil v3 WILL NOT. THANK YOU.

### System Components Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              Nevil v3.0 System Architecture                    │
├─────────────────────────────────────────────────────────────────────────────────┤
│  CLI Interface (./nevil)                                                       │
│  ├── start [--logscope] - Launch system with optional monitoring              │
│  ├── stop - Graceful shutdown with cleanup                                     │
│  ├── status - System health and component status                               │
│  ├── monitor - Launch LogScope GUI dashboard                                   │
│  └── validate - Configuration validation and testing                           │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Declarative Launch System                                                     │
│  ├── Node Discovery Engine - Auto-discovers .nodes files                      │
│  ├── Dependency Resolver - Topological sorting for startup order              │
│  ├── Process Manager - Isolated node process management                        │
│  ├── Health Monitor - Real-time system and node health tracking               │
│  └── Config Watcher - Hot-reload configuration changes                         │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Message Bus & Communication                                                   │
│  ├── Inter-Process Message Passing - High-performance node communication      │
│  ├── Topic-Based Routing - Declarative publish/subscribe from .messages       │
│  ├── Message Queuing - Reliable delivery with backpressure handling           │
│  └── Health Heartbeats - Automatic node status monitoring                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Node Processes (Auto-Created from .nodes files)                              │
│  ├── Speech Recognition Node - Audio input and OpenAI Whisper integration     │
│  ├── Speech Synthesis Node - Text-to-speech with OpenAI voices                │
│  ├── AI Cognition Node - OpenAI GPT integration with conversation management  │
│  └── [Additional Nodes] - Dynamically discovered and created                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Logging & Monitoring (KEEP_LOG_FILES=3 Strict Limits)                        │
│  ├── Enhanced Logging - EST timestamps, color coding, node identification     │
│  ├── LogScope GUI Dashboard - Real-time monitoring with filtering/search      │
│  ├── Terminal Log Viewer - Basic command-line log monitoring                  │
│  └── Strict File Rotation - Bounded disk usage with automatic cleanup         │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Configuration Layer                                                           │
│  ├── Root .nodes - System-wide configuration and overrides                    │
│  ├── Individual .nodes - Per-node configuration and dependencies              │
│  ├── .messages Files - Declarative communication interfaces                   │
│  └── Environment Variables - Runtime configuration and API keys               │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Operational Model

**Startup Flow**:
1. **Configuration Discovery** - Scan for `.nodes` files and validate syntax/semantics
2. **Dependency Resolution** - Build startup order using topological sort of dependencies
3. **System Initialization** - Start message bus, health monitor, enhanced logging system
4. **Node Creation** - Declaratively create nodes from individual `.nodes` configurations
5. **Process Launch** - Start node processes in dependency order with isolation
6. **Health Verification** - Wait for all nodes to report healthy status within timeout
7. **Monitoring Loop** - Continuous health monitoring and automatic failure recovery

**Runtime Operation**:
- Nodes communicate through the message bus using topics defined in `.messages` files
- Health monitor tracks CPU, memory, error rates, and communication patterns
- Failed nodes are automatically restarted according to their restart policies
- Log rotation maintains bounded disk usage with automatic file cleanup
- Configuration changes can trigger hot-reloads without full system restart
- LogScope dashboard provides real-time monitoring with advanced filtering

**Shutdown Flow**:
- Graceful shutdown signals sent to all nodes with configurable timeout
- Nodes complete current operations and clean up resources
- Message bus shutdown with pending message delivery
- Log file finalization and resource cleanup
- Process termination with force-kill fallback for unresponsive nodes

**Resource Management**:
- **Log Files**: Strict `KEEP_LOG_FILES=3` limit (current + 2 backups per type)
- **Memory**: Per-node monitoring with alerting and automatic restart policies
- **CPU**: Process-level resource tracking and bottleneck identification
- **Disk**: Predictable, bounded storage with automatic cleanup
- **Network**: Message bus with backpressure handling and performance monitoring

This architecture ensures that Nevil v3.0 robots are reliable, maintainable, and operate predictably across different deployment environments while providing comprehensive monitoring and debugging capabilities through the integrated LogScope dashboard and enhanced logging system.

## 1. System Architecture Overview

### 1.1 Design Principles

- **Simplicity First**: Minimal dependencies, clear interfaces
- **Proven Components**: Preserve working v1.0 audio pipeline
- **Autonomous Nodes**: Self-contained processes with clear responsibilities
- **Configuration-Driven**: Declarative setup via simple config files
- **Thread-Safe**: Proper concurrency without complexity
- **Maintainable**: Easy to understand, debug, and extend

### 1.2 Architecture Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Speech Recognition│ │ Speech Synthesis│ │  AI Cognition   ││
│  │      Node         │ │      Node       │ │      Node       ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                   Framework Layer                           │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │   Node Manager  │ │ Message System  │ │ Logging System  ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                   Hardware Layer                            │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │   Audio I/O     │ │   PiCar-X       │ │   Sensors       ││
│  │  (v1.0 proven) │ │   Hardware      │ │                 ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

## 2. Core Components

### 2.1 Framework Core

#### 2.1.1 Node Manager
- **Purpose**: Launch, monitor, and manage node lifecycle
- **Responsibilities**:
  - Parse `.nodes` configuration file
  - Launch nodes as separate processes
  - Monitor node health and restart if needed
  - Handle graceful shutdown
  - Manage node communication channels

#### 2.1.2 Message System
- **Purpose**: Inter-node communication
- **Implementation**: Simple publish/subscribe using Python multiprocessing
- **Features**:
  - Type-safe message definitions via `.messages` files
  - Asynchronous message delivery
  - Message queuing and buffering
  - Topic-based routing

#### 2.1.3 Logging System
- **Purpose**: Centralized logging with multiple levels
- **Levels**: info, debug, warning, error
- **Outputs**: Console + individual node log files
- **Features**:
  - Structured logging with timestamps
  - Node-specific log files
  - Configurable log levels per node
  - Log rotation and cleanup

### 2.2 Node Architecture

#### 2.2.1 Base Node Class
```python
class NevilNode:
    def __init__(self, node_name, config_path=None):
        self.node_name = node_name
        self.config_path = config_path or f"nodes/{node_name}/.messages"
        self.logger = self._setup_logging()
        self.message_bus = self._setup_messaging()
        self.running = False
        
        # Declarative messaging setup
        self.init_messages()
        
    def init_messages(self):
        # Read .messages file and automatically set up all publishers and subscribers
        # This runs once during __init__ for the life of the node
        
    def start(self):
        # Node initialization and main loop
        
    def stop(self):
        # Graceful shutdown
        
    # No separate publish/subscribe methods needed - all setup declaratively
```

#### 2.2.2 Node Directory Structure
```
nodes/
├── speech_recognition/
│   ├── __init__.py
│   ├── speech_recognition_node.py
│   └── .messages
├── speech_synthesis/
│   ├── __init__.py
│   ├── speech_synthesis_node.py
│   └── .messages
└── ai_cognition/
    ├── __init__.py
    ├── ai_cognition_node.py
    └── .messages
```

## 3. Configuration System

### 3.1 .nodes File Format (Root Level)

```yaml
# .nodes - Node configuration and status
nodes:
  speech_recognition:
    status: live          # live | muted | disabled
    priority: high        # high | medium | low
    restart_policy: always # always | on_failure | never
    environment:
      OPENAI_API_KEY: ${OPENAI_API_KEY}
    
  speech_synthesis:
    status: live
    priority: high
    restart_policy: always
    environment:
      OPENAI_API_KEY: ${OPENAI_API_KEY}
      TTS_VOICE: onyx
    
  ai_cognition:
    status: live
    priority: medium
    restart_policy: on_failure
    environment:
      OPENAI_API_KEY: ${OPENAI_API_KEY}
      OPENAI_ASSISTANT_ID: ${OPENAI_ASSISTANT_ID}

launch:
  startup_delay: 2.0      # seconds between node launches
  shutdown_timeout: 10.0  # seconds to wait for graceful shutdown
  health_check_interval: 5.0  # seconds between health checks
```

### 3.2 .messages File Format (Per Node)

```yaml
# nodes/speech_recognition/.messages
publishes:
  - topic: "prompt_audio"
    message_type: "PromptAudio"
    schema:
      text: str
      confidence: float
      audio_data: Optional[bytes]
      audio_format: str
      sample_rate: int
      language: str
      timestamp: float
      source: str
      
  - topic: "listening_status"
    message_type: "ListeningStatus"
    schema:
      listening: bool
      paused: bool
      wake_word_mode: bool
      reason: str
      timestamp: float
      
  - topic: "audio_level"
    message_type: "AudioLevel"
    schema:
      level: float
      timestamp: float

subscribes:
  - topic: "speaking_status"
    message_type: "SpeakingStatus"
    callback: "on_speaking_status_change"
    
  - topic: "system_mode"
    message_type: "SystemMode"
    callback: "on_system_mode_change"
    
  - topic: "audio_config"
    message_type: "AudioConfig"
    callback: "on_audio_config_change"
```

```yaml
# nodes/ai_cognition/.messages
publishes:
  - topic: "text_response"
    message_type: "TextResponse"
    schema:
      text: str
      voice: str
      priority: int
      speed: float
      volume: float
      timestamp: float
      response_id: str
      source_command: str
      
  - topic: "system_mode"
    message_type: "SystemMode"
    schema:
      mode: str
      reason: str
      timestamp: float

subscribes:
  - topic: "prompt_audio"
    message_type: "PromptAudio"
    callback: "on_voice_command"
    
  - topic: "text_command"
    message_type: "TextCommand"
    callback: "on_text_command"
    
  - topic: "system_heartbeat"
    message_type: "NodeStatus"
    callback: "on_system_heartbeat"
```

```yaml
# nodes/speech_synthesis/.messages
publishes:
  - topic: "speaking_status"
    message_type: "SpeakingStatus"
    schema:
      speaking: bool
      text: str
      voice: str
      speech_id: str
      priority: int
      timestamp: float

subscribes:
  - topic: "text_response"
    message_type: "TextResponse"
    callback: "on_text_response"
    
  - topic: "speech_control"
    message_type: "SpeechControl"
    callback: "on_speech_control"
    
  - topic: "audio_config"
    message_type: "AudioConfig"
    callback: "on_audio_config_change"
```

## 4. Threading Model

### 4.1 Process-Based Architecture
- Each node runs in its own Python process
- Inter-process communication via multiprocessing queues
- Shared memory for high-frequency data (sensor readings)
- Process isolation prevents cascading failures

### 4.2 Intra-Node Threading
```python
class NevilNode:
    def __init__(self):
        self.main_thread = None
        self.message_thread = None
        self.worker_threads = []
        
    def start(self):
        # Main processing thread
        self.main_thread = threading.Thread(target=self._main_loop)
        
        # Message handling thread
        self.message_thread = threading.Thread(target=self._message_loop)
        
        # Start threads
        self.main_thread.start()
        self.message_thread.start()
```

### 4.3 Thread Safety
- Thread-safe message queues
- Mutex protection for shared resources
- Lock-free data structures where possible
- Proper cleanup on shutdown

## 5. Launch System Design

### 5.1 Launch Process
```python
class NevilLauncher:
    def __init__(self, nodes_config_path=".nodes"):
        self.config = self._load_config(nodes_config_path)
        self.processes = {}
        
    def launch_all(self):
        for node_name, node_config in self.config['nodes'].items():
            if node_config['status'] == 'live':
                self._launch_node(node_name, node_config)
                
    def _launch_node(self, node_name, config):
        # Launch node as subprocess
        # Set up monitoring
        # Configure environment variables
```

### 5.2 Health Monitoring
- Periodic health checks via heartbeat messages
- Automatic restart on failure (based on restart_policy)
- Resource usage monitoring (CPU, memory)
- Log analysis for error detection

### 5.3 Graceful Shutdown
- SIGTERM signal handling
- Cleanup timeouts
- Resource deallocation
- Log file finalization

## 6. Logging Architecture

### 6.1 Log Structure (KEEP_LOG_FILES=3 Strict Limit)
```
logs/
├── system.log                      # Current system log
├── system.log.1                    # Previous system log
├── system.log.2                    # Oldest system log (deleted on next rotation)
├── speech_recognition.log          # Current speech recognition log
├── speech_recognition.log.1        # Previous speech recognition log
├── speech_recognition.log.2        # Oldest speech recognition log
├── speech_synthesis.log            # Current speech synthesis log
├── speech_synthesis.log.1          # Previous speech synthesis log
├── speech_synthesis.log.2          # Oldest speech synthesis log
├── ai_cognition.log               # Current AI cognition log
├── ai_cognition.log.1             # Previous AI cognition log
└── ai_cognition.log.2             # Oldest AI cognition log

# MAXIMUM: 12 total files (3 per log type × 4 log types)
# ROTATION: .log → .log.1 → .log.2 → DELETE
# ENFORCEMENT: KEEP_LOG_FILES=3 constant prevents log bloat
```

### 6.2 Log Format

#### 6.2.1 Enhanced Readability Format (v3.0)
```
[EST_TIMESTAMP EST] [LEVEL    ] [NODE_TYPE      ] [COMPONENT          ] MESSAGE
[2025-09-14 14:30:15 EST] [INFO    ] [speech_recognition] [audio_capture      ] Audio device initialized
[2025-09-14 14:30:16 EST] [DEBUG   ] [speech_recognition] [microphone_handler ] Sensitivity adjusted to 300
[2025-09-14 14:30:17 EST] [ERROR   ] [speech_synthesis  ] [openai_tts_client  ] OpenAI API timeout after 30s
[2025-09-14 14:30:18 EST] [WARNING ] [ai_cognition      ] [conversation_mgr   ] Context window approaching limit
[2025-09-14 14:30:19 EST] [INFO    ] [system            ] [node_manager       ] All nodes healthy, uptime: 1h 23m
```

#### 6.2.2 Color Coding Scheme
- **Log Levels**:
  - DEBUG (Cyan), INFO (Green), WARNING (Yellow), ERROR (Red), CRITICAL (Bold Magenta)
- **Node Types**:
  - speech_recognition (Light Cyan), speech_synthesis (Light Green)
  - ai_cognition (Light Magenta), system (Blue)
  - audio_interface (Light Yellow), sensor_interface (Light White)

#### 6.2.3 System.log Node Type Color Coding
System logs automatically detect and color-code entries by source node:
```
[2025-09-14 14:30:20 EST] [INFO    ] [speech_recognition] [system             ] Node startup completed
[2025-09-14 14:30:21 EST] [WARNING ] [speech_synthesis  ] [system             ] High CPU usage detected
[2025-09-14 14:30:22 EST] [ERROR   ] [ai_cognition      ] [system             ] API rate limit exceeded
```

### 6.3 Log Management
- **Strict Log Retention**: Keep only current file + last 2 archived files (`KEEP_LOG_FILES=3`)
- **Automatic Pruning**: System enforces 3-file limit for both system and node logs
- **Log Rotation**: Size-based rotation (100MB default) with immediate old file cleanup
- **Storage Control**: Maximum 9 total log files (3 system + 3 per node × 2 nodes = 9 files)
- **Real-time Streaming**: Live log monitoring for debugging

## 7. Audio Integration Strategy

### 7.1 Preserve v1.0 Working Components

#### 7.1.1 Audio Output (Speech Synthesis)
```python
# Preserve proven v1.0 audio pipeline
class AudioOutput:
    def __init__(self):
        # Use exact v1.0 approach
        from robot_hat import Music, TTS
        self.music = Music()
        self.tts = TTS()
        
    def speak_text(self, text, voice="onyx"):
        # OpenAI TTS → MP3 → robot_hat.Music → HiFiBerry DAC
        # Exact v1.0 pipeline that works
```

#### 7.1.2 Audio Input (Speech Recognition)
```python
# Preserve proven v1.0 microphone handling
class AudioInput:
    def __init__(self):
        # Use exact v1.0 speech recognition setup
        import speech_recognition as sr
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=1)  # USB PnP Sound Device
        
        # v1.0 proven parameters
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold = 0.5
        self.recognizer.dynamic_energy_adjustment_damping = 0.1
```

### 7.2 Audio Hardware Configuration
- Preserve `.asoundrc` configuration from v1.0
- Use HiFiBerry DAC (card index 3) for output
- Use USB microphone (device index 1) for input
- Maintain ALSA-only approach (no PulseAudio/PipeWire)

## 8. Initial Node Specifications

### 8.1 Speech Recognition Node
- **Purpose**: Convert voice input to text commands
- **Input**: Microphone audio stream
- **Output**: Text commands with confidence scores
- **Key Features**:
  - Continuous listening with voice activity detection
  - OpenAI Whisper integration
  - Noise filtering and ambient adjustment
  - Wake word detection

### 8.2 Speech Synthesis Node
- **Purpose**: Convert text responses to speech output
- **Input**: Text messages for speech
- **Output**: Audio playback through speakers
- **Key Features**:
  - OpenAI TTS integration
  - Voice selection and rate control
  - Audio queue management
  - Volume and quality control

### 8.3 AI Cognition Node
- **Purpose**: Process commands and generate responses
- **Input**: Text commands from speech recognition
- **Output**: Text responses for speech synthesis
- **Key Features**:
  - OpenAI GPT integration
  - Context and conversation management
  - Action planning and execution
  - Fallback to local models when offline

## 9. Message Flow Architecture

### 9.1 Core Message Types

#### 9.1.1 Audio-AI Workflow Messages
```python
@dataclass
class PromptAudio:
    """Audio data with recognition results for AI processing"""
    text: str                    # Recognized text from speech
    confidence: float           # Recognition confidence (0.0-1.0)
    audio_data: Optional[bytes] # Raw audio data (optional)
    audio_format: str = "wav"   # Audio format if audio_data provided
    sample_rate: int = 16000    # Audio sample rate
    language: str = "en-US"     # Recognition language
    timestamp: float            # Recognition timestamp
    source: str = "speech_recognition"

@dataclass
class TextResponse:
    """Text response from AI for speech synthesis"""
    text: str                   # Text to be spoken
    voice: str = "onyx"        # Voice identifier for TTS
    priority: int = 100        # Speech priority (lower = higher priority)
    speed: float = 1.0         # Speaking speed multiplier
    volume: float = 0.8        # Volume level (0.0-1.0)
    timestamp: float           # Response generation timestamp
    response_id: str           # Unique response identifier
    source_command: str = ""   # Original command that triggered this response

@dataclass
class VoiceCommand:
    """Legacy voice command format (for backward compatibility)"""
    text: str
    confidence: float
    timestamp: float
    source: str = "speech_recognition"
```

#### 9.1.2 System Control Messages
```python
@dataclass
class SystemMode:
    mode: str  # "listening", "speaking", "thinking", "idle"
    reason: str = ""           # Reason for mode change
    timestamp: float

@dataclass
class SpeakingStatus:
    """Status of speech synthesis"""
    speaking: bool             # Currently speaking
    text: str = ""            # Text being spoken (if speaking)
    voice: str = ""           # Voice being used
    speech_id: str = ""       # Unique speech identifier
    priority: int = 100       # Speech priority
    timestamp: float

@dataclass
class ListeningStatus:
    """Status of speech recognition"""
    listening: bool           # Currently listening
    paused: bool = False     # Temporarily paused
    wake_word_mode: bool = False  # In wake word detection mode
    reason: str = ""         # Reason for status change
    timestamp: float

@dataclass
class AudioLevel:
    """Audio level information for UI feedback"""
    level: float             # Audio level (0.0-1.0)
    timestamp: float
```

#### 9.1.3 Node Management Messages
```python
@dataclass
class NodeStatus:
    node_name: str
    status: str  # "running", "error", "stopped", "starting"
    cpu_usage: float
    memory_usage: float
    uptime: float            # Node uptime in seconds
    error_count: int = 0     # Number of errors since start
    timestamp: float

@dataclass
class AudioConfig:
    """Audio configuration updates"""
    energy_threshold: Optional[int] = None      # Microphone sensitivity
    pause_threshold: Optional[float] = None     # Pause detection threshold
    confidence_threshold: Optional[float] = None # Recognition confidence threshold
    volume: Optional[float] = None              # Default volume
    voice: Optional[str] = None                 # Default voice
    speed: Optional[float] = None               # Default speaking speed
    timestamp: float

@dataclass
class SpeechControl:
    """Speech synthesis control commands"""
    command: str             # "stop", "pause", "resume", "clear_queue", "set_voice"
    voice: Optional[str] = None      # For set_voice command
    timestamp: float
```

### 9.2 Message Flow Diagram

#### 9.2.1 Primary Audio-AI Workflow
```
┌─────────────────┐    prompt_audio     ┌─────────────────┐    text_response    ┌─────────────────┐
│ Speech          │ ──────────────────> │ AI Cognition    │ ──────────────────> │ Speech          │
│ Recognition     │                     │ Node            │                     │ Synthesis       │
│ Node            │                     │                 │                     │ Node            │
└─────────────────┘                     └─────────────────┘                     └─────────────────┘
         │                                       │                                       │
         │ listening_status                      │ (processing)                          │ speaking_status
         │ audio_level                           │                                       │
         v                                       v                                       v
┌─────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Message Bus                                                      │
│  Topics: prompt_audio, text_response, speaking_status, listening_status,                           │
│          audio_level, system_mode, audio_config, speech_control, node_status                      │
└─────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

#### 9.2.2 Detailed Message Flow Sequence
```
1. Speech Recognition detects voice input
   ├─> Publishes: listening_status (listening=true)
   ├─> Publishes: audio_level (for UI feedback)
   └─> Processes audio and publishes: prompt_audio

2. AI Cognition receives prompt_audio
   ├─> Publishes: system_mode (mode="thinking")
   ├─> Processes command with OpenAI API
   └─> Publishes: text_response

3. Speech Synthesis receives text_response
   ├─> Publishes: speaking_status (speaking=true)
   ├─> Synthesizes and plays audio
   └─> Publishes: speaking_status (speaking=false)

4. Speech Recognition receives speaking_status
   ├─> Pauses listening when speaking=true
   └─> Resumes listening when speaking=false
```

#### 9.2.3 Control and Status Messages
```
┌─────────────────┐                    ┌─────────────────┐                    ┌─────────────────┐
│ Speech          │                    │ AI Cognition    │                    │ Speech          │
│ Recognition     │                    │ Node            │                    │ Synthesis       │
└─────────────────┘                    └─────────────────┘                    └─────────────────┘
         │                                       │                                       │
         │ ←─────────────── audio_config ────────────────────────────────────────────── │
         │ ←─────────────── speech_control ──────────────────────────────────────────── │
         │ ←─────────────── system_mode ─────────────────────────────────────────────── │
         │                                       │                                       │
         └─────────────────── node_status ──────┼─────────────── node_status ──────────┘
                                                 │
                                                 └─────────────── node_status
```

## 10. Performance Requirements

### 10.1 Latency Targets
- Voice command recognition: < 2 seconds
- Text-to-speech generation: < 3 seconds
- Inter-node message delivery: < 100ms
- Node startup time: < 5 seconds
- System shutdown time: < 10 seconds

### 10.2 Resource Constraints
- Memory usage per node: < 200MB
- CPU usage per node: < 25% (single core)
- Disk space for logs: < 1GB (with rotation)
- Network bandwidth: Minimal (local IPC only)

### 10.3 Reliability Targets
- Node uptime: > 99% (with automatic restart)
- Message delivery: > 99.9% success rate
- Audio quality: No dropouts or distortion
- System recovery: < 30 seconds after failure

## 11. Development and Testing Strategy

### 11.1 Implementation Phases
1. **Phase 1**:
   - Core framework and message system
   - Basic node implementation and launch system
   - Audio integration with v1.0 components

2. **Phase 2**:
   - Advanced features and optimization
   - Enhanced monitoring and analytics
   - Performance optimization and scalability

**Phase 2 Documentation:**
See the `phase 2/` folder for comprehensive Phase 2 feature documentation including advanced monitoring, optimization, and enhanced capabilities.

### 11.2 Testing Approach
- Unit tests for each framework component
- Integration tests for node communication
- Audio hardware tests with real devices
- Performance benchmarking
- Stress testing with multiple nodes

### 11.3 Migration Strategy
- Gradual migration from v2.0 ROS2 system
- Parallel operation during transition
- Fallback mechanisms to v1.0 components
- Comprehensive testing before full deployment

## 12. Security and Reliability

### 12.1 Security Considerations
- Environment variable protection for API keys
- Process isolation between nodes
- Input validation for all messages
- Secure temporary file handling
- Network security (if remote access added)

### 12.2 Error Handling
- Graceful degradation on component failure
- Automatic retry mechanisms
- Fallback to offline modes
- Comprehensive error logging
- User-friendly error messages

### 12.3 Monitoring and Diagnostics
- Real-time system health dashboard
- Performance metrics collection
- Automated alerting on failures
- Remote diagnostics capabilities
- Log analysis and reporting

## 13. Future Extensibility

### 13.1 Plugin Architecture
- Standardized node interface for easy extension
- Dynamic node loading and unloading
- Configuration-driven feature enablement
- Third-party node integration support

### 13.2 Scalability Considerations
- Horizontal scaling with multiple processes
- Load balancing for high-traffic scenarios
- Distributed deployment capabilities
- Cloud integration options

### 13.3 Maintenance and Updates
- Hot-swappable node updates
- Configuration changes without restart
- Automated backup and restore
- Version management and rollback

## Conclusion

Nevil v3.0 represents a return to simplicity while maintaining the sophisticated capabilities required for an intelligent robotic companion. By preserving the proven working components from v1.0 and implementing a clean, maintainable architecture, we achieve the goal of "simple architecture = working robot" while providing a solid foundation for future enhancements.

The framework prioritizes reliability, maintainability, and ease of understanding over complex abstractions, ensuring that the system remains debuggable and extensible as requirements evolve.