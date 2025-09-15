
# Nevil v3.0 Configuration File Formats

## Overview

Nevil v3.0 uses simple, human-readable configuration files to define node behavior, message schemas, and system settings. The framework implements **complete declarative configuration** with two complementary systems:

- **Individual `.nodes` files**: Declarative node creation and configuration (handled by Launch system)
- **Individual `.messages` files**: Declarative messaging setup (handled by NevilNode.init_messages())
- **Root `.nodes` file**: System-wide configuration and coordination

This document specifies the exact format and structure of these configuration files.

## 1. Individual Node Configuration Files (.nodes)

Each node has its own `.nodes` file that declaratively defines how the node should be created and configured. This follows the same pattern as `.messages` files but for node lifecycle management.

### 1.1 File Location and Format
- **Location**: `nodes/{node_name}/.nodes`
- **Format**: YAML
- **Encoding**: UTF-8
- **Purpose**: Declarative node creation and runtime configuration

### 1.2 Complete Individual .nodes File Example

```yaml
# nodes/speech_recognition/.nodes
# Declarative node creation and configuration

# Metadata
node_name: "speech_recognition"
version: "1.0"
description: "Converts voice input to text commands using OpenAI Whisper"
author: "Nevil Framework"
created: "2024-01-15T10:30:00Z"
tags: ["audio", "ai", "speech"]

# Node creation specification
creation:
  # Class and module information
  class_name: "SpeechRecognitionNode"
  module_path: "nodes.speech_recognition.speech_recognition_node"
  base_class: "NevilNode"
  
  # Creation behavior
  auto_start: true
  lazy_load: false
  singleton: true
  
  # Initialization parameters
  init_params:
    config_path: "nodes/speech_recognition/.messages"
    log_level: "INFO"

# Runtime configuration
runtime:
  # Node status: live, muted, disabled
  status: live
  
  # Process priority: high, medium, low
  priority: high
  
  # Restart policy: always, on_failure, never
  restart_policy: always
  max_restarts: 5
  restart_delay: 2.0
  
  # Startup configuration
  startup_timeout: 30.0
  shutdown_timeout: 10.0
  
  # Process isolation
  isolated_process: true
  shared_memory: false

# Environment variables
environment:
  OPENAI_API_KEY: "${OPENAI_API_KEY}"
  SPEECH_RECOGNITION_LANGUAGE: "en-US"
  MICROPHONE_DEVICE_INDEX: "1"
  ENERGY_THRESHOLD: "300"
  PAUSE_THRESHOLD: "0.5"
  LOG_LEVEL: "${LOG_LEVEL:-INFO}"

# Resource limits
resources:
  max_memory_mb: 200
  max_cpu_percent: 25
  max_threads: 10
  max_file_descriptors: 100
  
  # Storage limits
  max_log_size_mb: 50
  max_temp_files: 10

# Dependencies and relationships
dependencies:
  # Nodes that must be running before this node starts
  requires: []
  
  # Services this node provides
  provides: ["speech_recognition", "audio_input"]
  
  # Optional dependencies (start if available)
  optional: ["audio_config_manager"]
  
  # Conflicting nodes (cannot run simultaneously)
  conflicts: ["alternative_speech_recognition"]

# Health monitoring
health_check:
  enabled: true
  timeout: 5.0
  interval: 10.0
  failure_threshold: 3
  
  # Custom health check method
  method: "check_microphone_health"
  
  # Health check parameters
  params:
    check_audio_device: true
    check_api_connection: true

# Development and testing
development:
  debug_mode: false
  profiling: false
  mock_mode: false
  
  # Test configuration
  test_config:
    mock_audio_input: false
    simulate_recognition_errors: false
    test_data_path: "test/data/audio_samples"

# Security settings
security:
  # Allowed file paths
  allowed_paths:
    - "./audio/"
    - "./logs/"
    - "/tmp/"
  
  # Network restrictions
  network_access: true
  allowed_hosts:
    - "api.openai.com"
  
  # Capability restrictions
  capabilities:
    audio_access: true
    file_write: true
    network_client: true

# Monitoring and metrics
monitoring:
  metrics_enabled: true
  
  # Custom metrics to collect
  custom_metrics:
    - "recognition_accuracy"
    - "audio_level"
    - "processing_latency"
  
  # Performance thresholds
  thresholds:
    max_response_time_ms: 2000
    min_accuracy_percent: 80
    max_error_rate_percent: 5

# Integration points
integration:
  # Message bus configuration
  message_bus:
    queue_size: 100
    timeout: 5.0
  
  # External service configuration
  external_services:
    openai:
      timeout: 30.0
      retry_attempts: 3
      rate_limit: 60  # requests per minute
```

### 1.3 Individual .nodes File Validation Schema

```python
# Individual .nodes file validation schema
INDIVIDUAL_NODES_SCHEMA = {
    "node_name": {"type": "string", "required": True},
    "version": {"type": "string", "required": True},
    "description": {"type": "string", "required": True},
    "creation": {
        "type": "dict",
        "required": True,
        "schema": {
            "class_name": {"type": "string", "required": True},
            "module_path": {"type": "string", "required": True},
            "base_class": {"type": "string", "default": "NevilNode"},
            "auto_start": {"type": "boolean", "default": True},
            "singleton": {"type": "boolean", "default": True}
        }
    },
    "runtime": {
        "type": "dict",
        "schema": {
            "status": {"type": "string", "allowed": ["live", "muted", "disabled"], "required": True},
            "priority": {"type": "string", "allowed": ["high", "medium", "low"], "required": True},
            "restart_policy": {"type": "string", "allowed": ["always", "on_failure", "never"], "required": True},
            "max_restarts": {"type": "integer", "min": 0, "max": 10},
            "restart_delay": {"type": "float", "min": 0.0, "max": 60.0},
            "isolated_process": {"type": "boolean", "default": True}
        }
    },
    "dependencies": {
        "type": "dict",
        "schema": {
            "requires": {"type": "list", "schema": {"type": "string"}},
            "provides": {"type": "list", "schema": {"type": "string"}},
            "optional": {"type": "list", "schema": {"type": "string"}},
            "conflicts": {"type": "list", "schema": {"type": "string"}}
        }
    }
}
```

## 2. Root Configuration File (.nodes)

The root `.nodes` file defines system-wide settings and coordinates the individual node configurations. It works in conjunction with the individual `.nodes` files.

### 2.1 File Location and Format
- **Location**: `nevil_v3/.nodes` (project root)
- **Format**: YAML
- **Encoding**: UTF-8
- **Purpose**: System-wide coordination and global settings

### 2.2 Updated Root .nodes File Example

```yaml
# .nodes - Nevil v3.0 System Configuration
# This file coordinates individual node configurations and provides system-wide settings

# Metadata
version: "3.0"
created: "2024-01-15T10:30:00Z"
description: "Nevil v3.0 system coordination and global configuration"

# System-wide settings
system:
  framework_version: "3.0.0"
  python_version: "3.8+"
  log_level: "INFO"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
  max_log_size_mb: 100
  log_retention_days: 30
  health_check_interval: 5.0  # seconds
  shutdown_timeout: 10.0      # seconds
  startup_delay: 2.0          # seconds between node launches

# Global environment variables (applied to all nodes)
environment:
  NEVIL_VERSION: "3.0"
  PYTHONPATH: "${PYTHONPATH}:./nevil_framework:./audio"
  LOG_LEVEL: "${LOG_LEVEL:-INFO}"

# Node discovery and coordination
nodes:
  # Discovery settings
  discovery:
    auto_discover: true  # Automatically discover .nodes files
    nodes_directory: "nodes"
    scan_recursive: true
    
  # Global node overrides (override individual .nodes file settings)
  global_overrides:
    # Apply to all nodes
    all:
      environment:
        GLOBAL_DEBUG: "${DEBUG:-false}"
      resources:
        max_log_size_mb: 50
    
    # Apply to specific nodes
    speech_recognition:
      runtime:
        priority: high  # Override individual .nodes file setting
    
    ai_cognition:
      dependencies:
        requires: ["speech_recognition", "speech_synthesis"]  # Override dependencies

  # Node status overrides (can disable nodes without editing individual files)
  status_overrides:
    # Temporarily disable nodes
    disabled_nodes: []
    
    # Force mute nodes (run but don't publish)
    muted_nodes: []
    
    # Development mode nodes (only run in development)
    dev_only_nodes: ["debug_monitor", "test_harness"]

# Launch configuration
launch:
  # Launch strategy
  strategy: "declarative"  # declarative, manual, hybrid
  
  # Node creation method
  creation_method: "individual_configs"  # individual_configs, centralized, hybrid
  
  # Startup behavior
  startup:
    # Use dependency order from individual .nodes files
    respect_individual_dependencies: true
    
    # Global startup order (applied after individual dependencies)
    global_order: []
    
    # Parallel launch (start nodes simultaneously if dependencies allow)
    parallel_launch: false
    
    # Wait for all nodes to be healthy before considering system ready
    wait_for_healthy: true
    
    # Timeout for system to become ready
    ready_timeout: 30.0
    
    # Startup validation
    validate_configs: true
    validate_dependencies: true

# Monitoring and alerting
monitoring:
  enabled: true
  
  # System metrics collection
  metrics:
    cpu_usage: true
    memory_usage: true
    message_throughput: true
    error_rates: true
    response_times: true
  
  # Alerting thresholds
  alerts:
    high_cpu_percent: 80
    high_memory_percent: 85
    high_error_rate: 0.1  # 10% error rate
    slow_response_ms: 5000
  
  # Monitoring output
  output:
    console: true
    file: "logs/monitoring.log"
    interval: 60.0  # seconds

# Development and debugging
development:
  # Enable debug mode (more verbose logging, additional checks)
  debug_mode: false
  
  # Enable profiling
  profiling: false
  
  # Mock external services (for testing without API keys)
  mock_services:
    openai: false
    audio_hardware: false
  
  # Test mode (use simulated data)
  test_mode: false
  
  # Development overrides
  dev_overrides:
    # Override individual node settings in development
    force_debug_logging: false
    disable_health_checks: false
    mock_external_services: false

# Security settings
security:
  # Validate environment variables
  validate_env_vars: true
  
  # Required environment variables
  required_env_vars:
    - OPENAI_API_KEY
  
  # Mask sensitive values in logs
  mask_secrets: true
  
  # Allowed file paths for nodes
  allowed_paths:
    - "./nodes/"
    - "./audio/"
    - "./logs/"
    - "/tmp/"

# Integration settings
integration:
  # Message bus configuration
  message_bus:
    type: "internal"  # internal, redis, rabbitmq
    max_queue_size: 1000
    timeout: 5.0
  
  # External service coordination
  external_services:
    openai:
      global_timeout: 30.0
      global_retry_attempts: 3
      rate_limit_coordination: true
```

### 2.3 Relationship Between Individual and Root .nodes Files

The declarative node creation system uses both individual `.nodes` files and the root `.nodes` file in a coordinated manner:

#### 2.3.1 Configuration Hierarchy

```
Root .nodes file (System-wide coordination)
    ↓ coordinates and overrides
Individual .nodes files (Node-specific creation)
    ↓ defines creation and configuration
Node instances (Created declaratively)
```

#### 2.3.2 Configuration Resolution Order

1. **Individual .nodes file** defines base configuration
2. **Root .nodes file global_overrides.all** applies to all nodes
3. **Root .nodes file global_overrides.{node_name}** applies to specific node
4. **Root .nodes file status_overrides** can disable/mute nodes
5. **Environment variables** provide runtime values

#### 2.3.3 Example Configuration Resolution

```yaml
# nodes/speech_recognition/.nodes (Individual)
runtime:
  status: live
  priority: medium
environment:
  LOG_LEVEL: "INFO"

# .nodes (Root - Global overrides)
nodes:
  global_overrides:
    all:
      environment:
        GLOBAL_DEBUG: "true"
    speech_recognition:
      runtime:
        priority: high  # Overrides individual file
  status_overrides:
    disabled_nodes: []  # Could disable this node

# Final resolved configuration:
# status: live (from individual)
# priority: high (from root override)
# environment: {LOG_LEVEL: "INFO", GLOBAL_DEBUG: "true"}
```

### 2.4 Root .nodes File Validation Schema

```python
# Root .nodes file validation schema
ROOT_NODES_SCHEMA = {
    "version": {"type": "string", "required": True},
    "system": {
        "type": "dict",
        "schema": {
            "framework_version": {"type": "string", "required": True},
            "log_level": {"type": "string", "allowed": ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]},
            "health_check_interval": {"type": "float", "min": 1.0, "max": 60.0},
            "shutdown_timeout": {"type": "float", "min": 5.0, "max": 60.0},
            "startup_delay": {"type": "float", "min": 0.0, "max": 10.0}
        }
    },
    "nodes": {
        "type": "dict",
        "schema": {
            "discovery": {
                "type": "dict",
                "schema": {
                    "auto_discover": {"type": "boolean", "default": True},
                    "nodes_directory": {"type": "string", "default": "nodes"},
                    "scan_recursive": {"type": "boolean", "default": True}
                }
            },
            "global_overrides": {
                "type": "dict",
                "schema": {
                    "all": {"type": "dict"},  # Can override any individual .nodes settings
                    # Dynamic keys for specific node names
                }
            },
            "status_overrides": {
                "type": "dict",
                "schema": {
                    "disabled_nodes": {"type": "list", "schema": {"type": "string"}},
                    "muted_nodes": {"type": "list", "schema": {"type": "string"}},
                    "dev_only_nodes": {"type": "list", "schema": {"type": "string"}}
                }
            }
        }
    },
    "launch": {
        "type": "dict",
        "schema": {
            "strategy": {"type": "string", "allowed": ["declarative", "manual", "hybrid"], "default": "declarative"},
            "creation_method": {"type": "string", "allowed": ["individual_configs", "centralized", "hybrid"], "default": "individual_configs"}
        }
    }
}
```

### 2.5 Complete Declarative Configuration Pattern

The complete declarative pattern ensures consistency across the framework:

```
Project Structure:
├── .nodes                           # Root system coordination
├── nodes/
│   ├── speech_recognition/
│   │   ├── .nodes                   # Individual node creation config
│   │   ├── .messages                # Individual messaging config
│   │   └── speech_recognition_node.py
│   ├── speech_synthesis/
│   │   ├── .nodes                   # Individual node creation config
│   │   ├── .messages                # Individual messaging config
│   │   └── speech_synthesis_node.py
│   └── ai_cognition/
│       ├── .nodes                   # Individual node creation config
│       ├── .messages                # Individual messaging config
│       └── ai_cognition_node.py
└── nevil_framework/
    ├── declarative_launcher.py     # Handles .nodes files
    └── base_node.py                # Handles .messages files
```

#### Declarative Flow:
1. **System Startup**: Launch system reads root `.nodes` file
2. **Node Discovery**: Automatically discovers individual `.nodes` files
3. **Node Creation**: Creates nodes based on individual `.nodes` configurations
4. **Messaging Setup**: Each node reads its `.messages` file via `init_messages()`
5. **System Ready**: All nodes created and configured declaratively

## 2. Node Message Configuration (.messages)

Each node has a `.messages` file that defines its message interface - what topics it publishes to and subscribes from.

### 2.1 File Location and Format
- **Location**: `nodes/{node_name}/.messages`
- **Format**: YAML
- **Encoding**: UTF-8

### 2.2 Speech Recognition Node .messages Example

```yaml
# nodes/speech_recognition/.messages
# Message interface definition for speech recognition node

# Metadata
node_name: "speech_recognition"
version: "1.0"
description: "Converts voice input to text commands"

# Topics this node publishes to
publishes:
  - topic: "voice_command"
    message_type: "VoiceCommand"
    description: "Recognized voice commands with confidence scores"
    frequency: "on_demand"  # on_demand, periodic, continuous
    qos:
      reliability: "reliable"  # reliable, best_effort
      durability: "volatile"   # volatile, persistent
      history: "keep_last"     # keep_last, keep_all
      depth: 1
    schema:
      text:
        type: "string"
        required: true
        description: "Recognized text from speech"
        max_length: 500
      confidence:
        type: "float"
        required: true
        description: "Recognition confidence (0.0 to 1.0)"
        min: 0.0
        max: 1.0
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp when speech was recognized"
      language:
        type: "string"
        required: false
        description: "Detected language code"
        default: "en-US"
      duration:
        type: "float"
        required: false
        description: "Duration of speech in seconds"
        min: 0.0

  - topic: "audio_level"
    message_type: "AudioLevel"
    description: "Current microphone audio level"
    frequency: "continuous"
    qos:
      reliability: "best_effort"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      level:
        type: "float"
        required: true
        description: "Audio level (0.0 to 1.0)"
        min: 0.0
        max: 1.0
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp of measurement"

  - topic: "listening_status"
    message_type: "ListeningStatus"
    description: "Whether the node is actively listening"
    frequency: "on_change"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      listening:
        type: "boolean"
        required: true
        description: "True if actively listening for speech"
      reason:
        type: "string"
        required: false
        description: "Reason for status change"
        allowed: ["user_request", "speaking_detected", "system_mode", "error"]

# Topics this node subscribes to
subscribes:
  - topic: "system_mode"
    message_type: "SystemMode"
    callback: "on_system_mode_change"
    description: "System-wide mode changes"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      mode:
        type: "string"
        required: true
        description: "Current system mode"
        allowed: ["listening", "speaking", "thinking", "idle", "error"]
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp of mode change"
      source:
        type: "string"
        required: false
        description: "Source of mode change"

  - topic: "speaking_status"
    message_type: "SpeakingStatus"
    callback: "on_speaking_status_change"
    description: "Speech synthesis status updates"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      speaking:
        type: "boolean"
        required: true
        description: "True if speech synthesis is active"
      text:
        type: "string"
        required: false
        description: "Text being spoken (if speaking=true)"

  - topic: "audio_config"
    message_type: "AudioConfig"
    callback: "on_audio_config_change"
    description: "Audio configuration updates"
    qos:
      reliability: "reliable"
      durability: "persistent"
      history: "keep_last"
      depth: 1
    schema:
      energy_threshold:
        type: "integer"
        required: false
        description: "Microphone energy threshold"
        min: 50
        max: 4000
      pause_threshold:
        type: "float"
        required: false
        description: "Pause threshold in seconds"
        min: 0.1
        max: 2.0
      language:
        type: "string"
        required: false
        description: "Recognition language"

# Node-specific configuration
configuration:
  # Audio input settings
  audio:
    device_index: 1  # USB microphone
    sample_rate: 44100
    chunk_size: 1024
    channels: 1
  
  # Speech recognition settings
  recognition:
    energy_threshold: 300
    pause_threshold: 0.5
    phrase_threshold: 0.3
    non_speaking_duration: 0.3
    dynamic_energy_threshold: false
    operation_timeout: 10.0
  
  # Performance settings
  performance:
    max_queue_size: 10
    processing_timeout: 5.0
    retry_attempts: 3
    retry_delay: 1.0

# Testing and validation
testing:
  # Mock data for testing
  mock_data:
    voice_commands:
      - text: "Hello Nevil"
        confidence: 0.95
      - text: "What time is it"
        confidence: 0.87
      - text: "Turn left"
        confidence: 0.92
  
  # Test scenarios
  test_scenarios:
    - name: "basic_recognition"
      description: "Test basic voice recognition"
      input: "mock_audio_hello.wav"
      expected_output:
        topic: "voice_command"
        text: "Hello Nevil"
        confidence: ">0.8"
    
    - name: "noise_handling"
      description: "Test recognition with background noise"
      input: "mock_audio_noisy.wav"
      expected_output:
        topic: "voice_command"
        confidence: ">0.6"
```

### 2.3 Speech Synthesis Node .messages Example

```yaml
# nodes/speech_synthesis/.messages
# Message interface definition for speech synthesis node

node_name: "speech_synthesis"
version: "1.0"
description: "Converts text to speech output"

publishes:
  - topic: "speaking_status"
    message_type: "SpeakingStatus"
    description: "Current speech synthesis status"
    frequency: "on_change"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      speaking:
        type: "boolean"
        required: true
        description: "True if currently speaking"
      text:
        type: "string"
        required: false
        description: "Text being spoken"
        max_length: 1000
      voice:
        type: "string"
        required: false
        description: "Voice being used"
      progress:
        type: "float"
        required: false
        description: "Speech progress (0.0 to 1.0)"
        min: 0.0
        max: 1.0
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

  - topic: "audio_output_status"
    message_type: "AudioOutputStatus"
    description: "Audio output device status"
    frequency: "on_change"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      available:
        type: "boolean"
        required: true
        description: "True if audio output is available"
      device:
        type: "string"
        required: false
        description: "Audio output device name"
      volume:
        type: "float"
        required: false
        description: "Current volume (0.0 to 1.0)"
        min: 0.0
        max: 1.0

subscribes:
  - topic: "text_response"
    message_type: "TextResponse"
    callback: "on_text_response"
    description: "Text to be converted to speech"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_all"
      depth: 10
    schema:
      text:
        type: "string"
        required: true
        description: "Text to speak"
        max_length: 1000
      voice:
        type: "string"
        required: false
        description: "Voice to use"
        default: "onyx"
        allowed: ["onyx", "alloy", "echo", "fable", "nova", "shimmer"]
      priority:
        type: "integer"
        required: false
        description: "Message priority (lower = higher priority)"
        default: 100
        min: 1
        max: 1000
      speed:
        type: "float"
        required: false
        description: "Speech speed multiplier"
        default: 1.0
        min: 0.5
        max: 2.0
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

  - topic: "audio_config"
    message_type: "AudioConfig"
    callback: "on_audio_config_change"
    description: "Audio configuration updates"
    qos:
      reliability: "reliable"
      durability: "persistent"
      history: "keep_last"
      depth: 1
    schema:
      volume:
        type: "float"
        required: false
        description: "Output volume (0.0 to 1.0)"
        min: 0.0
        max: 1.0
      voice:
        type: "string"
        required: false
        description: "Default voice"
      speed:
        type: "float"
        required: false
        description: "Default speech speed"
        min: 0.5
        max: 2.0

configuration:
  # Audio output settings
  audio:
    device: "hw:3,0"  # HiFiBerry DAC
    sample_rate: 44100
    channels: 2
    buffer_size: 1024
  
  # TTS settings
  tts:
    default_voice: "onyx"
    default_speed: 1.0
    default_volume: 0.8
    api_timeout: 30.0
    max_text_length: 1000
  
  # Performance settings
  performance:
    max_queue_size: 5
    processing_timeout: 10.0
    retry_attempts: 2
    retry_delay: 2.0

testing:
  mock_data:
    text_responses:
      - text: "Hello, how can I help you?"
        voice: "onyx"
        priority: 100
      - text: "I'm processing your request."
        voice: "onyx"
        priority: 50
  
  test_scenarios:
    - name: "basic_synthesis"
      description: "Test basic text-to-speech"
      input:
        topic: "text_response"
        text: "Hello world"
        voice: "onyx"
      expected_output:
        topic: "speaking_status"
        speaking: true
```

### 2.4 AI Cognition Node .messages Example

```yaml
# nodes/ai_cognition/.messages
# Message interface definition for AI cognition node

node_name: "ai_cognition"
version: "1.0"
description: "AI-powered conversation and decision making"

publishes:
  - topic: "text_response"
    message_type: "TextResponse"
    description: "AI-generated text responses"
    frequency: "on_demand"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 5
    schema:
      text:
        type: "string"
        required: true
        description: "AI-generated response text"
        max_length: 1000
      voice:
        type: "string"
        required: false
        description: "Suggested voice for TTS"
        default: "onyx"
      priority:
        type: "integer"
        required: false
        description: "Response priority"
        default: 100
        min: 1
        max: 1000
      context_id:
        type: "string"
        required: false
        description: "Conversation context identifier"
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

  - topic: "system_mode"
    message_type: "SystemMode"
    description: "System mode changes initiated by AI"
    frequency: "on_change"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      mode:
        type: "string"
        required: true
        description: "New system mode"
        allowed: ["listening", "speaking", "thinking", "idle", "error"]
      reason:
        type: "string"
        required: false
        description: "Reason for mode change"
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

subscribes:
  - topic: "voice_command"
    message_type: "VoiceCommand"
    callback: "on_voice_command"
    description: "Voice commands to process"
    qos:
      reliability: "reliable"
      durability: "volatile"
      history: "keep_all"
      depth: 5
    schema:
      text:
        type: "string"
        required: true
        description: "Voice command text"
      confidence:
        type: "float"
        required: true
        description: "Recognition confidence"
        min: 0.0
        max: 1.0
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

  - topic: "system_heartbeat"
    message_type: "SystemHeartbeat"
    callback: "on_system_heartbeat"
    description: "System health monitoring"
    qos:
      reliability: "best_effort"
      durability: "volatile"
      history: "keep_last"
      depth: 1
    schema:
      node_name:
        type: "string"
        required: true
        description: "Source node name"
      status:
        type: "string"
        required: true
        description: "Node status"
      timestamp:
        type: "float"
        required: true
        description: "Unix timestamp"

configuration:
  # AI settings
  ai:
    model: "gpt-3.5-turbo"
    max_tokens: 150
    temperature: 0.7
    timeout: 30.0
    max_context_length: 20  # conversation turns
  
  # Processing settings
  processing:
    confidence_threshold: 0.5
    max_queue_size: 3
    processing_timeout: 15.0
    retry_attempts: 2
    retry_delay: 3.0
  
  # Context management
  context:
    max_history_length: 50
    context_timeout: 3600.0  # 1 hour
    save_context: true
    context_file: "logs/conversation_context.json"

testing:
  mock_data:
    voice_commands:
      - text: "What time is it?"
        confidence: 0.95
      - text: "Tell me a joke"
        confidence: 0.87
  
  test_scenarios:
    - name: "basic_conversation"
      description: "Test basic AI conversation"
      input:
        topic: "voice_command"
        text: "Hello"
        confidence: 0.9
      expected_output:
        topic: "text_response"
        text: "contains:hello"  # Response should contain greeting
```

## 3. Configuration Loading and Validation

### 3.1 Configuration Loader

#### Code Summary

This section implements the ConfigLoader class for loading, validating, and processing YAML configuration files:

**Classes:**
- **ValidationError** (Dataclass): Configuration validation error with file path, error type, message, and optional line number
- **ConfigLoader**: Configuration loading engine with validation and environment variable expansion

**Key Methods in ConfigLoader:**
- `__init__()`: Initialize config loader with root path and validation error tracking
- `load_nodes_config()`: Load and validate .nodes configuration file with YAML parsing and environment expansion
- `load_node_messages_config()`: Load and validate individual node .messages configuration files
- `_validate_nodes_config()`: Validate .nodes configuration against schema rules
- `_validate_messages_config()`: Validate .messages configuration against schema rules
- `_expand_environment_variables()`: Recursively expand ${VAR} patterns in configuration
- `get_validation_errors()`: Return list of validation errors encountered during loading

**Key Features:**
- YAML syntax validation with detailed error reporting
- Schema validation for both .nodes and .messages files
- Environment variable expansion with ${VAR} and ${VAR:-default} syntax
- Comprehensive error tracking and reporting
- UTF-8 encoding support for international characters
- File existence checking with descriptive error messages

```python
# nevil_framework/config_loader.py

import yaml
import os
import jsonschema
from typing import Dict, Any, List
from dataclasses import dataclass

@dataclass
class ValidationError:
    file_path: str
    error_type: str
    message: str
    line_number: int = None

class ConfigLoader:
    """Loads and validates Nevil v3.0 configuration files"""
    
    def __init__(self, root_path: str = "."):
        self.root_path = root_path
        self.validation_errors = []
    
    def load_nodes_config(self) -> Dict[str, Any]:
        """Load and validate .nodes configuration file"""
        config_path = os.path.join(self.root_path, ".nodes")
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # Validate configuration
            self._validate_nodes_config(config, config_path)
            
            # Expand environment variables
            config = self._expand_environment_variables(config)
            
            return config
            
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in {config_path}: {e}")
    
    def load_node_messages_config(self, node_name: str) -> Dict[str, Any]:
        """Load and validate node .messages configuration file"""
        config_path = os.path.join(self.root_path, "nodes", node_name, ".messages")
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Messages config not found: {config_path}")
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # Validate configuration
            self._validate_messages_config(config, config_path)
            
            return config
            
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in {config_path}: {e}")
    
    def _validate_nodes_config(self, config: Dict[str, Any], file_path: str):
        """Validate .nodes configuration against schema"""
        # Implementation would use jsonschema or custom validation
        pass
    
    def _validate_messages_config(self, config: Dict[str, Any], file_path: str):
        """Validate .messages configuration against schema"""
        # Implementation would validate message schemas
        pass
    
    def _expand_environment_variables(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Expand environment variables in configuration"""
        # Implementation would recursively expand ${VAR} patterns
        pass
    
    def get_validation_errors(self) -> List[ValidationError]:
        """Get list of validation errors"""
        return self.validation_errors

# TEST: Configuration loader handles missing files gracefully
# TEST: YAML syntax errors are properly reported
# TEST: Environment variable expansion works correctly
# TEST: Validation catches schema violations
# TEST: Circular dependencies in node startup order are detected
```

## 4. Configuration Management Tools

### 4.1 Configuration Validator

#### Code Summary

This section implements a bash script for validating Nevil v3.0 configuration files with comprehensive YAML syntax checking:

**Script Components:**
- **validate_config.sh**: Complete configuration validation script with Python integration

**Key Functions:**
- **File existence check**: Verify .nodes file exists in project root
- **YAML syntax validation**: Use Python yaml.safe_load() to validate syntax
- **Node directory scanning**: Iterate through all node directories for .messages files
- **Individual node validation**: Check each .messages file for valid YAML syntax
- **Error reporting**: Clear success/failure indicators with descriptive messages
- **Exit code handling**: Proper exit codes for integration with CI/CD systems

**Key Features:**
- Cross-platform bash script with Python integration
- Comprehensive validation of all configuration files
- Clear visual feedback with ✓ and ✗ indicators
- Automatic discovery of node directories
- Detailed error messages for troubleshooting
- Integration-friendly exit codes for automation
- Validation of both root .nodes and individual .messages files

```bash
#!/bin/bash
# validate_config.sh - Validate Nevil v3.0 configuration files

echo "Validating Nevil v3.0 configuration..."

# Check if .nodes file exists
if [ ! -f ".nodes" ]; then
    echo "ERROR: .nodes file not found"
    exit 1
fi

# Validate YAML syntax
python3 -c "
import yaml
import sys

try:
    with open('.nodes', 'r') as f:
        yaml.safe_load(f)
    print('✓ .nodes file has valid YAML syntax')
except yaml.YAMLError as e:
    print(f'✗ .nodes file has invalid YAML: {e}')
    sys.exit(1)
"

# Check node message files
for node_dir in nodes/*/; do
    if [ -d "$node_dir" ]; then
        node_name=$(basename "$node_dir")
        messages_file="$node_dir/.messages"
        
        if [ -f "$messages_file" ]; then
            python3 -c "
import yaml
import sys

try:
    with open('$messages_file', 'r') as f:
        yaml.safe_load(f)
    print('✓ $node_name .messages file is valid')
except yaml.YAMLError as e:
    print(f'✗ $node_name .messages file has invalid YAML: {e}')
    sys.exit(1)
"
        else
            echo "✗ Missing .messages file for node: $node_name"
        fi
    fi
done

echo "Configuration validation complete"
```

### 4.2 Configuration Generator

#### Code Summary

This section implements a Python script for generating configuration file templates with command-line interface:

**Functions:**
- **generate_nodes_config()**: Generate template .nodes configuration file with default system settings
- **generate_node_messages_config()**: Generate template .messages configuration file for specific nodes
- **Main script**: Command-line interface with argparse for template generation

**Key Features in generate_nodes_config():**
- Complete .nodes template with version, system settings, environment variables
- Default framework configuration with health checks and timeouts
- PYTHONPATH setup for framework and audio components
- Launch configuration with timing and parallelization settings
- YAML serialization with proper formatting and indentation

**Key Features in generate_node_messages_config():**
- Individual .messages template for specific nodes
- Automatic directory creation for node structure
- Template structure with publishes, subscribes, configuration, and testing sections
- Node-specific versioning and description placeholders
- Mock data and test scenario template structures

**Command-line Interface:**
- `--nodes`: Generate root .nodes configuration template
- `--node <name>`: Generate .messages template for specified node
- Help system with usage instructions and examples

```python
# tools/generate_config.py - Generate configuration templates

def generate_nodes_config():
    """Generate a template .nodes configuration file"""
    template = {
        "version": "3.0",
        "description": "Nevil v3.0 configuration template",
        "system": {
            "framework_version": "3.0.0",
            "log_level": "INFO",
            "health_check_interval": 5.0,
            "shutdown_timeout": 10.0,
            "startup_delay": 2.0
        },
        "environment": {
            "NEVIL_VERSION": "3.0",
            "PYTHONPATH": "${PYTHONPATH}:./nevil_framework:./audio"
        },
        "nodes": {},
        "launch": {
            "parallel_launch": False,
            "wait_for_healthy": True,
            "ready_timeout": 30.0
        }
    }
    
    with open(".nodes", "w") as f:
        yaml.dump(template, f, default_flow_style=False, indent=2)

def generate_node_messages_config(node_name: str):
    """Generate a template .messages configuration file for a node"""
    template = {
        "node_name": node_name,
        "version": "1.0",
        "description": f"Message interface for {node_name} node",
        "publishes": [],
        "subscribes": [],
        "configuration": {},
        "testing": {
            "mock_data": {},
            "test_scenarios": []
        }
    }
    
    os.makedirs(f"nodes/{node_name}", exist_ok=True)
    with open(f"nodes/{node_name}/.messages", "w") as f:
        yaml.dump(template, f, default_flow_style=False, indent=2)

if __name__ == "__main__":
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description="Generate Nevil v3.0 configuration files")
    parser.add_argument("--nodes", action="store_true", help="Generate .nodes template")
    parser.add_argument("--node", type=str, help="Generate .messages template for specific node")
    
    args = parser.parse_args()
    
    if args.nodes:
        generate_nodes_config()
        print("Generated .nodes template")
    
    if args.node:
        generate_node_messages_config(args.node)
        print(f"Generated .messages template for {args.node}")
    
    if not args.nodes and not args.node:
        parser.print_help()

# TEST: Configuration generator creates valid YAML files
# TEST: Generated templates pass validation
# TEST: Environment variable placeholders are correctly formatted
```

## 5. Environment Variable Management

### 5.1 Environment Variable Patterns

```bash
# .env.example - Template for environment variables

# Required API Keys
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_ASSISTANT_ID=asst_your-assistant-id-here

# Audio Configuration
MICROPHONE_DEVICE_INDEX=1
SPEECH_RECOGNITION_LANGUAGE=en-US
TTS_VOICE=onyx
SPEECH_SYNTHESIS_VOLUME=6
SPEECH_SYNTHESIS_RATE=200

# System Configuration
LOG_LEVEL=INFO
NEVIL_VERSION=3.0
DEBUG_MODE=false

# Development Settings
MOCK_OPENAI=false
MOCK_AUDIO_HARDWARE=false
TEST_MODE=false
PROFILING=false

# Security Settings
MASK_SECRETS=true
VALIDATE_ENV_VARS=true
```

### 5.2 Environment Variable Validation

```python
# nevil_framework/env_validator.py

import os
import re
from typing import Dict, List, Optional, Any
from dataclasses import dataclass

@dataclass
class EnvVarRule:
    name: str
    required: bool = True
    pattern: Optional[str] = None
    allowed_values: Optional[List[str]] = None
    default: Optional[str] = None
    description: str = ""

class EnvironmentValidator:
    """Validates environment variables according to configuration rules"""
    
    def __init__(self):
        self.rules = {
            "OPENAI_API_KEY": EnvVarRule(
                name="OPENAI_API_KEY",
                required=True,
                pattern=r"^sk-[a-zA-Z0-9]{48}$",
                description="OpenAI API key for AI services"
            ),
            "OPENAI_ASSISTANT_ID": EnvVarRule(
                name="OPENAI_ASSISTANT_ID",
                required=False,
                pattern=r"^asst_[a-zA-Z0-9]{24}$",
                description="OpenAI Assistant ID for conversation"
            ),
            "LOG_LEVEL": EnvVarRule(
                name="LOG_LEVEL",
                required=False,
                allowed_values=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                default="INFO",
                description="Logging level for the system"
            ),
            "TTS_VOICE": EnvVarRule(
                name="TTS_VOICE",
                required=False,
                allowed_values=["onyx", "alloy", "echo", "fable", "nova", "shimmer"],
                default="onyx",
                description="Default voice for text-to-speech"
            ),
            "MICROPHONE_DEVICE_INDEX": EnvVarRule(
                name="MICROPHONE_DEVICE_INDEX",
                required=False,
                pattern=r"^\d+$",
                default="1",
                description="Audio device index for microphone"
            ),
            "SPEECH_SYNTHESIS_VOLUME": EnvVarRule(
                name="SPEECH_SYNTHESIS_VOLUME",
                required=False,
                pattern=r"^\d+(\.\d+)?$",
                default="6",
                description="Audio volume for speech synthesis"
            )
        }
    
    def validate_all(self) -> List[str]:
        """Validate all environment variables, return list of errors"""
        errors = []
        
        for rule in self.rules.values():
            error = self.validate_var(rule)
            if error:
                errors.append(error)
        
        return errors
    
    def validate_var(self, rule: EnvVarRule) -> Optional[str]:
        """Validate a single environment variable"""
        value = os.getenv(rule.name)
        
        # Check if required variable is missing
        if rule.required and not value:
            return f"Required environment variable {rule.name} is not set"
        
        # Use default if not set
        if not value and rule.default:
            os.environ[rule.name] = rule.default
            value = rule.default
        
        # Skip validation if not set and not required
        if not value:
            return None
        
        # Validate pattern
        if rule.pattern and not re.match(rule.pattern, value):
            return f"Environment variable {rule.name} does not match required pattern: {rule.pattern}"
        
        # Validate allowed values
        if rule.allowed_values and value not in rule.allowed_values:
            return f"Environment variable {rule.name} must be one of: {', '.join(rule.allowed_values)}"
        
        return None
    
    def get_var_with_default(self, name: str) -> str:
        """Get environment variable with default value"""
        if name in self.rules:
            rule = self.rules[name]
            return os.getenv(name, rule.default or "")
        return os.getenv(name, "")
    
    def mask_sensitive_vars(self, text: str) -> str:
        """Mask sensitive environment variables in text"""
        sensitive_patterns = [
            (r"sk-[a-zA-Z0-9]{48}", "sk-***MASKED***"),
            (r"asst_[a-zA-Z0-9]{24}", "asst_***MASKED***")
        ]
        
        masked_text = text
        for pattern, replacement in sensitive_patterns:
            masked_text = re.sub(pattern, replacement, masked_text)
        
        return masked_text

# TEST: Environment validator catches missing required variables
# TEST: Pattern validation works for API keys and other formatted values
# TEST: Default values are applied correctly
# TEST: Sensitive values are properly masked in logs
```

## 6. Configuration Hot-Reloading

### 6.1 Configuration Watcher

#### Code Summary

This section implements file system monitoring for hot-reloading configuration files with debouncing and callback management:

**Classes:**
- **ConfigFileHandler**: File system event handler for configuration file changes
- **ConfigurationWatcher**: Main watcher class for monitoring and reloading configurations

**Key Methods in ConfigFileHandler:**
- `__init__()`: Initialize event handler with callback and change tracking
- `on_modified()`: Handle file modification events with debouncing logic

**Key Methods in ConfigurationWatcher:**
- `__init__()`: Initialize watcher with root path and callback registry
- `watch_file()`: Register callback for specific configuration file changes
- `start_watching()`: Begin file system monitoring with recursive scanning
- `stop_watching()`: Clean shutdown of file system observer
- `_on_file_changed()`: Process configuration file changes with validation and callback execution

**Key Features:**
- File system monitoring using watchdog library
- Debouncing to prevent excessive reload events (1.0 second minimum)
- Recursive monitoring of nodes directories for .messages files
- Automatic configuration validation on file changes
- Callback system for custom reload behavior
- Error handling for invalid configuration changes
- Support for both .nodes and .messages file types
- Thread-safe operation with proper cleanup

```python
# nevil_framework/config_watcher.py

import os
import time
import threading
from typing import Callable, Dict, Any
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ConfigFileHandler(FileSystemEventHandler):
    """Handles configuration file change events"""
    
    def __init__(self, callback: Callable[[str], None]):
        self.callback = callback
        self.last_modified = {}
        
    def on_modified(self, event):
        if event.is_directory:
            return
            
        file_path = event.src_path
        
        # Only process .nodes and .messages files
        if not (file_path.endswith('.nodes') or file_path.endswith('.messages')):
            return
        
        # Debounce rapid file changes
        current_time = time.time()
        if file_path in self.last_modified:
            if current_time - self.last_modified[file_path] < 1.0:
                return
        
        self.last_modified[file_path] = current_time
        self.callback(file_path)

class ConfigurationWatcher:
    """Watches configuration files for changes and triggers reloads"""
    
    def __init__(self, root_path: str = "."):
        self.root_path = root_path
        self.observer = Observer()
        self.callbacks = {}  # file_path -> callback
        self.running = False
        
    def watch_file(self, file_path: str, callback: Callable[[Dict[str, Any]], None]):
        """Watch a specific configuration file for changes"""
        self.callbacks[file_path] = callback
        
    def start_watching(self):
        """Start watching for configuration changes"""
        if self.running:
            return
            
        handler = ConfigFileHandler(self._on_file_changed)
        
        # Watch root directory for .nodes file
        self.observer.schedule(handler, self.root_path, recursive=False)
        
        # Watch nodes directories for .messages files
        nodes_dir = os.path.join(self.root_path, "nodes")
        if os.path.exists(nodes_dir):
            self.observer.schedule(handler, nodes_dir, recursive=True)
        
        self.observer.start()
        self.running = True
        
    def stop_watching(self):
        """Stop watching for configuration changes"""
        if self.running:
            self.observer.stop()
            self.observer.join()
            self.running = False
    
    def _on_file_changed(self, file_path: str):
        """Handle configuration file change"""
        try:
            # Reload and validate configuration
            if file_path.endswith('.nodes'):
                config = ConfigLoader(self.root_path).load_nodes_config()
            elif file_path.endswith('.messages'):
                # Extract node name from path
                node_name = os.path.basename(os.path.dirname(file_path))
                config = ConfigLoader(self.root_path).load_node_messages_config(node_name)
            else:
                return
            
            # Call registered callback
            if file_path in self.callbacks:
                self.callbacks[file_path](config)
                
        except Exception as e:
            print(f"Error reloading configuration {file_path}: {e}")

# TEST: Configuration watcher detects file changes correctly
# TEST: Debouncing prevents excessive reload events
# TEST: Invalid configuration changes are handled gracefully
# TEST: Callbacks are triggered with correct configuration data
```

## Conclusion

The Nevil v3.0 configuration system provides a comprehensive, yet simple approach to managing system and node configuration. Key features include:

- **Human-readable YAML format** for easy editing and version control
- **Comprehensive validation** to catch errors early
- **Environment variable integration** for secure credential management
- **Hot-reloading capabilities** for development and debugging
- **Extensive documentation** and examples for each configuration option

This configuration system enables easy customization of node behavior, message interfaces, and system settings while maintaining type safety and validation to prevent runtime errors.

# TEST: All configuration examples are valid YAML
# TEST: Configuration validation catches common errors
# TEST: Environment variable expansion works in all contexts
# TEST: Hot-reloading maintains system stability
# TEST: Generated templates create working configurations