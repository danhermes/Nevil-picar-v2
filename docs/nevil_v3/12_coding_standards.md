# Nevil v3.0 Coding Standards and Conventions

## Overview

This document defines the coding standards that must be followed throughout the Nevil v3.0 framework to ensure consistency, maintainability, and reliability.

## Directory Structure and Package Management

### __init__.py Files
- **All new directories must contain an empty `__init__.py` file**
- This includes node directories, package directories, and any subdirectories containing Python modules
- Empty `__init__.py` files are sufficient - no content required unless explicit package initialization is needed

```bash
# Required directory structure with __init__.py files
nevil_v3/
├── __init__.py
├── nevil_framework/
│   ├── __init__.py
│   ├── base_node.py
│   └── message_bus.py
├── nodes/
│   ├── __init__.py
│   ├── speech_recognition/
│   │   ├── __init__.py
│   │   ├── speech_recognition_node.py
│   │   ├── .nodes
│   │   └── .messages
│   └── ai_cognition/
│       ├── __init__.py
│       ├── ai_cognition_node.py
│       ├── .nodes
│       └── .messages
└── audio/
    ├── __init__.py
    ├── audio_input.py
    └── audio_output.py
```

### Import Management
- **Use `__init__.py` approach for import references rather than adding to PYTHONPATH**
- Favor explicit package imports over path manipulation
- Avoid dynamic PYTHONPATH modification in production code

```python
# Preferred approach - explicit package imports
from nevil_framework.base_node import NevilNode
from nevil_framework.message_bus import MessageBus
from audio.audio_input import AudioInput

# Avoid this approach - PYTHONPATH manipulation
import sys
sys.path.append('./audio')
import audio_input  # Less clear, harder to track
```

## Import Organization

### Import Placement
- **Place all imports at the top of the file unless special logic is required**
- Group imports in the following order:
  1. Standard library imports
  2. Third-party library imports
  3. Local application imports
- Separate each group with a blank line

```python
# Standard library imports
import os
import sys
import time
import threading
from typing import Dict, List, Optional
from dataclasses import dataclass

# Third-party imports
import yaml
import numpy as np
import psutil

# Local application imports
from nevil_framework.base_node import NevilNode
from nevil_framework.message_bus import MessageBus
from audio.audio_input import AudioInput
```

### Conditional Imports
- Only use conditional imports when absolutely necessary (e.g., optional dependencies)
- Document why conditional imports are required
- Provide graceful fallbacks for missing optional dependencies

```python
# Acceptable conditional import for optional dependency
try:
    import pvporcupine  # Optional wake word detection
    WAKE_WORD_AVAILABLE = True
except ImportError:
    WAKE_WORD_AVAILABLE = False
    pvporcupine = None

# Document why this is conditional
def initialize_wake_word_detection(self):
    if not WAKE_WORD_AVAILABLE:
        self.logger.warning("Wake word detection not available - pvporcupine not installed")
        return False
    # ... implementation
```

## Logging Standards

### One-Line Logger Setup

All modules must use the standardized logging setup with a single import line:

```python
from nevil_framework.logging import get_logger; logger = get_logger(__name__)
```

This provides:
- Zero configuration setup
- Automatic module name detection
- Consistent formatting across all components
- Integration with framework-wide log management

### Strategic Log Statement Placement

#### Required Logging Points
- **Function Entry/Exit**: Log entry for complex functions and all exit points with results
- **API Calls**: All external service calls (OpenAI, hardware interfaces, network requests)
- **State Changes**: Node state transitions, configuration changes, mode switches
- **Exception Handlers**: All caught exceptions with context and recovery actions
- **Resource Operations**: File I/O, database operations, hardware access

#### Placement Examples

```python
def process_audio_chunk(self, audio_data):
    logger.debug(f"Processing audio chunk: {len(audio_data)} samples")

    try:
        # API call logging
        logger.info("Calling OpenAI Whisper API")
        result = self.whisper_client.transcribe(audio_data)
        logger.info(f"Transcription completed: {len(result.text)} characters")

        # State change logging
        logger.info(f"Speech recognition state: {self.state} -> PROCESSING")
        self.state = ProcessingState.PROCESSING

        return result

    except OpenAIError as e:
        logger.error(f"Whisper API failed: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.critical(f"Unexpected error in audio processing: {e}", exc_info=True)
        raise
```

### Log Level Usage Guidelines

#### DEBUG Level
- Variable values and intermediate calculations
- Detailed execution flow for troubleshooting
- Performance timing for optimization
- **When to use**: Development and detailed troubleshooting only

#### INFO Level
- Normal application flow and milestones
- Successful API calls and state changes
- User-initiated actions and their outcomes
- **When to use**: Standard operational logging

#### WARNING Level
- Recoverable errors and fallback activations
- Performance issues or resource constraints
- Deprecated functionality usage
- **When to use**: Issues that don't stop operation but need attention

#### ERROR Level
- Failed operations that are retried or handled gracefully
- API failures with fallback mechanisms
- Configuration errors that can be worked around
- **When to use**: Errors that are handled but may indicate problems

#### CRITICAL Level
- System failures requiring immediate attention
- Unrecoverable errors leading to shutdown
- Security violations or access violations
- **When to use**: Failures that compromise system operation

### Zero-Configuration Module Setup

Every module must be able to start logging with no additional configuration:

```python
# Standard module template
"""Module docstring"""

# Standard imports...
from nevil_framework.logging import get_logger; logger = get_logger(__name__)

class ExampleNode:
    def __init__(self):
        logger.info(f"{self.__class__.__name__} initializing")
        # Module starts logging immediately
```

### Severity and Context Guidelines

- **Include context**: Always log relevant variable values and state
- **Use exc_info=True**: For exceptions that need stack traces
- **Avoid sensitive data**: Never log passwords, API keys, or personal information
- **Performance awareness**: DEBUG level should not impact production performance

## Code Organization Standards

### Module Structure
- Each module should have a clear, single responsibility
- Organize code in the following order:
  1. Module docstring
  2. Imports
  3. Constants and configuration
  4. Classes and functions
  5. Main execution block (if applicable)

```python
"""
Module docstring explaining the purpose and functionality.

This module implements the speech recognition node for Nevil v3.0,
integrating OpenAI Whisper for audio processing while preserving
v1.0 audio pipeline compatibility.
"""

# Standard library imports
import os
import time
from typing import Dict, Optional

# Third-party imports
import openai

# Local imports
from nevil_framework.base_node import NevilNode
from audio.audio_input import AudioInput

# Requirements.txt
"""
All non-system libs must be listed with their version (if possible) in the root-level requirements.txt. Nevil launch should be able to install requirements.txt libs and have ALL the needed libs for Nevil to run.
"""

# Module constants
DEFAULT_ENERGY_THRESHOLD = 300
DEFAULT_TIMEOUT = 18.0

class SpeechRecognitionNode(NevilNode):
    """Speech recognition node implementation."""

    def __init__(self):
        super().__init__("speech_recognition")
        # ... implementation

if __name__ == "__main__":
    # Main execution block
    node = SpeechRecognitionNode()
    node.start()
```

### Class Organization
- Organize class methods in logical groups:
  1. `__init__` and setup methods
  2. Public interface methods
  3. Event handlers and callbacks
  4. Private helper methods
  5. Cleanup and shutdown methods

```python
class SpeechRecognitionNode(NevilNode):
    """Speech recognition node with standardized method organization."""

    # 1. Initialization
    def __init__(self):
        super().__init__("speech_recognition")
        self._setup_audio()

    def _setup_audio(self):
        """Private setup method."""
        pass

    # 2. Public interface
    def start_listening(self):
        """Public method to start speech recognition."""
        pass

    def stop_listening(self):
        """Public method to stop speech recognition."""
        pass

    # 3. Event handlers
    def on_audio_data(self, audio_data):
        """Handle incoming audio data."""
        pass

    def on_system_mode_change(self, message):
        """Handle system mode changes."""
        pass

    # 4. Private helpers
    def _process_audio(self, audio_data):
        """Private audio processing method."""
        pass

    def _validate_input(self, input_data):
        """Private validation method."""
        pass

    # 5. Cleanup
    def cleanup(self):
        """Cleanup resources on shutdown."""
        super().cleanup()
```

## File and Directory Naming

### Naming Conventions
- **Python files**: `snake_case.py` (e.g., `speech_recognition_node.py`)
- **Directories**: `snake_case` (e.g., `speech_recognition/`)
- **Classes**: `PascalCase` (e.g., `SpeechRecognitionNode`)
- **Functions and variables**: `snake_case` (e.g., `process_audio_data`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `DEFAULT_TIMEOUT`)

### Node Directory Structure
Each node must follow this exact structure:
```bash
nodes/{node_name}/
├── __init__.py                    # Required empty file
├── {node_name}_node.py           # Main node implementation
├── .nodes                        # Node configuration
├── .messages                     # Message interface
└── [additional_modules.py]       # Optional supporting modules
```

## Documentation Standards

### Docstring Requirements
- All public classes and methods must have docstrings
- Use Google-style docstrings for consistency
- Include type hints in function signatures

```python
class SpeechRecognitionNode(NevilNode):
    """Speech recognition node for Nevil v3.0 framework.

    This node handles audio input processing and speech-to-text conversion
    using OpenAI Whisper while maintaining v1.0 audio pipeline compatibility.

    Attributes:
        audio_input: AudioInput instance for microphone access
        energy_threshold: Minimum energy level for speech detection
    """

    def process_audio_data(self, audio_data: bytes, timeout: float = 5.0) -> Optional[str]:
        """Process raw audio data and return recognized text.

        Args:
            audio_data: Raw audio bytes from microphone
            timeout: Maximum processing time in seconds

        Returns:
            Recognized text string or None if recognition failed

        Raises:
            AudioProcessingError: If audio processing fails
            TimeoutError: If processing exceeds timeout
        """
        # Implementation here
        pass
```

## Error Handling Standards

### Exception Handling
- Use specific exception types rather than generic `Exception`
- Always log errors with appropriate context
- Provide graceful degradation when possible

```python
def process_audio_data(self, audio_data: bytes) -> Optional[str]:
    """Process audio with proper error handling."""
    try:
        result = self.recognizer.recognize_speech(audio_data)
        return result

    except AudioDeviceError as e:
        self.logger.error(f"Audio device error: {e}")
        # Attempt to reinitialize audio device
        self._reinitialize_audio_device()
        return None

    except NetworkError as e:
        self.logger.warning(f"Network error, using fallback: {e}")
        # Graceful degradation to offline processing
        return self._fallback_recognition(audio_data)

    except Exception as e:
        self.logger.error(f"Unexpected error in audio processing: {e}")
        # Don't re-raise - return None for graceful handling
        return None
```

## Configuration Standards

### Configuration File Conventions
- Use YAML for all configuration files
- Follow the established `.nodes` and `.messages` patterns
- Include validation schemas for all configuration

```yaml
# Example .nodes file structure
node_name: "speech_recognition"
version: "1.0"
description: "Speech recognition using OpenAI Whisper"

creation:
  class_name: "SpeechRecognitionNode"
  module_path: "nodes.speech_recognition.speech_recognition_node"

runtime:
  status: live
  priority: high
  restart_policy: always
```

## Testing Standards

### Test Organization
- Mirror the source code structure in test directories
- Use descriptive test method names that explain what is being tested
- Include both unit tests and integration tests

```bash
tests/
├── __init__.py
├── unit/
│   ├── __init__.py
│   ├── test_speech_recognition_node.py
│   └── test_audio_input.py
└── integration/
    ├── __init__.py
    └── test_speech_pipeline.py
```

```python
class TestSpeechRecognitionNode(unittest.TestCase):
    """Unit tests for SpeechRecognitionNode."""

    def test_audio_processing_with_valid_input_returns_text(self):
        """Test that valid audio input produces recognized text."""
        # Test implementation
        pass

    def test_audio_processing_with_invalid_input_returns_none(self):
        """Test that invalid audio input is handled gracefully."""
        # Test implementation
        pass

    def test_node_initialization_creates_required_components(self):
        """Test that node initialization sets up all required components."""
        # Test implementation
        pass
```

## Logging Standards

### Logging Configuration and Setup

#### One-Line Logger Setup
- **Every module must use the standardized one-line logger import**
- **No manual logger configuration required**
- **Automatic module-level logger creation with proper naming**

```python
# Standard one-line logger setup for ANY module
from nevil_framework.logging import get_logger
logger = get_logger(__name__)  # Creates logger named after the module

# That's it! Now use logger throughout the module
logger.info("Module loaded and ready")
logger.debug("Detailed diagnostic information")
logger.error("Something went wrong", exc_info=True)
```

#### Module Types and Logger Setup

**For Node Classes:**
```python
# nodes/speech_recognition/speech_recognition_node.py
from nevil_framework.logging import get_logger
from nevil_framework.base_node import NevilNode

logger = get_logger(__name__)  # Creates: "nodes.speech_recognition.speech_recognition_node"

class SpeechRecognitionNode(NevilNode):
    def __init__(self):
        super().__init__("speech_recognition")
        logger.info("Speech recognition node initializing")
```

**For Framework Modules:**
```python
# nevil_framework/message_bus.py
from nevil_framework.logging import get_logger

logger = get_logger(__name__)  # Creates: "nevil_framework.message_bus"

class MessageBus:
    def __init__(self):
        logger.info("Message bus starting")
```

**For Utility Modules:**
```python
# audio/audio_input.py
from nevil_framework.logging import get_logger

logger = get_logger(__name__)  # Creates: "audio.audio_input"

class AudioInput:
    def __init__(self):
        logger.info("Audio input initialized")
```

**For Scripts and Standalone Files:**
```python
# scripts/health_check.py
from nevil_framework.logging import get_logger

logger = get_logger(__name__)  # Creates: "scripts.health_check"

def main():
    logger.info("Health check script starting")
```

#### Logger Naming Convention
The `get_logger(__name__)` pattern automatically creates hierarchical logger names:
- `nodes.speech_recognition.speech_recognition_node`
- `nevil_framework.message_bus`
- `audio.audio_input`
- `scripts.health_check`

This enables per-module log level control:
```bash
# Environment variables for granular control
export LOG_LEVEL=INFO                           # Global default
export LOG_LEVEL_nodes=DEBUG                    # All node modules
export LOG_LEVEL_audio=WARNING                  # Audio modules only
export LOG_LEVEL_nodes_speech_recognition=DEBUG # Specific node only
```

### Strategic Log Statement Placement

#### Where to Place Log Statements
- **Function entry/exit for important operations**
- **Before and after external API calls**
- **At decision points and state changes**
- **In exception handlers**
- **At resource allocation/deallocation**

```python
def process_audio_data(self, audio_data):
    """Process audio with strategic logging placement."""

    # Log function entry for important operations
    logger.info("Starting audio processing", extra={
        "audio_length": len(audio_data),
        "sample_rate": self.sample_rate
    })

    try:
        # Log before external API calls
        logger.debug("Calling OpenAI speech recognition API")
        result = self.openai_client.audio.transcribe(audio_data)

        # Log successful completion
        logger.info("Audio processing completed", extra={
            "result_length": len(result.text) if result.text else 0,
            "confidence": getattr(result, 'confidence', None)
        })

        return result.text

    except Exception as e:
        # Log in exception handlers with context
        logger.error("Audio processing failed", extra={
            "error_type": type(e).__name__,
            "audio_length": len(audio_data)
        }, exc_info=True)
        return None
```

#### Function-Level Logging Patterns

**Constructor Logging:**
```python
def __init__(self, config):
    logger.info("Initializing speech recognition node")
    self.config = config
    self._setup_audio()
    logger.info("Speech recognition node initialized successfully")
```

**API Call Logging:**
```python
def call_openai_api(self, audio_data):
    logger.debug("Making OpenAI API request", extra={"data_size": len(audio_data)})

    try:
        response = self.client.audio.transcribe(audio_data)
        logger.info("OpenAI API call successful", extra={
            "response_length": len(response.text)
        })
        return response

    except openai.APIError as e:
        logger.error("OpenAI API error", extra={
            "error_code": e.code,
            "error_type": e.type
        }, exc_info=True)
        raise
```

**State Change Logging:**
```python
def set_listening_mode(self, mode):
    logger.info("Changing listening mode", extra={
        "from_mode": self.current_mode,
        "to_mode": mode,
        "reason": "user_request"
    })

    self.current_mode = mode
    self._update_audio_settings()

    logger.info("Listening mode changed successfully", extra={
        "new_mode": self.current_mode
    })
```

### Log Level Usage and Severity Guidelines

#### DEBUG Level - Development and Troubleshooting
- **Use for**: Detailed diagnostic information, variable values, loop iterations
- **NOT for production**: Performance impact, sensitive data exposure risk
- **Placement**: Inside loops, detailed state information, trace-level debugging

```python
# DEBUG - Detailed diagnostic info
logger.debug("Processing audio chunk", extra={
    "chunk_index": i,
    "chunk_size": len(chunk),
    "processing_time_ms": processing_time * 1000,
    "memory_usage": psutil.Process().memory_info().rss / 1024 / 1024
})

# DEBUG - Variable state tracking
logger.debug("Recognition parameters", extra={
    "energy_threshold": self.energy_threshold,
    "pause_threshold": self.pause_threshold,
    "timeout": self.timeout
})
```

#### INFO Level - Normal Operations
- **Use for**: Successful operations, system state changes, user-relevant events
- **Placement**: Function completion, mode changes, successful API calls
- **Frequency**: Moderate - important events but not overwhelming

```python
# INFO - System state changes
logger.info("Speech recognition activated", extra={
    "trigger": "wake_word_detected",
    "confidence": 0.85
})

# INFO - Successful operations
logger.info("Audio device configured", extra={
    "device": "USB Microphone",
    "sample_rate": 44100,
    "channels": 1
})

# INFO - User-relevant events
logger.info("Voice command recognized", extra={
    "command": "turn on lights",
    "confidence": 0.92,
    "processing_time_ms": 1500
})
```

#### WARNING Level - Recoverable Issues
- **Use for**: Fallback mechanisms, performance degradation, configuration issues
- **Placement**: When using fallbacks, timeout handling, resource constraints
- **Action**: System continues but with degraded performance

```python
# WARNING - Fallback mechanisms
logger.warning("Primary speech service unavailable, using fallback", extra={
    "primary_service": "openai_whisper",
    "fallback_service": "local_speech_recognition",
    "expected_impact": "reduced_accuracy"
})

# WARNING - Performance issues
logger.warning("Audio processing taking longer than expected", extra={
    "processing_time_ms": 5000,
    "expected_time_ms": 2000,
    "audio_length_seconds": 10
})

# WARNING - Configuration issues
logger.warning("Using default configuration", extra={
    "missing_config": "energy_threshold",
    "default_value": 300,
    "config_file": ".env"
})
```

#### ERROR Level - Functionality Impact
- **Use for**: Failed operations, resource exhaustion, integration failures
- **Placement**: Exception handlers, failed API calls, initialization failures
- **Action**: Feature may be unavailable but system continues

```python
# ERROR - Failed operations
logger.error("Audio device initialization failed", extra={
    "device_index": 1,
    "error": "Device not found",
    "available_devices": available_devices
}, exc_info=True)

# ERROR - API failures
logger.error("OpenAI API request failed", extra={
    "endpoint": "/v1/audio/transcriptions",
    "status_code": 429,
    "retry_after": retry_after
}, exc_info=True)

# ERROR - Resource exhaustion
logger.error("Insufficient memory for audio processing", extra={
    "required_mb": required_memory,
    "available_mb": available_memory,
    "audio_duration_seconds": duration
})
```

#### CRITICAL Level - System-Threatening
- **Use for**: Security violations, data corruption, unrecoverable failures
- **Placement**: Security breaches, system shutdown triggers, data integrity issues
- **Action**: Immediate attention required, possible system shutdown

```python
# CRITICAL - Security violations
logger.critical("Unauthorized access attempt detected", extra={
    "source_ip": request.remote_addr,
    "attempted_resource": "/admin/config",
    "action_taken": "connection_blocked"
})

# CRITICAL - System resource exhaustion
logger.critical("System memory critically low", extra={
    "available_memory_mb": 50,
    "memory_threshold_mb": 100,
    "action": "initiating_emergency_shutdown"
})

# CRITICAL - Data corruption
logger.critical("Configuration file corruption detected", extra={
    "file_path": ".nodes",
    "corruption_type": "invalid_yaml_syntax",
    "backup_available": backup_exists
})
```

### Module Setup Requirements

#### Mandatory Import Pattern
**Every Python file must start with this exact pattern:**

```python
# Standard library imports
import os
import sys
from typing import Dict, List, Optional

# Third-party imports
import yaml
import numpy as np

# Framework imports
from nevil_framework.logging import get_logger

# Module logger - REQUIRED as second import block
logger = get_logger(__name__)

# Rest of imports
from nevil_framework.base_node import NevilNode
```

#### Logger Configuration Hierarchy
The framework automatically configures loggers based on module hierarchy:

```python
# Auto-configured logger names and inheritance:
# nevil_framework              <- Root framework logger
# ├── nevil_framework.message_bus
# ├── nevil_framework.base_node
# └── nevil_framework.logging
#
# nodes                        <- All nodes logger
# ├── nodes.speech_recognition
# ├── nodes.speech_synthesis
# └── nodes.ai_cognition
#
# audio                        <- Audio subsystem logger
# ├── audio.audio_input
# └── audio.audio_output
```

#### Zero-Configuration Setup
**No manual logger configuration needed:**
- Log levels set via environment variables
- Output format automatically configured
- File rotation handled by framework
- EST timestamps applied automatically
- Node identification added automatically

```python
# This is ALL you need for logging:
from nevil_framework.logging import get_logger
logger = get_logger(__name__)

# Start logging immediately:
logger.info("Module ready")  # Automatically formatted with EST timestamp, node ID, etc.
```

### Log Levels and Usage Guidelines

#### DEBUG Level
- Detailed diagnostic information
- Variable values and state information
- Function entry/exit traces
- Should not be enabled in production

```python
self.logger.debug("Processing audio chunk", extra={
    "chunk_size": len(chunk),
    "sample_rate": self.sample_rate,
    "processing_time_ms": processing_time * 1000
})
```

#### INFO Level
- General operational information
- System state changes
- Successful completion of major operations
- User-relevant events

```python
self.logger.info("Speech recognition activated", extra={
    "activation_source": "wake_word",
    "confidence": 0.85,
    "duration_ms": 1200
})
```

#### WARNING Level
- Recoverable errors or unusual conditions
- Performance degradation
- Fallback mechanism activation
- Configuration issues that don't prevent operation

```python
self.logger.warning("Using fallback speech recognition", extra={
    "reason": "api_timeout",
    "fallback_method": "local_whisper",
    "expected_degradation": "accuracy_reduction"
})
```

#### ERROR Level
- Error conditions that affect functionality
- Failed operations that require attention
- Resource exhaustion
- Integration failures

```python
self.logger.error("Audio device initialization failed", extra={
    "device_index": device_index,
    "error_code": e.errno,
    "available_devices": available_devices
}, exc_info=True)
```

#### CRITICAL Level
- System-threatening errors
- Security violations
- Unrecoverable failures
- Conditions requiring immediate attention

```python
self.logger.critical("Node shutdown due to resource exhaustion", extra={
    "memory_usage_mb": memory_usage,
    "memory_limit_mb": memory_limit,
    "action": "emergency_shutdown"
})
```

### Log File Management and Rotation
- **Respect the `KEEP_LOG_FILES=3` limit strictly**
- **Use the framework's automatic log rotation**
- **Monitor log file sizes and implement size-based rotation**
- **Ensure proper cleanup of old log files**

```python
# Log file configuration (handled by framework)
LOG_CONFIG = {
    "max_files": 3,  # KEEP_LOG_FILES=3
    "max_size_mb": 50,
    "rotation_policy": "size_and_count",
    "cleanup_on_startup": True
}

# Node-specific log file naming
def get_log_file_path(self):
    """Get the log file path for this node."""
    return f"logs/nodes/{self.node_name}/{self.node_name}.log"
```

### Structured Logging Patterns
- **Include context information consistently**
- **Use extra fields for machine-readable data**
- **Maintain consistent field naming across nodes**

```python
# Standard context fields
STANDARD_LOG_FIELDS = {
    "node_name": self.node_name,
    "process_id": os.getpid(),
    "thread_id": threading.current_thread().ident,
    "timestamp": time.time()
}

# Example of consistent structured logging
def log_message_processing(self, message, status, duration_ms=None, error=None):
    """Log message processing with consistent structure."""
    log_data = {
        "message_id": message.get("id"),
        "message_type": message.get("type"),
        "status": status,
        "component": "message_processor"
    }

    if duration_ms:
        log_data["duration_ms"] = duration_ms

    if error:
        log_data["error"] = str(error)
        self.logger.error("Message processing failed", extra=log_data, exc_info=True)
    else:
        self.logger.info("Message processed successfully", extra=log_data)
```

### Performance and Resource Considerations
- **Avoid expensive string formatting in log messages**
- **Use lazy evaluation for debug messages**
- **Implement log level checking for expensive operations**

```python
# Efficient logging practices
def process_large_dataset(self, dataset):
    """Process dataset with efficient logging."""

    # Use lazy evaluation for debug logging
    if self.logger.isEnabledFor(logging.DEBUG):
        self.logger.debug("Processing dataset", extra={
            "dataset_size": len(dataset),
            "memory_usage": self._get_memory_usage()
        })

    # Avoid expensive string operations in production
    if result:
        self.logger.info("Dataset processed successfully", extra={
            "processed_count": len(result),
            "success_rate": success_rate
        })
    else:
        self.logger.error("Dataset processing failed")
```

### Security and Sensitive Data Handling
- **Never log sensitive information directly**
- **Implement automatic masking for known sensitive fields**
- **Use placeholder values for debugging sensitive data**

```python
def log_configuration_safely(self, config):
    """Log configuration while protecting sensitive data."""

    # Define sensitive field patterns
    sensitive_patterns = ['key', 'token', 'password', 'secret', 'credential']

    # Create safe copy of configuration
    safe_config = self._mask_sensitive_data(config, sensitive_patterns)

    self.logger.info("Node configuration loaded", extra={
        "config": safe_config,
        "config_source": config.get("source", "unknown")
    })

def _mask_sensitive_data(self, data, sensitive_patterns):
    """Mask sensitive data in configuration."""
    if isinstance(data, dict):
        masked = {}
        for key, value in data.items():
            if any(pattern.lower() in key.lower() for pattern in sensitive_patterns):
                masked[key] = "***MASKED***"
            elif isinstance(value, (dict, list)):
                masked[key] = self._mask_sensitive_data(value, sensitive_patterns)
            else:
                masked[key] = value
        return masked
    elif isinstance(data, list):
        return [self._mask_sensitive_data(item, sensitive_patterns) for item in data]
    else:
        return data
```

### Log Analysis and Monitoring Integration
- **Use consistent field names for log analysis tools**
- **Include correlation IDs for request tracing**
- **Implement log aggregation-friendly formats**

```python
def start_request_processing(self, request_id, request_type):
    """Start processing with correlation tracking."""
    correlation_id = f"{self.node_name}-{request_id}-{int(time.time())}"

    self.logger.info("Request processing started", extra={
        "correlation_id": correlation_id,
        "request_id": request_id,
        "request_type": request_type,
        "component": "request_handler",
        "phase": "start"
    })

    return correlation_id

def complete_request_processing(self, correlation_id, status, duration_ms):
    """Complete processing with correlation tracking."""
    self.logger.info("Request processing completed", extra={
        "correlation_id": correlation_id,
        "status": status,
        "duration_ms": duration_ms,
        "component": "request_handler",
        "phase": "complete"
    })
```

### Error Context and Debugging Information
- **Include sufficient context for debugging**
- **Use exc_info=True for exception logging**
- **Provide actionable information in error messages**

```python
def handle_processing_error(self, operation, input_data, error):
    """Log processing errors with comprehensive context."""

    # Gather debugging context
    debug_context = {
        "operation": operation,
        "input_size": len(input_data) if input_data else 0,
        "node_state": self._get_node_state(),
        "memory_usage_mb": self._get_memory_usage(),
        "error_type": type(error).__name__,
        "error_message": str(error),
        "component": "error_handler"
    }

    # Include suggestions for resolution
    if isinstance(error, MemoryError):
        debug_context["suggested_action"] = "reduce_batch_size_or_restart_node"
    elif isinstance(error, TimeoutError):
        debug_context["suggested_action"] = "check_network_connectivity_or_increase_timeout"
    else:
        debug_context["suggested_action"] = "check_logs_and_system_resources"

    self.logger.error("Processing operation failed", extra=debug_context, exc_info=True)
```

### Log Configuration Best Practices
- **Configure log levels per component**
- **Use environment variables for runtime log control**
- **Implement dynamic log level adjustment**

```python
# Environment-based log configuration
def configure_logging(self):
    """Configure logging based on environment variables."""

    # Base log level from environment
    base_level = os.getenv("LOG_LEVEL", "INFO").upper()

    # Component-specific overrides
    component_levels = {
        "audio_processor": os.getenv("AUDIO_LOG_LEVEL", base_level),
        "message_handler": os.getenv("MESSAGE_LOG_LEVEL", base_level),
        "api_client": os.getenv("API_LOG_LEVEL", base_level)
    }

    # Apply configuration
    self.logger.setLevel(getattr(logging, base_level))

    # Set component-specific levels
    for component, level in component_levels.items():
        component_logger = logging.getLogger(f"{self.node_name}.{component}")
        component_logger.setLevel(getattr(logging, level))

    self.logger.info("Logging configured", extra={
        "base_level": base_level,
        "component_levels": component_levels
    })
```

These logging standards ensure consistent, secure, and maintainable logging practices across the Nevil v3.0 framework while providing the information needed for effective debugging and monitoring.

## Performance Standards

### Memory Management
- Implement proper cleanup in all `cleanup()` methods
- Avoid memory leaks in long-running processes
- Monitor resource usage and respect configured limits

### Processing Efficiency
- Use appropriate data structures for the task
- Implement proper caching where beneficial
- Avoid blocking operations in main processing loops

```python
def process_data_efficiently(self, data_stream):
    """Example of efficient data processing."""
    # Use appropriate data structures
    processed_data = collections.deque(maxlen=1000)

    # Batch processing for efficiency
    batch_size = 100
    for i in range(0, len(data_stream), batch_size):
        batch = data_stream[i:i+batch_size]
        result = self._process_batch(batch)
        processed_data.extend(result)

    return processed_data
```

## Security Standards

### Credential Management
- Never log sensitive information (API keys, tokens)
- Use environment variables for sensitive configuration
- Implement proper secret masking in logs

```python
def log_configuration_safely(self, config):
    """Log configuration while masking sensitive data."""
    safe_config = self._mask_sensitive_keys(config)
    self.logger.info("Node configuration: %s", safe_config)

def _mask_sensitive_keys(self, config):
    """Mask sensitive configuration keys."""
    sensitive_keys = ['api_key', 'token', 'password', 'secret']
    masked_config = config.copy()

    for key in sensitive_keys:
        if key in masked_config:
            masked_config[key] = "***MASKED***"

    return masked_config
```

## Version Control Standards

### Commit Messages
- Use clear, descriptive commit messages
- Include the component being modified
- Reference issue numbers when applicable

```bash
# Good commit messages
git commit -m "speech_recognition: Add wake word detection support"
git commit -m "message_bus: Fix memory leak in queue processing"
git commit -m "docs: Update configuration file format specification"
```

### Branch Naming
- Use descriptive branch names
- Include the feature or fix being implemented

```bash
# Good branch names
feature/wake-word-detection
fix/audio-device-reconnection
docs/coding-standards-update
```

## Compliance Requirements

All code contributed to the Nevil v3.0 framework must:

1. **Follow these coding standards completely**
2. **Include appropriate `__init__.py` files**
3. **Use proper import organization**
4. **Include comprehensive docstrings**
5. **Implement proper error handling**
6. **Respect resource limits and cleanup requirements**
7. **Pass all automated style and quality checks**

These standards ensure consistency across the Nevil v3.0 framework and facilitate maintenance, debugging, and collaboration. All contributors must follow these standards for code to be accepted into the framework.