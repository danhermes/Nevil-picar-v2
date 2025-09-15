# Nevil v3.0 Declarative Messaging Architecture

## Code Summary

This document contains comprehensive declarative messaging implementations across 3 major code sections:

### Section 1: Enhanced NevilNode Base Class
**Declarative messaging framework** - Complete NevilNode class with init_messages() method, automatic callback mapping, and configuration-driven pub/sub setup.

### Section 8: init_messages() Implementation
**Configuration processing engine** - Detailed implementation of automatic publisher/subscriber setup, callback validation, and message bus registration.

### Section 9: Enhanced publish() Method
**Publishing validation system** - Topic declaration validation, message creation, and bus integration with declarative constraints.

## Overview

This document describes the refined NevilNode base class design that implements declarative messaging through the `init_messages()` method. This approach eliminates the complexity of manual publish/subscribe method calls by automatically configuring all messaging during node initialization based on `.messages` configuration files.

## Core Philosophy

**"Configuration over Code"** - All messaging setup is declared once in configuration files and automatically applied during node initialization, eliminating the need for manual subscribe/publish method calls throughout the codebase.

## Architectural Clarification

**IMPORTANT**: The declarative messaging system uses a **hybrid architecture**:

- **Developer Layer**: Purely declarative via `.messages` files
- **Framework Layer**: Uses programmatic [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) calls internally

The [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) and [`unsubscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:562) methods **are NOT called by node developers**. They are internal framework methods called automatically by [`NevilNode._register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364) during the declarative setup process.

**The Flow**: `.messages` file → [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) → [`_register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364) → [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548)

## Key Benefits

### 1. **Simplicity**
- Single `init_messages()` method handles all messaging setup
- No manual `subscribe()` or `publish()` calls in node code
- Declarative configuration via `.messages` files

### 2. **Reliability**
- All messaging configured once during `__init__`
- Automatic validation of callback methods
- Clear separation between configuration and implementation

### 3. **Maintainability**
- Messaging topology visible at a glance in `.messages` files
- Easy to add/remove topics without code changes
- Centralized messaging configuration per node

## Architecture Components

### 1. Enhanced NevilNode Base Class

```python
class NevilNode(ABC):
    def __init__(self, node_name: str, config_path: str = None):
        # ... standard initialization ...
        
        # Message system with declarative approach
        self.message_queues = {}  # topic -> queue
        self.message_callbacks = {}  # topic -> callback method name
        self.published_topics = set()   # topics this node publishes
        self.subscribed_topics = set()  # topics this node subscribes to
        
        # Declarative messaging setup - runs once during initialization
        self.init_messages()
        
    def init_messages(self):
        """
        Initialize messaging based on .messages configuration file.
        
        This method runs once during __init__ and automatically sets up all
        publishers and subscribers for the life of the node based on the
        declarative configuration in the .messages file.
        """
        # Read .messages file and configure all pub/sub automatically
        # Validate callback methods exist
        # Set up message queues
        # Register with message bus when available
```

### 2. Declarative Configuration Format

Each node has a `.messages` file that declares all messaging:

```yaml
# nodes/speech_recognition/.messages
publishes:
  - topic: "prompt_audio"
    message_type: "PromptAudio"
    schema:
      text: str
      confidence: float
      timestamp: float
      
  - topic: "listening_status"
    message_type: "ListeningStatus"
    schema:
      listening: bool
      reason: str
      timestamp: float

subscribes:
  - topic: "speaking_status"
    message_type: "SpeakingStatus"
    callback: "on_speaking_status_change"
    
  - topic: "system_mode"
    message_type: "SystemMode"
    callback: "on_system_mode_change"
```

### 3. Automatic Callback Mapping

The framework automatically maps topics to callback methods:

```python
def _handle_message(self, topic: str, message: Message):
    """Handle incoming message using declaratively configured callbacks"""
    if topic in self.message_callbacks:
        callback_name = self.message_callbacks[topic]
        callback_method = getattr(self, callback_name, None)
        
        if callback_method and callable(callback_method):
            callback_method(message)
        else:
            self.logger.error(f"Callback method '{callback_name}' not found")
```

## Implementation Examples

### Before: Manual Approach (v2.0)

```python
class SpeechRecognitionNode(NevilNode):
    def initialize(self):
        # Initialize audio components
        self.audio_input = AudioInput()
        
        # Manual subscription setup - COMPLEX!
        self.subscribe("system_mode", self.on_system_mode_change)
        self.subscribe("speaking_status", self.on_speaking_status_change)
        self.subscribe("audio_config", self.on_audio_config_change)
        
    def process_audio(self, audio_data):
        # Manual publishing - SCATTERED!
        self.publish("voice_command", command_data)
        self.publish("listening_status", status_data)
        self.publish("audio_level", level_data)
```

### After: Declarative Approach (v3.0)

```python
class SpeechRecognitionNode(NevilNode):
    def initialize(self):
        # Initialize audio components
        self.audio_input = AudioInput()
        
        # No manual subscribe() calls needed!
        # All subscriptions are automatically configured via .messages file
        # during init_messages() in __init__
        
    def process_audio(self, audio_data):
        # Publishing still uses publish() but topics are pre-validated
        self.publish("voice_command", command_data)  # Declared in .messages
        self.publish("listening_status", status_data)  # Declared in .messages
        self.publish("audio_level", level_data)  # Declared in .messages
```

## Message Flow Architecture

### 1. Initialization Flow

```
Node Creation
    ↓
__init__() called
    ↓
Load .messages config
    ↓
init_messages() called
    ↓
Parse publishes section → Add to published_topics
    ↓
Parse subscribes section → Validate callbacks → Set up queues
    ↓
Register with message bus (when available)
    ↓
Node ready for messaging
```

### 2. Runtime Message Handling

```
Message arrives
    ↓
_handle_message() called
    ↓
Look up callback in message_callbacks
    ↓
Get callback method by name
    ↓
Call callback method with message
    ↓
Message processed
```

### 3. Publishing Validation

```
publish() called
    ↓
Check if topic in published_topics
    ↓
If not declared → Log warning and return False
    ↓
If declared → Create message and send to bus
    ↓
Message published
```

## Configuration Examples

### Speech Recognition Node

```yaml
# nodes/speech_recognition/.messages
publishes:
  - topic: "prompt_audio"
    message_type: "PromptAudio"
    schema:
      text: str
      confidence: float
      audio_data: Optional[bytes]
      timestamp: float
      language: str
      
  - topic: "listening_status"
    message_type: "ListeningStatus"
    schema:
      listening: bool
      paused: bool
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

### Speech Synthesis Node

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

### AI Cognition Node

```yaml
# nodes/ai_cognition/.messages
publishes:
  - topic: "text_response"
    message_type: "TextResponse"
    schema:
      text: str
      voice: str
      priority: int
      timestamp: float
      response_id: str
      
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

## Implementation Details

### 1. init_messages() Method

```python
def init_messages(self):
    """Initialize messaging based on .messages configuration file."""
    try:
        if not self.config:
            self.logger.warning(f"No .messages configuration found for {self.node_name}")
            return
        
        # Set up publishers
        publishes = self.config.get('publishes', [])
        for pub_config in publishes:
            topic = pub_config.get('topic')
            if topic:
                self.published_topics.add(topic)
                self.logger.debug(f"Configured to publish to topic: {topic}")
        
        # Set up subscribers
        subscribes = self.config.get('subscribes', [])
        for sub_config in subscribes:
            topic = sub_config.get('topic')
            callback_name = sub_config.get('callback')
            
            if topic and callback_name:
                # Verify callback method exists
                if hasattr(self, callback_name):
                    self.subscribed_topics.add(topic)
                    self.message_callbacks[topic] = callback_name
                    
                    # Create message queue for this topic
                    self.message_queues[topic] = queue.Queue(maxsize=100)
                    
                    self.logger.debug(f"Configured to subscribe to topic: {topic} -> {callback_name}")
                else:
                    self.logger.error(f"Callback method '{callback_name}' not found for topic '{topic}'")
        
        # Register with message bus when it becomes available
        if self.message_bus:
            self._register_with_message_bus()
        
        self.logger.info(f"Messaging initialized: {len(self.published_topics)} publishers, {len(self.subscribed_topics)} subscribers")
        
    except Exception as e:
        self.logger.error(f"Error initializing messages: {e}")
```

### 2. Enhanced publish() Method

```python
def publish(self, topic: str, data: Any):
    """
    Publish message to topic.
    
    Note: Topic must be declared in .messages file under 'publishes' section.
    """
    try:
        # Verify topic is declared for publishing
        if topic not in self.published_topics:
            self.logger.warning(f"Topic '{topic}' not declared in .messages file for publishing")
            return False
        
        message = Message(
            topic=topic,
            data=data,
            timestamp=time.time(),
            source_node=self.node_name,
            message_id=f"{self.node_name}_{time.time()}"
        )
        
        if self.message_bus:
            return self.message_bus.publish(message)
        else:
            self.logger.warning(f"No message bus available for publishing to {topic}")
            return False
            
    except Exception as e:
        self.logger.error(f"Error publishing to topic {topic}: {e}")
        return False
```

### 3. Message Bus Registration

```python
def _register_with_message_bus(self):
    """Register subscriptions with the message bus"""
    try:
        for topic in self.subscribed_topics:
            if topic in self.message_queues:
                self.message_bus.subscribe(self.node_name, topic, self.message_queues[topic])
                self.logger.debug(f"Registered subscription for topic: {topic}")
    except Exception as e:
        self.logger.error(f"Error registering with message bus: {e}")
```

## Migration Guide

### From v2.0 to v3.0

1. **Create .messages file** for each node
2. **Move subscribe() calls** from `initialize()` to `.messages` file
3. **Declare publish topics** in `.messages` file
4. **Remove manual subscribe() calls** from node code
5. **Keep callback methods** unchanged

### Example Migration

**Before (v2.0):**
```python
def initialize(self):
    self.audio_input = AudioInput()
    self.subscribe("speaking_status", self.on_speaking_status_change)
    self.subscribe("system_mode", self.on_system_mode_change)
```

**After (v3.0):**
```python
def initialize(self):
    self.audio_input = AudioInput()
    # Subscriptions now configured automatically via .messages file
```

**New .messages file:**
```yaml
subscribes:
  - topic: "speaking_status"
    callback: "on_speaking_status_change"
  - topic: "system_mode"
    callback: "on_system_mode_change"
```

## Benefits Summary

### 1. **Reduced Complexity**
- No manual subscribe/publish setup code
- Single point of messaging configuration
- Automatic validation and error handling

### 2. **Improved Maintainability**
- Clear messaging topology in configuration files
- Easy to modify messaging without code changes
- Centralized message schema documentation

### 3. **Enhanced Reliability**
- Automatic callback validation during initialization
- Consistent message handling patterns
- Early detection of configuration errors

### 4. **Better Documentation**
- Message schemas defined in configuration
- Clear publisher/subscriber relationships
- Self-documenting messaging architecture

## Testing Strategy

### 1. Configuration Validation Tests
```python
def test_init_messages_validates_callbacks():
    # Test that missing callback methods are detected
    
def test_init_messages_sets_up_queues():
    # Test that message queues are created for subscriptions
    
def test_publish_validates_declared_topics():
    # Test that undeclared topics are rejected
```

### 2. Integration Tests
```python
def test_declarative_messaging_flow():
    # Test complete message flow using declarative configuration
    
def test_message_bus_registration():
    # Test automatic registration with message bus
```

### 3. Migration Tests
```python
def test_v2_to_v3_compatibility():
    # Test that v3 nodes work with existing message patterns
```

## Architectural Inconsistency Resolution

### The Original Problem

The documentation showed an apparent inconsistency:
1. **Declarative messaging** via `.messages` files (handled by [`NevilNode.init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315))
2. **Manual subscribe/unsubscribe methods** in [`MessageBus`](docs/nevil_v3/02_node_structure_threading_model.md:526) class

### The Resolution

**The design is architecturally sound** - it uses a **two-layer approach**:

#### **Layer 1: Developer Interface (Declarative)**
- Developers only work with `.messages` configuration files
- No manual [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) calls in node code
- Simple [`publish()`](docs/nevil_v3/02_node_structure_threading_model.md:374) calls with automatic validation

#### **Layer 2: Framework Implementation (Programmatic)**
- Framework reads `.messages` files during [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315)
- Framework calls [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) internally via [`_register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364)
- MessageBus provides traditional pub/sub infrastructure for the framework

### Key Clarifications

#### **Who Calls MessageBus Methods?**
- **NOT node developers** - they never call [`subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) or [`unsubscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:562)
- **The framework calls them** - specifically [`NevilNode._register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364)

#### **When Are They Called?**
- **During node initialization** - after [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) processes the `.messages` file
- **During node shutdown** - for cleanup and resource management

#### **How Does Declarative Integration Work?**
1. Node reads `.messages` file → [`_load_config()`](docs/nevil_v3/02_node_structure_threading_model.md:115)
2. [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) processes configuration
3. Framework calls [`_register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364)
4. This calls [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) for each configured topic

### Design Benefits

#### **True Declarative Experience**
- Node developers work purely with configuration files
- No messaging infrastructure code in business logic
- Framework handles all complexity transparently

#### **Robust Infrastructure**
- MessageBus retains full programmatic capability for framework needs
- Thread-safe operations with proper error handling
- Flexible enough to support future messaging patterns

#### **Clear Separation of Concerns**
- **Configuration**: What messages flow where (`.messages` files)
- **Implementation**: How messages are processed (callback methods)
- **Infrastructure**: How messages are routed (MessageBus internals)

## Conclusion

The declarative messaging architecture in Nevil v3.0 significantly simplifies node development by:

- **Eliminating manual subscribe/publish setup code**
- **Centralizing messaging configuration in .messages files**
- **Providing automatic validation and error handling**
- **Maintaining the v3.0 philosophy of "simple architecture = working robot"**
- **Using a clean two-layer architecture that separates developer concerns from framework implementation**

This hybrid approach provides the best of both worlds: **declarative simplicity for developers** and **programmatic flexibility for the framework**, making the system easier to understand, maintain, and extend while maintaining robust messaging infrastructure.