
# Nevil v3.0 Node Structure and Threading Model

## Overview

This document defines the detailed node structure and threading model for Nevil v3.0, focusing on simplicity, reliability, and maintainability while ensuring thread-safe operations and proper resource management.

## 1. Node Architecture

### 1.1 Base Node Class Structure

#### Code Summary

This section implements the complete NevilNode base class framework with threading, messaging, and lifecycle management:

**Classes:**
- **NodeStatus** (Enum): Node lifecycle states (INITIALIZING, RUNNING, STOPPING, STOPPED, ERROR)
- **Message** (Dataclass): Message structure with topic, data, timestamp, source_node, message_id
- **NevilNode** (ABC): Complete base class for all nodes

**Key Methods in NevilNode:**
- `__init__()`: Initialize node with threading components, message system, configuration loading, logging setup, and declarative messaging
- `start()`: Start node with all threads (main, message, heartbeat)
- `stop()`: Graceful shutdown with timeout handling
- `_main_loop()`: Main processing loop with error handling and heartbeat updates
- `_message_loop()`: Message processing loop handling all subscribed topics
- `_handle_message()`: Route messages to declaratively configured callbacks
- `_heartbeat_loop()`: Send periodic system health heartbeats
- `init_messages()`: Declarative messaging setup from .messages configuration file
- `publish()`: Publish messages with topic validation
- `_load_config()`: Load YAML configuration from .messages file
- `_setup_logging()`: Configure node-specific logging with file handlers

```python
# nevil_framework/base_node.py

import threading
import multiprocessing
import queue
import time
import logging
import signal
import os
import yaml
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Any, Callable, Optional
from enum import Enum

class NodeStatus(Enum):
    INITIALIZING = "initializing"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"

@dataclass
class Message:
    topic: str
    data: Any
    timestamp: float
    source_node: str
    message_id: str

class NevilNode(ABC):
    """
    Base class for all Nevil v3.0 nodes.
    
    Provides:
    - Declarative message setup via .messages files
    - Thread-safe message handling
    - Logging infrastructure
    - Configuration management
    - Graceful shutdown handling
    - Health monitoring
    """
    
    def __init__(self, node_name: str, config_path: str = None):
        self.node_name = node_name
        self.config_path = config_path or f"nodes/{node_name}/.messages"
        self.status = NodeStatus.INITIALIZING
        
        # Threading components
        self.main_thread = None
        self.message_thread = None
        self.worker_threads = []
        self.shutdown_event = threading.Event()
        self.thread_lock = threading.RLock()
        
        # Message system
        self.message_queues = {}  # topic -> queue
        self.message_callbacks = {}  # topic -> callback method name
        self.published_topics = set()   # topics this node publishes
        self.subscribed_topics = set()  # topics this node subscribes to
        self.message_bus = None   # Set by framework
        
        # Configuration and logging
        self.config = self._load_config()
        self.logger = self._setup_logging()
        
        # Health monitoring
        self.last_heartbeat = time.time()
        self.error_count = 0
        self.max_errors = 10
        
        # Signal handlers
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Declarative messaging setup - runs once during initialization
        self.init_messages()
        
        self.logger.info(f"Node {self.node_name} initialized with messaging configured")
    
    def _load_config(self) -> Dict[str, Any]:
        """Load node configuration from .messages file"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    return yaml.safe_load(f)
            return {}
        except Exception as e:
            print(f"Error loading config for {self.node_name}: {e}")
            return {}
    
    def _setup_logging(self) -> logging.Logger:
        """Setup node-specific logging"""
        logger = logging.getLogger(self.node_name)
        logger.setLevel(logging.DEBUG)
        
        # Create logs directory
        log_dir = f"logs/nodes/{self.node_name}"
        os.makedirs(log_dir, exist_ok=True)
        
        # File handler for node logs
        file_handler = logging.FileHandler(f"{log_dir}/{self.node_name}.log")
        file_handler.setLevel(logging.INFO)
        
        # Debug file handler
        debug_handler = logging.FileHandler(f"{log_dir}/{self.node_name}.debug.log")
        debug_handler.setLevel(logging.DEBUG)
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        
        # Formatter
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)s] [%(name)s] [%(threadName)s] %(message)s'
        )
        file_handler.setFormatter(formatter)
        debug_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        logger.addHandler(file_handler)
        logger.addHandler(debug_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def start(self):
        """Start the node and all its threads"""
        try:
            self.status = NodeStatus.RUNNING
            self.logger.info(f"Starting node {self.node_name}")
            
            # Initialize node-specific components
            self.initialize()
            
            # Start message handling thread
            self.message_thread = threading.Thread(
                target=self._message_loop,
                name=f"{self.node_name}_messages",
                daemon=False
            )
            self.message_thread.start()
            
            # Start main processing thread
            self.main_thread = threading.Thread(
                target=self._main_loop,
                name=f"{self.node_name}_main",
                daemon=False
            )
            self.main_thread.start()
            
            # Start heartbeat thread
            heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                name=f"{self.node_name}_heartbeat",
                daemon=True
            )
            heartbeat_thread.start()
            
            self.logger.info(f"Node {self.node_name} started successfully")
            
        except Exception as e:
            self.status = NodeStatus.ERROR
            self.logger.error(f"Failed to start node {self.node_name}: {e}")
            raise
    
    def stop(self, timeout: float = 10.0):
        """Stop the node gracefully"""
        self.logger.info(f"Stopping node {self.node_name}")
        self.status = NodeStatus.STOPPING
        
        # Signal shutdown
        self.shutdown_event.set()
        
        # Stop node-specific components
        try:
            self.cleanup()
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        
        # Wait for threads to finish
        threads_to_join = [self.main_thread, self.message_thread] + self.worker_threads
        
        for thread in threads_to_join:
            if thread and thread.is_alive():
                thread.join(timeout=timeout/len(threads_to_join))
                if thread.is_alive():
                    self.logger.warning(f"Thread {thread.name} did not stop gracefully")
        
        self.status = NodeStatus.STOPPED
        self.logger.info(f"Node {self.node_name} stopped")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info(f"Received signal {signum}, initiating shutdown")
        self.stop()
    
    def _main_loop(self):
        """Main processing loop - implemented by subclasses"""
        try:
            while not self.shutdown_event.is_set():
                try:
                    # Call subclass main loop
                    self.main_loop()
                    
                    # Update heartbeat
                    self.last_heartbeat = time.time()
                    
                except Exception as e:
                    self.error_count += 1
                    self.logger.error(f"Error in main loop: {e}")
                    
                    if self.error_count >= self.max_errors:
                        self.logger.critical(f"Too many errors ({self.error_count}), stopping node")
                        self.status = NodeStatus.ERROR
                        break
                    
                    # Brief pause before retry
                    time.sleep(1.0)
                
        except Exception as e:
            self.logger.critical(f"Critical error in main loop: {e}")
            self.status = NodeStatus.ERROR
    
    def _message_loop(self):
        """Message handling loop"""
        while not self.shutdown_event.is_set():
            try:
                # Process incoming messages
                for topic, message_queue in self.message_queues.items():
                    try:
                        message = message_queue.get_nowait()
                        self._handle_message(topic, message)
                    except queue.Empty:
                        continue
                    except Exception as e:
                        self.logger.error(f"Error processing message on topic {topic}: {e}")
                
                # Brief pause to prevent busy waiting
                time.sleep(0.01)
                
            except Exception as e:
                self.logger.error(f"Error in message loop: {e}")
                time.sleep(0.1)
    
    def _handle_message(self, topic: str, message: Message):
        """Handle incoming message using declaratively configured callbacks"""
        try:
            if topic in self.message_callbacks:
                callback_name = self.message_callbacks[topic]
                callback_method = getattr(self, callback_name, None)
                
                if callback_method and callable(callback_method):
                    callback_method(message)
                else:
                    self.logger.error(f"Callback method '{callback_name}' not found or not callable for topic {topic}")
            else:
                self.logger.warning(f"No callback configured for topic {topic}")
        except Exception as e:
            self.logger.error(f"Error in message callback for topic {topic}: {e}")
    
    def _heartbeat_loop(self):
        """Send periodic heartbeat messages"""
        while not self.shutdown_event.is_set():
            try:
                heartbeat_data = {
                    "node_name": self.node_name,
                    "status": self.status.value,
                    "timestamp": time.time(),
                    "error_count": self.error_count,
                    "thread_count": threading.active_count()
                }
                
                self.publish("system_heartbeat", heartbeat_data)
                time.sleep(5.0)  # Heartbeat every 5 seconds
                
            except Exception as e:
                self.logger.error(f"Error sending heartbeat: {e}")
                time.sleep(5.0)
    
    def init_messages(self):
        """
        Initialize messaging based on .messages configuration file.
        
        This method runs once during __init__ and automatically sets up all
        publishers and subscribers for the life of the node based on the
        declarative configuration in the .messages file.
        """
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
    
    def _register_with_message_bus(self):
        """Register subscriptions with the message bus"""
        try:
            for topic in self.subscribed_topics:
                if topic in self.message_queues:
                    self.message_bus.subscribe(self.node_name, topic, self.message_queues[topic])
                    self.logger.debug(f"Registered subscription for topic: {topic}")
        except Exception as e:
            self.logger.error(f"Error registering with message bus: {e}")
    
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
    
    # Abstract methods to be implemented by subclasses
    @abstractmethod
    def initialize(self):
        """Initialize node-specific components"""
        pass
    
    @abstractmethod
    def main_loop(self):
        """Main processing loop - called repeatedly"""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Cleanup node-specific resources"""
        pass
```

### 1.2 Node Directory Structure

```
nevil_v3/
├── .nodes                          # Root configuration file
├── framework/                # Core framework
│   ├── __init__.py
│   ├── base_node.py               # Base node class
│   ├── message_bus.py             # Message system
│   ├── launcher.py                # Node launcher
│   └── logger.py                  # Logging utilities
├── nodes/                         # Node implementations
│   ├── speech_recognition/
│   │   ├── __init__.py
│   │   ├── speech_recognition_node.py
│   │   ├── .messages              # Node message config
│   │   └── logs/                  # Node-specific logs
│   ├── speech_synthesis/
│   │   ├── __init__.py
│   │   ├── speech_synthesis_node.py
│   │   ├── .messages
│   │   └── logs/
│   └── ai_cognition/
│       ├── __init__.py
│       ├── ai_cognition_node.py
│       ├── .messages
│       └── logs/
├── audio/                         # v1.0 audio components
│   ├── __init__.py
│   ├── audio_input.py            # Microphone handling
│   ├── audio_output.py           # Speaker handling
│   └── audio_utils.py            # Utility functions
├── logs/                         # System logs
│   ├── system.log
│   └── nodes/                    # Node logs (symlinks)
├── docs/                         # documentation 
└── tests/                        # Test suite
    ├── test_framework.py
    ├── test_nodes.py
    └── test_audio.py
```

## 2. Threading Model

### 2.1 Process-Level Architecture

```python
# Process hierarchy
Main Process (Launcher)
├── Speech Recognition Process
│   ├── Main Thread (audio processing)

├── Speech Synthesis Process
│   ├── Main Thread (TTS processing)

└── AI Cognition Process
    ├── Main Thread (AI processing)

```

### 2.2 Inter-Process Communication

#### Code Summary

This section implements the MessageBus class for inter-node communication using multiprocessing queues:

**Classes:**
- **MessageBus**: Central message routing hub for inter-node communication

**Key Methods in MessageBus:**
- `__init__()`: Initialize message bus with topic tracking, publisher/subscriber management, and statistics
- `create_topic()`: Create new message topic
- `subscribe()`: Register node subscription to topic with message queue (**Called internally by framework**)
- `unsubscribe()`: Remove node subscription from topic (**Called internally by framework**)
- `publish()`: Broadcast message to all topic subscribers with error handling
- `get_stats()`: Return message bus statistics (message count, error count, topic count, subscriber count)

**Key Features:**
- Thread-safe operations with RLock
- Non-blocking message delivery with queue overflow handling
- Publisher and subscriber tracking per node
- Message statistics collection
- Error handling for full queues

#### Declarative Messaging Integration

**IMPORTANT**: The `subscribe()` and `unsubscribe()` methods in MessageBus are **NOT called directly by node developers**. They are internal framework methods called automatically during the declarative messaging setup process:

1. **Node reads `.messages` file** → [`NevilNode.init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315)
2. **Framework parses configuration** → Sets up `subscribed_topics` and `message_queues`
3. **Framework calls MessageBus methods** → [`_register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364) calls `subscribe()`
4. **Node is ready for messaging** → All subscriptions active, no manual calls needed

This maintains the declarative philosophy while using programmatic methods internally for actual message bus operations.

```python
# nevil_framework/message_bus.py

import multiprocessing
import queue
import threading
import time
from typing import Dict, Set
from dataclasses import dataclass

class MessageBus:
    """
    Simple message bus for inter-node communication.
    Uses multiprocessing queues for reliability and isolation.
    """
    
    def __init__(self):
        self.topics = {}  # topic -> set of subscriber queues
        self.publishers = {}  # node_name -> set of topics
        self.subscribers = {}  # node_name -> {topic -> queue}
        self.message_lock = threading.RLock()
        
        # Statistics
        self.message_count = 0
        self.error_count = 0
        
    def create_topic(self, topic: str):
        """Create a new topic"""
        with self.message_lock:
            if topic not in self.topics:
                self.topics[topic] = set()
    
    def subscribe(self, node_name: str, topic: str, message_queue: queue.Queue):
        """Subscribe a node to a topic"""
        with self.message_lock:
            # Create topic if it doesn't exist
            self.create_topic(topic)
            
            # Add queue to topic subscribers
            self.topics[topic].add(message_queue)
            
            # Track subscription
            if node_name not in self.subscribers:
                self.subscribers[node_name] = {}
            self.subscribers[node_name][topic] = message_queue
    
    def unsubscribe(self, node_name: str, topic: str):
        """Unsubscribe a node from a topic"""
        with self.message_lock:
            if (node_name in self.subscribers and 
                topic in self.subscribers[node_name]):
                
                message_queue = self.subscribers[node_name][topic]
                
                # Remove from topic
                if topic in self.topics:
                    self.topics[topic].discard(message_queue)
                
                # Remove from node tracking
                del self.subscribers[node_name][topic]
    
    def publish(self, message: Message):
        """Publish message to all subscribers of the topic"""
        try:
            with self.message_lock:
                topic = message.topic
                
                if topic in self.topics:
                    # Send to all subscribers
                    for subscriber_queue in self.topics[topic]:
                        try:
                            # Non-blocking put with timeout
                            subscriber_queue.put_nowait(message)
                        except queue.Full:
                            # Log dropped message
                            print(f"Warning: Message queue full for topic {topic}")
                            self.error_count += 1
                
                self.message_count += 1
                
        except Exception as e:
            print(f"Error publishing message to topic {message.topic}: {e}")
            self.error_count += 1
    
    def get_stats(self) -> Dict[str, Any]:
        """Get message bus statistics"""
        with self.message_lock:
            return {
                "message_count": self.message_count,
                "error_count": self.error_count,
                "topic_count": len(self.topics),
                "subscriber_count": sum(len(subs) for subs in self.subscribers.values()),
                "topics": list(self.topics.keys())
            }
```

### 2.3 Declarative Messaging Flow Architecture

#### 2.3.1 The Declarative → Programmatic Bridge

**ARCHITECTURAL CLARIFICATION**: The Nevil v3.0 messaging system uses a **hybrid approach** that appears declarative to developers but uses programmatic methods internally:

#### **Developer Experience (Declarative)**
- Developers only configure `.messages` files
- No manual `subscribe()` or `publish()` calls in node code
- Framework handles all messaging setup automatically

#### **Internal Implementation (Programmatic)**
- Framework reads `.messages` files during [`NevilNode.__init__()`](docs/nevil_v3/02_node_structure_threading_model.md:78)
- Framework calls [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) internally
- MessageBus uses traditional pub/sub methods for actual message routing

#### 2.3.2 Complete Messaging Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    DECLARATIVE MESSAGING FLOW                   │
└─────────────────────────────────────────────────────────────────┘

1. NODE INITIALIZATION
   ┌─────────────────┐
   │ Node Created    │
   │ __init__()      │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Load .messages  │
   │ config file     │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ init_messages() │
   │ called          │
   └─────────┬───────┘
             │
             ▼

2. CONFIGURATION PROCESSING
   ┌─────────────────┐
   │ Parse publishes │
   │ section         │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Add topics to   │
   │ published_topics│
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Parse subscribes│
   │ section         │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Validate        │
   │ callback methods│
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Create message  │
   │ queues          │
   └─────────┬───────┘
             │
             ▼

3. FRAMEWORK REGISTRATION (INTERNAL)
   ┌─────────────────┐
   │ _register_with_ │
   │ message_bus()   │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ FOR EACH TOPIC: │
   │ MessageBus.     │
   │ subscribe()     │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Node ready for  │
   │ messaging       │
   └─────────────────┘

4. RUNTIME OPERATION
   ┌─────────────────┐
   │ Messages arrive │
   │ in queues       │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ _handle_message │
   │ routes to       │
   │ callbacks       │
   └─────────┬───────┘
             │
             ▼
   ┌─────────────────┐
   │ Node publishes  │
   │ using publish() │
   │ (validated)     │
   └─────────────────┘
```

#### 2.3.3 Key Architectural Points

##### **MessageBus Methods Are Framework-Internal**
- [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) is called by [`NevilNode._register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364)
- [`MessageBus.unsubscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:562) is called during node shutdown
- **Node developers never call these methods directly**

##### **Declarative Configuration Drives Everything**
- `.messages` files are the **single source of truth** for messaging topology
- [`NevilNode.init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) reads configuration and sets up internal state
- Framework automatically calls MessageBus methods based on configuration

##### **Two-Layer Architecture**
```
┌─────────────────────────────────────────────────────────────┐
│                    DEVELOPER LAYER                          │
│  • .messages files (declarative)                           │
│  • Node callback methods                                   │
│  • publish() calls (validated against .messages)          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   FRAMEWORK LAYER                          │
│  • init_messages() processing                              │
│  • MessageBus.subscribe() calls                           │
│  • Message routing and delivery                           │
│  • Queue management                                       │
└─────────────────────────────────────────────────────────────┘
```

#### 2.3.4 Why This Design Works

##### **Separation of Concerns**
- **Developers**: Focus on business logic and message handling
- **Framework**: Handles complex messaging infrastructure

##### **Configuration Validation**
- Callback methods validated during [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315)
- Publishing topics validated during [`publish()`](docs/nevil_v3/02_node_structure_threading_model.md:374)
- Early error detection prevents runtime messaging failures

##### **Flexibility with Simplicity**
- MessageBus retains full programmatic capability for framework use
- Nodes get simple declarative interface
- System can evolve without breaking node implementations

#### 2.3.5 Answering the Key Questions

**Q: Should MessageBus have subscribe/unsubscribe methods if messaging is declarative?**
**A: YES** - These methods are essential for the framework layer. They provide the programmatic interface that the declarative system uses internally.

**Q: Who calls these methods and when?**
**A: The framework calls them** - Specifically [`NevilNode._register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364) calls [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548) during node initialization.

**Q: How does the declarative .messages file reading integrate with MessageBus?**
**A: Through automatic conversion** - [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) reads `.messages` files and translates them into MessageBus method calls.

**Q: What's the actual flow: Node reads .messages → calls MessageBus.subscribe()?**
**A: Exactly** - Node reads `.messages` → [`init_messages()`](docs/nevil_v3/02_node_structure_threading_model.md:315) processes config → [`_register_with_message_bus()`](docs/nevil_v3/02_node_structure_threading_model.md:364) calls [`MessageBus.subscribe()`](docs/nevil_v3/02_node_structure_threading_model.md:548)

### 2.4 Thread Safety Patterns

#### Code Summary

This section implements thread safety patterns for resource protection and producer-consumer operations:

**Classes:**
- **ThreadSafeResource**: Pattern for protecting shared resources with initialization and cleanup
- **ThreadSafeQueue**: Enhanced queue with statistics and timeout handling

**Key Methods in ThreadSafeResource:**
- `__init__()`: Initialize with RLock and resource state tracking
- `initialize()`: Thread-safe resource initialization
- `use_resource()`: Execute operations on protected resource
- `cleanup()`: Thread-safe resource cleanup and disposal

**Key Methods in ThreadSafeQueue:**
- `__init__()`: Initialize queue with size limits and statistics tracking
- `put()`: Add item with timeout and drop count tracking
- `get()`: Retrieve item with timeout handling
- `get_stats()`: Return queue statistics (size, put/get/drop counts)

**Key Features:**
- RLock-based resource protection
- Statistics collection for monitoring
- Timeout handling for operations
- Producer-consumer pattern implementation

#### 2.3.1 Resource Protection
```python
class ThreadSafeResource:
    """Pattern for protecting shared resources"""
    
    def __init__(self):
        self._lock = threading.RLock()
        self._resource = None
        self._initialized = False
    
    def initialize(self):
        with self._lock:
            if not self._initialized:
                self._resource = self._create_resource()
                self._initialized = True
    
    def use_resource(self, operation):
        with self._lock:
            if self._initialized:
                return operation(self._resource)
            else:
                raise RuntimeError("Resource not initialized")
    
    def cleanup(self):
        with self._lock:
            if self._initialized and self._resource:
                self._cleanup_resource(self._resource)
                self._resource = None
                self._initialized = False
```

#### 2.3.2 Producer-Consumer Pattern
```python
class ThreadSafeQueue:
    """Thread-safe queue with timeout and size limits"""
    
    def __init__(self, maxsize: int = 100):
        self._queue = queue.Queue(maxsize=maxsize)
        self._stats_lock = threading.Lock()
        self._put_count = 0
        self._get_count = 0
        self._drop_count = 0
    
    def put(self, item, timeout: float = 1.0) -> bool:
        try:
            self._queue.put(item, timeout=timeout)
            with self._stats_lock:
                self._put_count += 1
            return True
        except queue.Full:
            with self._stats_lock:
                self._drop_count += 1
            return False
    
    def get(self, timeout: float = 1.0):
        try:
            item = self._queue.get(timeout=timeout)
            with self._stats_lock:
                self._get_count += 1
            return item
        except queue.Empty:
            return None
    
    def get_stats(self):
        with self._stats_lock:
            return {
                "size": self._queue.qsize(),
                "put_count": self._put_count,
                "get_count": self._get_count,
                "drop_count": self._drop_count
            }
```

## 3. Node Implementation Examples

### 3.1 Speech Recognition Node

#### Code Summary

This section implements the SpeechRecognitionNode class using v1.0 audio components with threading and declarative messaging:

**Classes:**
- **SpeechRecognitionNode**: Converts voice input to text commands using OpenAI Whisper

**Key Methods in SpeechRecognitionNode:**
- `__init__()`: Initialize node with audio input tracking and listening state
- `initialize()`: Set up audio input using v1.0 AudioInput component
- `main_loop()`: Main processing loop for speech recognition with non-blocking audio capture
- `_process_audio()`: Process captured audio in separate thread using v1.0 recognition pipeline
- `on_system_mode_change()`: Handle system mode changes for listening control
- `on_speaking_status_change()`: Pause listening when speech synthesis is active
- `start_listening()`: Enable speech recognition and audio capture
- `stop_listening()`: Disable speech recognition and audio capture
- `cleanup()`: Clean up audio resources and stop listening

**Key Features:**
- Non-blocking audio processing with timeouts
- Thread-safe audio processing in separate threads
- Integration with v1.0 proven audio components
- Declarative message handling (no manual subscribe calls)
- Automatic listening control based on system state
- Voice command publishing with confidence scores

```python
# nodes/speech_recognition/speech_recognition_node.py

import time
import threading
from nevil_framework.base_node import NevilNode
from audio.audio_input import AudioInput

class SpeechRecognitionNode(NevilNode):
    """
    Speech recognition node using v1.0 proven audio components.
    Converts voice input to text commands.
    """
    
    def __init__(self):
        super().__init__("speech_recognition")
        self.audio_input = None
        self.listening = False
        self.audio_thread = None
        
    def initialize(self):
        """Initialize speech recognition components"""
        try:
            # Initialize audio input using v1.0 components
            self.audio_input = AudioInput()
            
            # Note: No manual subscribe() calls needed!
            # All subscriptions are automatically configured via .messages file
            # during init_messages() in __init__
            
            self.logger.info("Speech recognition initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize speech recognition: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        if self.listening and self.audio_input:
            try:
                # Listen for speech (non-blocking with timeout)
                audio_data = self.audio_input.listen_for_speech(timeout=1.0)
                
                if audio_data:
                    # Process audio in separate thread to avoid blocking
                    audio_thread = threading.Thread(
                        target=self._process_audio,
                        args=(audio_data,),
                        daemon=True
                    )
                    audio_thread.start()
                    
            except Exception as e:
                self.logger.error(f"Error in speech recognition: {e}")
        
        # Brief pause to prevent busy waiting
        time.sleep(0.1)
    
    def _process_audio(self, audio_data):
        """Process audio data in separate thread"""
        try:
            # Convert speech to text
            text = self.audio_input.recognize_speech(audio_data)
            
            if text:
                # Publish voice command
                command_data = {
                    "text": text,
                    "confidence": 0.8,  # Would come from recognition engine
                    "timestamp": time.time()
                }
                
                self.publish("voice_command", command_data)
                self.logger.info(f"Recognized: {text}")
                
        except Exception as e:
            self.logger.error(f"Error processing audio: {e}")
    
    def on_system_mode_change(self, message):
        """Handle system mode changes"""
        mode = message.data.get("mode")
        
        if mode == "listening":
            self.start_listening()
        elif mode == "speaking":
            self.stop_listening()
    
    def on_speaking_status_change(self, message):
        """Handle speaking status changes"""
        is_speaking = message.data.get("speaking", False)
        
        if is_speaking:
            self.stop_listening()
        else:
            self.start_listening()
    
    def start_listening(self):
        """Start listening for speech"""
        if not self.listening:
            self.listening = True
            self.logger.info("Started listening")
    
    def stop_listening(self):
        """Stop listening for speech"""
        if self.listening:
            self.listening = False
            self.logger.info("Stopped listening")
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop_listening()
        
        if self.audio_input:
            self.audio_input.cleanup()
            self.audio_input = None
        
        self.logger.info("Speech recognition cleaned up")

# TEST: Speech recognition node initializes without errors
# TEST: Audio input is properly configured with v1.0 settings
# TEST: Voice commands are published with correct format
# TEST: Node responds to system mode changes
# TEST: Graceful shutdown releases audio resources
```

### 3.2 Speech Synthesis Node

#### Code Summary

This section implements the SpeechSynthesisNode class using v1.0 audio components with queuing, threading, and status management:

**Classes:**
- **SpeechSynthesisNode**: Converts text to speech output using OpenAI TTS and v1.0 audio pipeline

**Key Methods in SpeechSynthesisNode:**
- `__init__()`: Initialize node with audio output, speech queue, and thread synchronization
- `initialize()`: Set up audio output using v1.0 AudioOutput component
- `main_loop()`: Main processing loop handling speech queue with non-blocking operations
- `on_text_response()`: Handle incoming text response messages with priority queuing
- `_speak_text()`: Thread-safe text-to-speech processing with status publishing
- `cleanup()`: Clean up speech queue and audio output resources

**Key Features:**
- Priority-based speech queue with overflow handling
- Thread-safe speech processing with locking
- Integration with v1.0 proven audio output pipeline
- Speaking status broadcasting for coordination
- Declarative message handling (no manual subscribe calls)
- Voice and speed control from message parameters
- Queue management with configurable size limits

```python
# nodes/speech_synthesis/speech_synthesis_node.py

import time
import queue
import threading
from nevil_framework.base_node import NevilNode
from audio.audio_output import AudioOutput

class SpeechSynthesisNode(NevilNode):
    """
    Speech synthesis node using v1.0 proven audio components.
    Converts text to speech output.
    """
    
    def __init__(self):
        super().__init__("speech_synthesis")
        self.audio_output = None
        self.speech_queue = queue.Queue(maxsize=10)
        self.currently_speaking = False
        self.speech_lock = threading.Lock()
        
    def initialize(self):
        """Initialize speech synthesis components"""
        try:
            # Initialize audio output using v1.0 components
            self.audio_output = AudioOutput()
            
            # Note: No manual subscribe() calls needed!
            # All subscriptions are automatically configured via .messages file
            # during init_messages() in __init__
            
            self.logger.info("Speech synthesis initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize speech synthesis: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        try:
            # Process speech queue
            try:
                speech_data = self.speech_queue.get_nowait()
                self._speak_text(speech_data)
            except queue.Empty:
                pass
            
        except Exception as e:
            self.logger.error(f"Error in speech synthesis: {e}")
        
        # Brief pause
        time.sleep(0.1)
    
    def on_text_response(self, message):
        """Handle text response messages"""
        try:
            text = message.data.get("text", "")
            voice = message.data.get("voice", "onyx")
            priority = message.data.get("priority", 100)
            
            if text:
                speech_data = {
                    "text": text,
                    "voice": voice,
                    "priority": priority,
                    "timestamp": time.time()
                }
                
                # Add to speech queue
                try:
                    self.speech_queue.put_nowait(speech_data)
                except queue.Full:
                    self.logger.warning("Speech queue full, dropping message")
                    
        except Exception as e:
            self.logger.error(f"Error handling text response: {e}")
    
    def _speak_text(self, speech_data):
        """Speak text using audio output"""
        with self.speech_lock:
            try:
                self.currently_speaking = True
                
                # Publish speaking status
                self.publish("speaking_status", {"speaking": True})
                
                # Speak using v1.0 audio output
                text = speech_data["text"]
                voice = speech_data.get("voice", "onyx")
                
                self.logger.info(f"Speaking: {text}")
                self.audio_output.speak_text(text, voice=voice, wait=True)
                
            except Exception as e:
                self.logger.error(f"Error speaking text: {e}")
            finally:
                self.currently_speaking = False
                
                # Publish speaking status
                self.publish("speaking_status", {"speaking": False})
    
    def cleanup(self):
        """Cleanup resources"""
        # Clear speech queue
        while not self.speech_queue.empty():
            try:
                self.speech_queue.get_nowait()
            except queue.Empty:
                break
        
        if self.audio_output:
            self.audio_output.cleanup()
            self.audio_output = None
        
        self.logger.info("Speech synthesis cleaned up")

# TEST: Speech synthesis node initializes without errors
# TEST: Text responses are queued and processed in order
# TEST: Audio output uses v1.0 proven pipeline
# TEST: Speaking status is published correctly
# TEST: Queue overflow is handled gracefully
```

### 3.3 AI Cognition Node

#### Code Summary

This section implements the AICognitionNode class with OpenAI integration, conversation history, and request queuing:

**Classes:**
- **AICognitionNode**: AI-powered conversation and decision making using OpenAI GPT

**Key Methods in AICognitionNode:**
- `__init__()`: Initialize node with OpenAI client, conversation history, and processing queue
- `initialize()`: Set up OpenAI client with API key validation
- `main_loop()`: Main processing loop handling AI requests with non-blocking queue operations
- `on_voice_command()`: Handle voice command messages with confidence threshold filtering
- `_process_ai_request()`: Process AI requests with context management and response generation
- `_generate_response()`: Generate AI responses using OpenAI chat completions API
- `cleanup()`: Clean up processing queue and conversation history

**Key Features:**
- OpenAI GPT integration with configurable model parameters
- Conversation history management with automatic truncation
- Confidence-based voice command filtering
- Request queuing with overflow handling
- Context-aware conversation with message history
- Thread-safe conversation history with locking
- Declarative message handling (no manual subscribe calls)
- Error handling for API failures with fallback responses

```python
# nodes/ai_cognition/ai_cognition_node.py

import time
import threading
from nevil_framework.base_node import NevilNode
from openai import OpenAI

class AICognitionNode(NevilNode):
    """
    AI cognition node for processing commands and generating responses.
    Uses OpenAI GPT for conversation and decision making.
    """
    
    def __init__(self):
        super().__init__("ai_cognition")
        self.openai_client = None
        self.conversation_history = []
        self.context_lock = threading.Lock()
        self.processing_queue = queue.Queue(maxsize=5)
        
    def initialize(self):
        """Initialize AI components"""
        try:
            # Initialize OpenAI client
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                self.openai_client = OpenAI(api_key=api_key)
            else:
                raise ValueError("OPENAI_API_KEY not found")
            
            # Note: No manual subscribe() calls needed!
            # All subscriptions are automatically configured via .messages file
            # during init_messages() in __init__
            
            self.logger.info("AI cognition initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize AI cognition: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        try:
            # Process AI requests
            try:
                request_data = self.processing_queue.get_nowait()
                self._process_ai_request(request_data)
            except queue.Empty:
                pass
            
        except Exception as e:
            self.logger.error(f"Error in AI cognition: {e}")
        
        # Brief pause
        time.sleep(0.1)
    
    def on_voice_command(self, message):
        """Handle voice command messages"""
        try:
            text = message.data.get("text", "")
            confidence = message.data.get("confidence", 0.0)
            
            if text and confidence > 0.5:  # Confidence threshold
                request_data = {
                    "text": text,
                    "confidence": confidence,
                    "timestamp": time.time(),
                    "source": "voice"
                }
                
                # Add to processing queue
                try:
                    self.processing_queue.put_nowait(request_data)
                except queue.Full:
                    self.logger.warning("AI processing queue full, dropping request")
                    
        except Exception as e:
            self.logger.error(f"Error handling voice command: {e}")
    
    def _process_ai_request(self, request_data):
        """Process AI request in separate thread"""
        try:
            text = request_data["text"]
            
            # Add to conversation history
            with self.context_lock:
                self.conversation_history.append({
                    "role": "user",
                    "content": text,
                    "timestamp": time.time()
                })
                
                # Keep only last 10 exchanges
                if len(self.conversation_history) > 20:
                    self.conversation_history = self.conversation_history[-20:]
            
            # Generate AI response
            response_text = self._generate_response(text)
            
            if response_text:
                # Add response to history
                with self.context_lock:
                    self.conversation_history.append({
                        "role": "assistant",
                        "content": response_text,
                        "timestamp": time.time()
                    })
                
                # Publish response
                response_data = {
                    "text": response_text,
                    "voice": "onyx",
                    "priority": 100,
                    "timestamp": time.time()
                }
                
                self.publish("text_response", response_data)
                self.logger.info(f"Generated response: {response_text}")
                
        except Exception as e:
            self.logger.error(f"Error processing AI request: {e}")
    
    def _generate_response(self, user_input: str) -> str:
        """Generate AI response using OpenAI"""
        try:
            if not self.openai_client:
                return "I'm sorry, I'm not able to process that right now."
            
            # Prepare conversation context
            with self.context_lock:
                messages = [
                    {"role": "system", "content": "You are Nevil, a helpful robot companion."},
                    *self.conversation_history[-10:]  # Last 10 messages
                ]
            
            # Call OpenAI
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=messages,
                max_tokens=150,
                temperature=0.7
            )
            
            return response.choices[0].message.content.strip()
            
        except Exception as e:
            self.logger.error(f"Error generating AI response: {e}")
            return "I'm sorry, I encountered an error processing your request."
    
    def cleanup(self):
        """Cleanup resources"""
        # Clear processing queue
        while not self.processing_queue.empty():
            try:
                self.processing_queue.get_nowait()
            except queue.Empty:
                break
        
        # Clear conversation history
        with self.context_lock:
            self.conversation_history.clear()
        
        self.openai_client = None
        self.logger.info("AI cognition cleaned up")

# TEST: AI cognition node initializes with valid OpenAI client
# TEST: Voice commands are queued and processed correctly
# TEST: Conversation history is maintained and limited
# TEST: AI responses are generated and published
# TEST: Error handling for API failures
```

## 4. Thread Synchronization Patterns

#### Code Summary

This section implements advanced threading utilities for complex synchronization needs:

**Classes:**
- **OrderedMessageQueue**: Ensures message ordering within topics using priority queues
- **ThreadSafeResourcePool**: Pool of reusable resources with thread safety and size limits

**Key Methods in OrderedMessageQueue:**
- `__init__()`: Initialize priority queue with sequence tracking
- `put()`: Add message with priority and sequence number for ordering
- `get()`: Retrieve message maintaining priority and sequence order

**Key Methods in ThreadSafeResourcePool:**
- `__init__()`: Initialize resource pool with factory function and size limits
- `acquire()`: Get resource from pool with timeout, create new if needed
- `release()`: Return resource to pool or discard if pool is full

**Key Features:**
- Priority-based message ordering with sequence numbers
- Resource pooling with automatic creation and disposal
- Thread-safe operations with proper locking
- Timeout handling for resource acquisition
- Pool size management with overflow handling

### 4.1 Message Ordering
```python
class OrderedMessageQueue:
    """Ensures message ordering within topics"""
    
    def __init__(self, maxsize: int = 100):
        self._queue = queue.PriorityQueue(maxsize=maxsize)
        self._sequence = 0
        self._lock = threading.Lock()
    
    def put(self, message, priority: int = 100):
        with self._lock:
            # Use sequence number to maintain order within priority
            item = (priority, self._sequence, message)
            self._sequence += 1
            self._queue.put(item)
    
    def get(self, timeout: float = None):
        priority, sequence, message = self._queue.get(timeout=timeout)
        return message
```

### 4.2 Resource Pooling
```python
class ThreadSafeResourcePool:
    """Pool of reusable resources with thread safety"""
    
    def __init__(self, resource_factory, max_size: int = 5):
        self._factory = resource_factory
        self._pool = queue.Queue(maxsize=max_size)
        self._created_count = 0
        self._max_size = max_size
        self._lock = threading.Lock()
    
    def acquire(self, timeout: float = 5.0):
        try:
            return self._pool.get(timeout=timeout)
        except queue.Empty:
            with self._lock:
                if self._created_count < self._max_size:
                    resource = self._factory()
                    self._created_count += 1
                    return resource
            raise RuntimeError("Resource pool exhausted")
    
    def release(self, resource):
        try:
            self._pool.put_nowait(resource)
        except queue.Full:
            # Pool is full, discard resource
            pass
```

## 5. Performance Considerations

### 5.3 I/O Optimization - Phase 1 Requirements
- Asynchronous file operations for logging
- Buffered message delivery
- Connection pooling for network resources
- Efficient audio buffer management

## 6. Error Recovery Patterns



### 6.2 Retry Logic

#### Code Summary

This section implements configurable retry logic with exponential backoff for robust error handling:

**Classes:**
- **RetryHandler**: Configurable retry mechanism with exponential backoff strategy

**Key Methods in RetryHandler:**
- `__init__()`: Initialize retry handler with maximum attempts and base delay
- `retry()`: Execute function with retry logic and exponential backoff on failures

**Key Features:**
- Configurable maximum retry attempts
- Exponential backoff delay calculation (base_delay * 2^attempt)
- Exception handling with re-raising after max retries
- Function wrapper pattern for easy integration
- Automatic delay progression for network and API resilience

```python
class RetryHandler:
    """Configurable retry logic with exponential backoff"""
    
    def __init__(self, max_retries: int = 3, base_delay: float = 1.0):
        self.max_retries = max_retries
        self.base_delay = base_delay
    
    def retry(self, func, *args, **kwargs):
        for attempt in range(self.max_retries + 1):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                if attempt == self.max_retries:
                    raise e
                
                delay = self.base_delay * (2 ** attempt)
                time.sleep(delay)
```

## 7. Testing Strategy

### 7.1 Unit Tests
- Test each node in isolation
- Mock message bus for testing
- Verify thread safety with concurrent tests
- Test error conditions and recovery

### 7.2 Integration Tests
- Test inter-node communication
- Verify message ordering and delivery
- Test system startup and shutdown
- Performance benchmarking



## Conclusion

The Nevil v3.0 node structure and threading model provides a robust foundation for building reliable, maintainable robotic applications. The design emphasizes:

- **Simplicity**: Clear, understandable patterns
- **Reliability**: Proper error handling and recovery
- **Performance**: Efficient resource usage and threading
- **Maintainability**: Well-structured code with comprehensive testing

This architecture enables the development of autonomous nodes that can operate independently while communicating effectively through a simple message bus system.

# TEST: Complete framework can launch and manage multiple nodes
# TEST: Message delivery is reliable under normal and stress conditions
# TEST: Graceful shutdown works correctly for all components
# TEST: Error recovery mechanisms function as designed
# TEST: Performance meets specified latency and throughput requirements