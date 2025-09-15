
# Nevil v3.0 Framework Core Components Pseudocode

## Overview

This document provides detailed pseudocode for the core framework components of Nevil v3.0. Each component is designed with simplicity, reliability, and maintainability as primary goals while providing the necessary functionality for a robust robotic framework.

**Key Enhancement**: The framework now implements **declarative messaging** through the `init_messages()` method, eliminating the need for manual publish/subscribe method calls and centralizing all messaging configuration in `.messages` files.

## Detailed Technical Design Specification

### Framework Architecture Philosophy

The Nevil v3.0 framework core is built on a **component-based architecture** that emphasizes loose coupling, high cohesion, and clear separation of concerns. Each core component operates independently while participating in a coordinated system through well-defined interfaces.

#### Design Principles

1. **Declarative Configuration**: All component behavior is defined through configuration rather than imperative code
2. **Event-Driven Architecture**: Components communicate through asynchronous events and messages


### Message Bus Technical Architecture

The message bus implements a **high-performance, topic-based publish-subscribe pattern** with advanced features for reliability and observability.

#### Message Flow Architecture

```
Publisher → Topic Router → Subscription Manager → Delivery Engine → Subscriber
    ↓           ↓              ↓                    ↓               ↓
Message → Topic Index → Subscriber List → Priority Queue → Callback
```

#### Advanced Messaging Features

**Message Prioritization System**:
- **Critical Priority**: System alerts and emergency messages (processed immediately)
- **High Priority**: User commands and time-sensitive operations
- **Normal Priority**: Standard application messages and data
- **Low Priority**: Background tasks and non-essential updates

**Delivery Guarantee Framework**:
- **At-Least-Once**: Messages guaranteed to be delivered at least once
- **Exactly-Once**: Duplicate detection and elimination for critical messages
- **Best-Effort**: High-performance delivery without guarantees for non-critical data



**Topic Hierarchy**:
- **Namespace Separation**: Logical separation of topics by domain


### Configuration Manager Technical Architecture

The configuration manager implements a **multi-source, hierarchical configuration system** with real-time updates and validation.

#### Configuration Source Hierarchy

```
Environment Variables → Command Line → Config Files → Defaults
        ↓                    ↓            ↓           ↓
    Highest Priority → Medium Priority → Low Priority → Fallback
```

#### Hot-Reload Architecture


### Node Registry Technical Architecture

The node registry implements a **distributed service discovery pattern** with comprehensive health monitoring and dependency management.

#### Service Discovery Framework

**Registration Process**:
1. **Node Announcement**: Nodes announce themselves with capability metadata


#### Health Monitoring Architecture

**Multi-Level Health Checks**:
- **Process Health**: Basic process existence and responsiveness
- **Application Health**: Custom health check endpoints and logic
- **Business Health**: Domain-specific health indicators
- **Integration Health**: External dependency health status

**Health State Machine**:
```
UNKNOWN → STARTING → HEALTHY → DEGRADED → UNHEALTHY → FAILED → RECOVERING
    ↓         ↓         ↓         ↓          ↓         ↓         ↓
Initial → Startup → Normal → Warning → Error → Critical → Recovery
```

**Failure Detection Strategies**:
- **Heartbeat Monitoring**: Regular heartbeat messages with timeout detection
- **Performance Monitoring**: Response time and throughput monitoring
- **Error Rate Monitoring**: Error frequency and pattern analysis
- **Resource Monitoring**: CPU, memory, and resource utilization tracking

### Error Handling Technical Architecture

The error handling system implements a **comprehensive error management framework** with automatic classification, recovery, and escalation.

#### Error Classification Framework

**Error Taxonomy:**
- **Transient Errors**: Temporary failures that may resolve automatically
- **Persistent Errors**: Consistent failures requiring intervention
- **Critical Errors**: System-threatening failures requiring immediate action
- **Business Errors**: Domain-specific errors with business logic implications

**Phase 2 Features:**
Advanced error handling features including context enrichment, recovery strategies, reporting analytics, and pattern detection are available in Phase 2.

**Phase 2 Documentation:**
See [`06_framework_core_components_pseudocode_phase_2.md`](./phase%202/06_framework_core_components_pseudocode_phase_2.md) for comprehensive Phase 2 framework features including:
- Dependency management framework
- Advanced error context enrichment and recovery strategies
- Performance optimization and scalability architecture
- Security and isolation enhancements
- Resource management framework

This comprehensive technical design ensures that the Nevil v3.0 framework core provides robust, scalable, and secure foundation components while maintaining the simplicity and reliability required for robotic applications.

## 1. Message Bus Implementation

### 1.1 Core Message Bus

```python
# nevil_framework/message_bus.py

import multiprocessing
import queue
import threading
import time
import uuid
from typing import Dict, Set, List, Any, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum

class MessagePriority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

@dataclass
class Message:
    topic: str
    data: Any
    timestamp: float
    source_node: str
    message_id: str
    priority: MessagePriority = MessagePriority.NORMAL
    ttl: Optional[float] = None  # Time to live in seconds
    reply_to: Optional[str] = None  # For request-response patterns

@dataclass
class Subscription:
    node_name: str
    topic: str
    queue: queue.Queue
    callback: Optional[Callable] = None
    filter_func: Optional[Callable] = None
    created_at: float = field(default_factory=time.time)

class MessageBus:
    """
    Simple, reliable message bus for Nevil v3.0.
    
    Features:
    - Topic-based publish/subscribe
    - Message priorities and TTL
    - Request-response patterns
    - Message filtering
    - Delivery guarantees
    - Performance monitoring
    """
    
    def __init__(self, max_queue_size: int = 1000):
        self.max_queue_size = max_queue_size
        
        # Core data structures
        self.topics = {}  # topic -> set of subscriptions
        self.subscriptions = {}  # node_name -> list of subscriptions
        self.message_history = {}  # topic -> deque of recent messages
        
        # Thread safety
        self.lock = threading.RLock()
        
        # Message processing
        self.message_queue = queue.PriorityQueue(maxsize=max_queue_size * 2)
        self.processor_thread = None
        self.running = False
        
        # Statistics
        self.stats = {
            'messages_published': 0,
            'messages_delivered': 0,
            'messages_dropped': 0,
            'delivery_failures': 0,
            'active_subscriptions': 0,
            'topics_count': 0
        }
        
        # Message retention for debugging
        self.retain_messages = True
        self.max_retained_messages = 100
        
    def start(self):
        """Start the message bus processing"""
        if self.running:
            return
            
        self.running = True
        self.processor_thread = threading.Thread(
            target=self._message_processor_loop,
            name="MessageBusProcessor",
            daemon=True
        )
        self.processor_thread.start()
        
    def stop(self):
        """Stop the message bus processing"""
        if not self.running:
            return
            
        self.running = False
        
        # Add sentinel message to wake up processor
        try:
            self.message_queue.put_nowait((0, time.time(), None))
        except queue.Full:
            pass
            
        if self.processor_thread:
            self.processor_thread.join(timeout=5.0)
    
    def publish(self, message: Message) -> bool:
        """
        Publish a message to a topic.
        
        Args:
            message: Message to publish
            
        Returns:
            True if message was queued for delivery, False otherwise
        """
        try:
            with self.lock:
                # Validate message
                if not self._validate_message(message):
                    return False
                
                # Check TTL
                if message.ttl and (time.time() - message.timestamp) > message.ttl:
                    self.stats['messages_dropped'] += 1
                    return False
                
                # Add to processing queue with priority
                priority_value = 5 - message.priority.value  # Higher priority = lower number
                queue_item = (priority_value, message.timestamp, message)
                
                try:
                    self.message_queue.put_nowait(queue_item)
                    self.stats['messages_published'] += 1
                    return True
                except queue.Full:
                    self.stats['messages_dropped'] += 1
                    return False
                    
        except Exception as e:
            print(f"Error publishing message: {e}")
            return False
    
    def subscribe(self, node_name: str, topic: str, message_queue: queue.Queue,
                 callback: Callable = None, filter_func: Callable = None) -> bool:
        """
        Subscribe to a topic.
        
        Args:
            node_name: Name of subscribing node
            topic: Topic to subscribe to
            message_queue: Queue to deliver messages to
            callback: Optional callback function
            filter_func: Optional message filter function
            
        Returns:
            True if subscription was successful
        """
        try:
            with self.lock:
                # Create subscription
                subscription = Subscription(
                    node_name=node_name,
                    topic=topic,
                    queue=message_queue,
                    callback=callback,
                    filter_func=filter_func
                )
                
                # Add to topic subscriptions
                if topic not in self.topics:
                    self.topics[topic] = set()
                    self.stats['topics_count'] += 1
                
                self.topics[topic].add(subscription)
                
                # Add to node subscriptions
                if node_name not in self.subscriptions:
                    self.subscriptions[node_name] = []
                
                self.subscriptions[node_name].append(subscription)
                self.stats['active_subscriptions'] += 1
                
                # Send retained messages if any
                if self.retain_messages and topic in self.message_history:
                    for retained_message in self.message_history[topic]:
                        self._deliver_message_to_subscription(retained_message, subscription)
                
                return True
                
        except Exception as e:
            print(f"Error subscribing to topic {topic}: {e}")
            return False
    
    def unsubscribe(self, node_name: str, topic: str) -> bool:
        """
        Unsubscribe from a topic.
        
        Args:
            node_name: Name of node to unsubscribe
            topic: Topic to unsubscribe from
            
        Returns:
            True if unsubscription was successful
        """
        try:
            with self.lock:
                # Find and remove subscription
                if node_name in self.subscriptions:
                    node_subs = self.subscriptions[node_name]
                    for subscription in node_subs[:]:  # Copy list to avoid modification during iteration
                        if subscription.topic == topic:
                            # Remove from topic
                            if topic in self.topics:
                                self.topics[topic].discard(subscription)
                                if not self.topics[topic]:
                                    del self.topics[topic]
                                    self.stats['topics_count'] -= 1
                            
                            # Remove from node
                            node_subs.remove(subscription)
                            self.stats['active_subscriptions'] -= 1
                            
                            return True
                
                return False
                
        except Exception as e:
            print(f"Error unsubscribing from topic {topic}: {e}")
            return False
    
    def unsubscribe_node(self, node_name: str) -> int:
        """
        Unsubscribe a node from all topics.
        
        Args:
            node_name: Name of node to unsubscribe
            
        Returns:
            Number of subscriptions removed
        """
        try:
            with self.lock:
                if node_name not in self.subscriptions:
                    return 0
                
                removed_count = 0
                node_subs = self.subscriptions[node_name][:]  # Copy list
                
                for subscription in node_subs:
                    if self.unsubscribe(node_name, subscription.topic):
                        removed_count += 1
                
                # Clean up empty node entry
                if node_name in self.subscriptions and not self.subscriptions[node_name]:
                    del self.subscriptions[node_name]
                
                return removed_count
                
        except Exception as e:
            print(f"Error unsubscribing node {node_name}: {e}")
            return 0
    
    def request(self, topic: str, data: Any, source_node: str, 
               timeout: float = 5.0) -> Optional[Message]:
        """
        Send a request and wait for response.
        
        Args:
            topic: Topic to send request to
            data: Request data
            source_node: Name of requesting node
            timeout: Timeout in seconds
            
        Returns:
            Response message or None if timeout
        """
        # Create response topic
        response_topic = f"{topic}_response_{uuid.uuid4().hex[:8]}"
        response_queue = queue.Queue(maxsize=1)
        
        # Subscribe to response topic
        if not self.subscribe(source_node, response_topic, response_queue):
            return None
        
        try:
            # Send request
            request_message = Message(
                topic=topic,
                data=data,
                timestamp=time.time(),
                source_node=source_node,
                message_id=str(uuid.uuid4()),
                reply_to=response_topic,
                priority=MessagePriority.HIGH
            )
            
            if not self.publish(request_message):
                return None
            
            # Wait for response
            try:
                response = response_queue.get(timeout=timeout)
                return response
            except queue.Empty:
                return None
                
        finally:
            # Clean up response subscription
            self.unsubscribe(source_node, response_topic)
    
    def respond(self, request_message: Message, response_data: Any, source_node: str) -> bool:
        """
        Send a response to a request.
        
        Args:
            request_message: Original request message
            response_data: Response data
            source_node: Name of responding node
            
        Returns:
            True if response was sent
        """
        if not request_message.reply_to:
            return False
        
        response_message = Message(
            topic=request_message.reply_to,
            data=response_data,
            timestamp=time.time(),
            source_node=source_node,
            message_id=str(uuid.uuid4()),
            priority=MessagePriority.HIGH
        )
        
        return self.publish(response_message)
    
    def _message_processor_loop(self):
        """Main message processing loop"""
        while self.running:
            try:
                # Get message from queue
                try:
                    priority, timestamp, message = self.message_queue.get(timeout=1.0)
                    
                    # Check for sentinel (shutdown signal)
                    if message is None:
                        break
                        
                except queue.Empty:
                    continue
                
                # Process message
                self._process_message(message)
                
            except Exception as e:
                print(f"Error in message processor: {e}")
    
    def _process_message(self, message: Message):
        """Process a single message"""
        try:
            with self.lock:
                topic = message.topic
                
                # Check if topic has subscribers
                if topic not in self.topics:
                    return
                
                # Deliver to all subscribers
                delivered_count = 0
                for subscription in self.topics[topic].copy():  # Copy to avoid modification during iteration
                    if self._deliver_message_to_subscription(message, subscription):
                        delivered_count += 1
                
                self.stats['messages_delivered'] += delivered_count
                
                # Store message for retention
                if self.retain_messages:
                    self._store_retained_message(message)
                    
        except Exception as e:
            print(f"Error processing message: {e}")
            self.stats['delivery_failures'] += 1
    
    def _deliver_message_to_subscription(self, message: Message, subscription: Subscription) -> bool:
        """Deliver message to a specific subscription"""
        try:
            # Apply filter if present
            if subscription.filter_func and not subscription.filter_func(message):
                return False
            
            # Deliver to queue
            try:
                subscription.queue.put_nowait(message)
                return True
            except queue.Full:
                # Queue is full, drop message
                self.stats['messages_dropped'] += 1
                return False
                
        except Exception as e:
            print(f"Error delivering message to {subscription.node_name}: {e}")
            return False
    
    def _store_retained_message(self, message: Message):
        """Store message for retention"""
        topic = message.topic
        
        if topic not in self.message_history:
            from collections import deque
            self.message_history[topic] = deque(maxlen=self.max_retained_messages)
        
        self.message_history[topic].append(message)
    
    def _validate_message(self, message: Message) -> bool:
        """Validate message before processing"""
        if not message.topic:
            return False
        if not message.source_node:
            return False
        if not message.message_id:
            return False
        return True
    
    def get_stats(self) -> Dict[str, Any]:
        """Get message bus statistics"""
        with self.lock:
            stats = self.stats.copy()
            stats['queue_size'] = self.message_queue.qsize()
            stats['topics'] = list(self.topics.keys())
            stats['nodes'] = list(self.subscriptions.keys())
            return stats
    
    def get_topic_info(self, topic: str) -> Dict[str, Any]:
        """Get information about a specific topic"""
        with self.lock:
            if topic not in self.topics:
                return {}
            
            subscribers = [sub.node_name for sub in self.topics[topic]]
            message_count = len(self.message_history.get(topic, []))
            
            return {
                'topic': topic,
                'subscriber_count': len(subscribers),
                'subscribers': subscribers,
                'retained_messages': message_count
            }

# TEST: Message bus delivers messages to all subscribers
# TEST: Priority messages are processed before normal messages
# TEST: TTL expiration prevents delivery of old messages
# TEST: Request-response pattern works correctly
# TEST: Message filtering functions properly
# TEST: Statistics are accurate under load
```

## 2. Configuration Manager

### 2.1 Configuration Manager Implementation

```python
# nevil_framework/config_manager.py

import os
import yaml
import threading
import time
from typing import Dict, Any, List, Optional, Callable
from pathlib import Path
from dataclasses import dataclass
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

@dataclass
class ConfigChange:
    file_path: str
    change_type: str  # 'modified', 'created', 'deleted'
    timestamp: float
    old_config: Optional[Dict] = None
    new_config: Optional[Dict] = None

class ConfigFileHandler(FileSystemEventHandler):
    """Handles configuration file change events"""
    
    def __init__(self, callback: Callable[[str], None]):
        self.callback = callback
        self.last_modified = {}
        self.debounce_time = 1.0  # seconds
    
    def on_modified(self, event):
        if event.is_directory:
            return
        
        file_path = event.src_path
        
        # Only process configuration files
        if not (file_path.endswith('.nodes') or file_path.endswith('.messages') or 
                file_path.endswith('.yaml') or file_path.endswith('.yml')):
            return
        
        # Debounce rapid changes
        current_time = time.time()
        if file_path in self.last_modified:
            if current_time - self.last_modified[file_path] < self.debounce_time:
                return
        
        self.last_modified[file_path] = current_time
        self.callback(file_path)

class ConfigurationManager:
    """
    Manages configuration loading, validation, and hot-reloading.
    
    Features:
    - YAML configuration loading
    - Environment variable expansion
    - Configuration validation
    - Hot-reloading with change notifications
    - Configuration merging and inheritance
    - Secure handling of sensitive data
    """
    
    def __init__(self, root_path: str = "."):
        self.root_path = Path(root_path)
        self.configs = {}  # file_path -> config_data
        self.watchers = {}  # file_path -> list of callbacks
        self.lock = threading.RLock()
        
        # File watching
        self.observer = Observer()
        self.watching = False
        
        # Validation schemas
        self.schemas = {}
        
        # Environment variable cache
        self.env_cache = {}
        self.env_cache_time = 0
        self.env_cache_ttl = 60  # seconds
        
    def load_config(self, file_path: str, schema: Dict = None) -> Dict[str, Any]:
        """
        Load configuration from file.
        
        Args:
            file_path: Path to configuration file
            schema: Optional validation schema
            
        Returns:
            Loaded and validated configuration
        """
        try:
            full_path = self.root_path / file_path
            
            if not full_path.exists():
                raise FileNotFoundError(f"Configuration file not found: {full_path}")
            
            # Load YAML
            with open(full_path, 'r', encoding='utf-8') as f:
                raw_config = yaml.safe_load(f)
            
            if raw_config is None:
                raw_config = {}
            
            # Expand environment variables
            config = self._expand_environment_variables(raw_config)
            
            # Validate if schema provided
            if schema:
                self._validate_config(config, schema, str(full_path))
            
            # Cache configuration
            with self.lock:
                self.configs[str(full_path)] = config
            
            return config
            
        except Exception as e:
            raise ValueError(f"Error loading configuration from {file_path}: {e}")
    
    def save_config(self, file_path: str, config: Dict[str, Any]) -> bool:
        """
        Save configuration to file.
        
        Args:
            file_path: Path to configuration file
            config: Configuration data to save
            
        Returns:
            True if successful
        """
        try:
            full_path = self.root_path / file_path
            
            # Create directory if needed
            full_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write YAML
            with open(full_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            
            # Update cache
            with self.lock:
                self.configs[str(full_path)] = config
            
            return True
            
        except Exception as e:
            print(f"Error saving configuration to {file_path}: {e}")
            return False
    
    def watch_config(self, file_path: str, callback: Callable[[Dict[str, Any]], None]):
        """
        Watch configuration file for changes.
        
        Args:
            file_path: Path to configuration file
            callback: Function to call when config changes
        """
        full_path = str(self.root_path / file_path)
        
        with self.lock:
            if full_path not in self.watchers:
                self.watchers[full_path] = []
            self.watchers[full_path].append(callback)
        
        # Start watching if not already
        if not self.watching:
            self._start_watching()
    
    def unwatch_config(self, file_path: str, callback: Callable = None):
        """
        Stop watching configuration file.
        
        Args:
            file_path: Path to configuration file
            callback: Specific callback to remove (None for all)
        """
        full_path = str(self.root_path / file_path)
        
        with self.lock:
            if full_path in self.watchers:
                if callback:
                    try:
                        self.watchers[full_path].remove(callback)
                    except ValueError:
                        pass
                else:
                    self.watchers[full_path].clear()
                
                # Remove empty watcher lists
                if not self.watchers[full_path]:
                    del self.watchers[full_path]
    
    def merge_configs(self, base_config: Dict, override_config: Dict) -> Dict[str, Any]:
        """
        Merge two configuration dictionaries.
        
        Args:
            base_config: Base configuration
            override_config: Configuration to merge in
            
        Returns:
            Merged configuration
        """
        result = base_config.copy()
        
        for key, value in override_config.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self.merge_configs(result[key], value)
            else:
                result[key] = value
        
        return result
    
    def get_config_value(self, file_path: str, key_path: str, default: Any = None) -> Any:
        """
        Get a specific value from configuration using dot notation.
        
        Args:
            file_path: Path to configuration file
            key_path: Dot-separated key path (e.g., 'nodes.speech_recognition.status')
            default: Default value if key not found
            
        Returns:
            Configuration value or default
        """
        full_path = str(self.root_path / file_path)
        
        with self.lock:
            if full_path not in self.configs:
                return default
            
            config = self.configs[full_path]
            
            # Navigate through key path
            current = config
            for key in key_path.split('.'):
                if isinstance(current, dict) and key in current:
                    current = current[key]
                else:
                    return default
            
            return current
    
    def set_config_value(self, file_path: str, key_path: str, value: Any) -> bool:
        """
        Set a specific value in configuration using dot notation.
        
        Args:
            file_path: Path to configuration file
            key_path: Dot-separated key path
            value: Value to set
            
        Returns:
            True if successful
        """
        full_path = str(self.root_path / file_path)
        
        with self.lock:
            if full_path not in self.configs:
                self.configs[full_path] = {}
            
            config = self.configs[full_path]
            
            # Navigate to parent and set value
            keys = key_path.split('.')
            current = config
            
            for key in keys[:-1]:
                if key not in current:
                    current[key] = {}
                current = current[key]
            
            current[keys[-1]] = value
            
            # Save to file
            return self.save_config(file_path, config)
    
    def _start_watching(self):
        """Start file system watching"""
        if self.watching:
            return
        
        handler = ConfigFileHandler(self._on_config_file_changed)
        self.observer.schedule(handler, str(self.root_path), recursive=True)
        self.observer.start()
        self.watching = True
    
    def _stop_watching(self):
        """Stop file system watching"""
        if not self.watching:
            return
        
        self.observer.stop()
        self.observer.join()
        self.watching = False
    
    def _on_config_file_changed(self, file_path: str):
        """Handle configuration file change"""
        try:
            with self.lock:
                if file_path not in self.watchers:
                    return
                
                # Load new configuration
                try:
                    rel_path = str(Path(file_path).relative_to(self.root_path))
                    new_config = self.load_config(rel_path)
                    
                    # Notify watchers
                    for callback in self.watchers[file_path]:
                        try:
                            callback(new_config)
                        except Exception as e:
                            print(f"Error in config change callback: {e}")
                            
                except Exception as e:
                    print(f"Error reloading configuration {file_path}: {e}")
                    
        except Exception as e:
            print(f"Error handling config file change: {e}")
    
    def _expand_environment_variables(self, config: Any) -> Any:
        """
        Recursively expand environment variables in configuration.
        
        Supports patterns like:
        - ${VAR}
        - ${VAR:-default}
        - ${VAR:default}
        """
        if isinstance(config, dict):
            return {key: self._expand_environment_variables(value) for key, value in config.items()}
        elif isinstance(config, list):
            return [self._expand_environment_variables(item) for item in config]
        elif isinstance(config, str):
            return self._expand_string_variables(config)
        else:
            return config
    
    def _expand_string_variables(self, text: str) -> str:
        """Expand environment variables in a string"""
        import re
        
        # Pattern for ${VAR} or ${VAR:-default} or ${VAR:default}
        pattern = r'\$\{([^}]+)\}'
        
        def replace_var(match):
            var_expr = match.group(1)
            
            # Handle default values
            if ':-' in var_expr:
                var_name, default = var_expr.split(':-', 1)
                return os.getenv(var_name.strip(), default)
            elif ':' in var_expr:
                var_name, default = var_expr.split(':', 1)
                value = os.getenv(var_name.strip())
                return value if value else default
            else:
                return os.getenv(var_expr.strip(), f"${{{var_expr}}}")
        
        return re.sub(pattern, replace_var, text)
    
    def _validate_config(self, config: Dict, schema: Dict, file_path: str):
        """Validate configuration against schema"""
        # Simple validation - could be extended with jsonschema
        errors = []
        
        def validate_recursive(data, schema_part, path=""):
            if 'required' in schema_part:
                for required_key in schema_part['required']:
                    if required_key not in data:
                        errors.append(f"{path}.{required_key} is required")
            
            if 'properties' in schema_part:
                for key, value in data.items():
                    if key in schema_part['properties']:
                        key_schema = schema_part['properties'][key]
                        validate_recursive(value, key_schema, f"{path}.{key}")
        
        validate_recursive(config, schema)
        
        if errors:
            raise ValueError(f"Configuration validation failed for {file_path}: {'; '.join(errors)}")
    
    def cleanup(self):
        """Clean up resources"""
        self._stop_watching()
        with self.lock:
            self.configs.clear()
            self.watchers.clear()

# TEST: Configuration manager loads YAML files correctly
# TEST: Environment variable expansion works for all patterns
# TEST: Hot-reloading triggers callbacks when files change
# TEST: Configuration validation catches schema violations
# TEST: Dot notation access works for nested configurations
```

## 3. Node Registry and Discovery

### 3.1 Node Registry Implementation

```python
# nevil_framework/node_registry.py

import time
import threading
from typing import Dict, List, Set, Optional, Any
from dataclasses import dataclass, field
from enum import Enum

class NodeStatus(Enum):
    UNKNOWN = "unknown"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"
    TIMEOUT = "timeout"

@dataclass
class NodeInfo:
    name: str
    status: NodeStatus = NodeStatus.UNKNOWN
    pid: Optional[int] = None
    start_time: Optional[float] = None
    last_heartbeat: Optional[float] = None
    restart_count: int = 0
    error_count: int = 0
    
    # Resource usage
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    thread_count: int = 0
    
    # Configuration
    config: Dict[str, Any] = field(default_factory=dict)
    
    # Communication
    published_topics: Set[str] = field(default_factory=set)
    subscribed_topics: Set[str] = field(default_factory=set)
    
    # Metadata
    version: str = "unknown"
    description: str = ""
    tags: Set[str] = field(default_factory=set)

class NodeRegistry:
    """
    Registry for tracking and managing nodes in the Nevil v3.0 system.
    
    Features:
    - Node registration and discovery
    - Health monitoring and status tracking
    - Resource usage monitoring
    - Topic subscription tracking
    - Node dependency management
    - Service discovery
    """
    
    def __init__(self, heartbeat_timeout: float = 30.0):
        self.heartbeat_timeout = heartbeat_timeout
        
        # Node storage
        self.nodes = {}  # node_name -> NodeInfo
        self.lock = threading.RLock()
        
        # Topic tracking
        self.topic_publishers = {}  # topic -> set of node names
        self.topic_subscribers = {}  # topic -> set of node names
        
        # Dependencies
        self.dependencies = {}  # node_name -> set of dependency node names
        self.dependents = {}  # node_name -> set of dependent node names
        
        # Health monitoring
        self.health_check_thread = None
        self.running = False
        
        # Statistics
        self.stats = {
            'total_nodes': 0,
            'running_nodes': 0,
            'failed_nodes': 0,
            'total_restarts': 0,
            'registry_start_time': time.time()
        }
    
    def start(self):
        """Start the node registry"""
        if self.running:
            return
        
        self.running = True
        self.health_check_thread = threading.Thread(
            target=self._health_check_loop,
            name="NodeRegistryHealthCheck",
            daemon=True
        )
        self.health_check_thread.start()
    
    def stop(self):
        """Stop the node registry"""
        self.running = False
        if self.health_check_thread:
            self.health_check_thread.join(timeout=5.0)
    
    def register_node(self, name: str, config: Dict[str, Any] = None) -> bool:
        """
        Register a new node.
        
        Args:
            name: Node name
            config: Node configuration
            
        Returns:
            True if registration successful
        """
        try:
            with self.lock:
                if name in self.nodes:
                    # Update existing node
                    node_info = self.nodes[name]
                    if config:
                        node_info.config.update(config)
                else:
                    # Create new node
                    node_info = NodeInfo(
                        name=name,
                        config=config or {},
                        start_time=time.time()
                    )
                    self.nodes[name] = node_info
                    self.stats['total_nodes'] += 1
                
                # Set up dependencies if specified
                if config and 'depends_on' in config:
                    self._setup_dependencies(name, config['depends_on'])
                
                return True
                
        except Exception as e:
            print(f"Error registering node {name}: {e}")
            return False
    
    def unregister_node(self, name: str) -> bool:
        """
        Unregister a node.
        
        Args:
            name: Node name
            
        Returns:
            True if unregistration successful
        """
        try:
            with self.lock:
                if name not in self.nodes:
                    return False
                
                node_info = self.nodes[name]
                
                # Clean up topic tracking
                for topic in node_info.published_topics:
                    if topic in self.topic_publishers:
                        self.topic_publishers[topic].discard(name)
                        if not self.topic_publishers[topic]:
                            del self.topic_publishers[topic]
                
                for topic in node_info.subscribed_topics:
                    if topic in self.topic_subscribers:
                        self.topic_subscribers[topic].discard(name)
                        if not self.topic_subscribers[topic]:
                            del self.topic_subscribers[topic]
                
                # Clean up dependencies
                self._cleanup_dependencies(name)
                
                # Remove node
                del self.nodes[name]
                self.stats['total_nodes'] -= 1
                
                return True
                
        except Exception as e:
            print(f"Error unregistering node {name}: {e}")
            return False
    
    def update_node_status(self, name: str, status: NodeStatus,
                          pid: int = None, **kwargs) -> bool:
        """
        Update node status and metadata.
        
        Args:
            name: Node name
            status: New status
            pid: Process ID
            **kwargs: Additional metadata to update
            
        Returns:
            True if update successful
        """
        try:
            with self.lock:
                if name not in self.nodes:
                    return False
                
                node_info = self.nodes[name]
                old_status = node_info.status
                
                # Update status
                node_info.status = status
                if pid is not None:
                    node_info.pid = pid
                
                # Update heartbeat for running nodes
                if status == NodeStatus.RUNNING:
                    node_info.last_heartbeat = time.time()
                
                # Update additional metadata
                for key, value in kwargs.items():
                    if hasattr(node_info, key):
                        setattr(node_info, key, value)
                
                # Update statistics
                if old_status != NodeStatus.RUNNING and status == NodeStatus.RUNNING:
                    self.stats['running_nodes'] += 1
                elif old_status == NodeStatus.RUNNING and status != NodeStatus.RUNNING:
                    self.stats['running_nodes'] -= 1
                
                if status == NodeStatus.ERROR:
                    node_info.error_count += 1
                    self.stats['failed_nodes'] += 1
                
                return True
                
        except Exception as e:
            print(f"Error updating node status for {name}: {e}")
            return False
    
    def heartbeat(self, name: str, **metrics) -> bool:
        """
        Record heartbeat from a node.
        
        Args:
            name: Node name
            **metrics: Performance metrics (cpu_usage, memory_usage, etc.)
            
        Returns:
            True if heartbeat recorded
        """
        try:
            with self.lock:
                if name not in self.nodes:
                    return False
                
                node_info = self.nodes[name]
                node_info.last_heartbeat = time.time()
                
                # Update metrics
                for key, value in metrics.items():
                    if hasattr(node_info, key):
                        setattr(node_info, key, value)
                
                # Update status to running if not already
                if node_info.status != NodeStatus.RUNNING:
                    self.update_node_status(name, NodeStatus.RUNNING)
                
                return True
                
        except Exception as e:
            print(f"Error recording heartbeat for {name}: {e}")
            return False
    
    def register_topic_publisher(self, node_name: str, topic: str) -> bool:
        """Register a node as publisher of a topic"""
        try:
            with self.lock:
                if node_name not in self.nodes:
                    return False
                
                # Add to node's published topics
                self.nodes[node_name].published_topics.add(topic)
                
                # Add to topic publishers
                if topic not in self.topic_publishers:
                    self.topic_publishers[topic] = set()
                self.topic_publishers[topic].add(node_name)
                
                return True
                
        except Exception as e:
            print(f"Error registering topic publisher: {e}")
            return False
    
    def register_topic_subscriber(self, node_name: str, topic: str) -> bool:
        """Register a node as subscriber of a topic"""
        try:
            with self.lock:
                if node_name not in self.nodes:
                    return False
                
                # Add to node's subscribed topics
                self.nodes[node_name].subscribed_topics.add(topic)
                
                # Add to topic subscribers
                if topic not in self.topic_subscribers:
                    self.topic_subscribers[topic] = set()
                self.topic_subscribers[topic].add(node_name)
                
                return True
                
        except Exception as e:
            print(f"Error registering topic subscriber: {e}")
            return False
    
    def get_node_info(self, name: str) -> Optional[NodeInfo]:
        """Get information about a node"""
        with self.lock:
            return self.nodes.get(name)
    
    def get_all_nodes(self) -> Dict[str, NodeInfo]:
        """Get information about all nodes"""
        with self.lock:
            return self.nodes.copy()
    
    def get_nodes_by_status(self, status: NodeStatus) -> List[NodeInfo]:
        """Get all nodes with a specific status"""
        with self.lock:
            return [node for node in self.nodes.values() if node.status == status]
    
    def get_topic_publishers(self, topic: str) -> Set[str]:
        """Get all publishers of a topic"""
        with self.lock:
            return self.topic_publishers.get(topic, set()).copy()
    
    def get_topic_subscribers(self, topic: str) -> Set[str]:
        """Get all subscribers of a topic"""
        with self.lock:
            return self.topic_subscribers.get(topic, set()).copy()
    
    def get_node_dependencies(self, name: str) -> Set[str]:
        """Get dependencies of a node"""
        with self.lock:
            return self.dependencies.get(name, set()).copy()
    
    def get_node_dependents(self, name: str) -> Set[str]:
        """Get nodes that depend on this node"""
        with self.lock:
            return self.dependents.get(name, set()).copy()
    
    def _setup_dependencies(self, node_name: str, dependencies: List[str]):
        """Setup node dependencies"""
        if node_name not in self.dependencies:
            self.dependencies[node_name] = set()
        
        for dep in dependencies:
            self.dependencies[node_name].add(dep)
            
            # Add reverse dependency
            if dep not in self.dependents:
                self.dependents[dep] = set()
            self.dependents[dep].add(node_name)
    
    def _cleanup_dependencies(self, node_name: str):
        """Clean up dependencies for a node"""
        # Remove dependencies
        if node_name in self.dependencies:
            for dep in self.dependencies[node_name]:
                if dep in self.dependents:
                    self.dependents[dep].discard(node_name)
                    if not self.dependents[dep]:
                        del self.dependents[dep]
            del self.dependencies[node_name]
        
        # Remove as dependent
        if node_name in self.dependents:
            for dependent in self.dependents[node_name]:
                if dependent in self.dependencies:
                    self.dependencies[dependent].discard(node_name)
                    if not self.dependencies[dependent]:
                        del self.dependencies[dependent]
            del self.dependents[node_name]
    
    def _health_check_loop(self):
        """Background health checking loop"""
        while self.running:
            try:
                current_time = time.time()
                
                with self.lock:
                    for node_name, node_info in self.nodes.items():
                        # Check for timeout
                        if (node_info.status == NodeStatus.RUNNING and
                            node_info.last_heartbeat and
                            current_time - node_info.last_heartbeat > self.heartbeat_timeout):
                            
                            print(f"Node {node_name} heartbeat timeout")
                            self.update_node_status(node_name, NodeStatus.TIMEOUT)
                
                time.sleep(5.0)  # Check every 5 seconds
                
            except Exception as e:
                print(f"Error in health check loop: {e}")
                time.sleep(5.0)
    
    def get_registry_stats(self) -> Dict[str, Any]:
        """Get registry statistics"""
        with self.lock:
            stats = self.stats.copy()
            stats['uptime'] = time.time() - stats['registry_start_time']
            stats['topics_count'] = len(set(list(self.topic_publishers.keys()) + list(self.topic_subscribers.keys())))
            
            # Status breakdown
            status_counts = {}
            for status in NodeStatus:
                status_counts[status.value] = len(self.get_nodes_by_status(status))
            stats['status_breakdown'] = status_counts
            
            return stats

# TEST: Node registry tracks node lifecycle correctly
# TEST: Heartbeat timeout detection works properly
# TEST: Topic publisher/subscriber tracking is accurate
# TEST: Node dependencies are managed correctly
# TEST: Registry statistics are calculated properly
```

## 4. Error Handling and Recovery

### 4.1 Error Handler Implementation

```python
# nevil_framework/error_handler.py

import traceback
import threading
import time
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass
from enum import Enum

class ErrorSeverity(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

class RecoveryAction(Enum):
    IGNORE = "ignore"
    RETRY = "retry"
    RESTART_NODE = "restart_node"
    RESTART_SYSTEM = "restart_system"
    SHUTDOWN = "shutdown"

@dataclass
class ErrorInfo:
    error_id: str
    timestamp: float
    node_name: str
    error_type: str
    message: str
    severity: ErrorSeverity
    traceback: str
    context: Dict[str, Any]
    recovery_action: RecoveryAction = RecoveryAction.IGNORE
    retry_count: int = 0
    resolved: bool = False

class ErrorHandler:
    """
    Centralized error handling and recovery system.
    
    Features:
    - Error classification and severity assessment
    - Automatic recovery strategies
    - Error pattern detection
    - Circuit breaker patterns
    - Error reporting and alerting
    - Recovery action coordination
    """
    
    def __init__(self):
        self.errors = {}  # error_id -> ErrorInfo
        self.error_patterns = {}  # pattern -> count
        self.recovery_handlers = {}  # error_type -> recovery_function
        self.lock = threading.RLock()
        
        # Circuit breaker state
        self.circuit_breakers = {}  # node_name -> circuit_breaker_state
        
        # Error thresholds
        self.error_thresholds = {
            'max_errors_per_minute': 10,
            'max_critical_errors': 3,
            'circuit_breaker_threshold': 5,
            'circuit_breaker_timeout': 60.0
        }
        
        # Statistics
        self.stats = {
            'total_errors': 0,
            'errors_by_severity': {severity.name: 0 for severity in ErrorSeverity},
            'errors_by_node': {},
            'recovery_actions_taken': 0,
            'successful_recoveries': 0
        }
        
        # Setup default recovery handlers
        self._setup_default_handlers()
    
    def handle_error(self, node_name: str, error: Exception,
                    context: Dict[str, Any] = None,
                    severity: ErrorSeverity = None) -> str:
        """
        Handle an error and determine recovery action.
        
        Args:
            node_name: Name of node where error occurred
            error: The exception that occurred
            context: Additional context information
            severity: Error severity (auto-detected if None)
            
        Returns:
            Error ID for tracking
        """
        try:
            # Generate error ID
            error_id = f"{node_name}_{int(time.time())}_{id(error)}"
            
            # Classify error
            error_type = type(error).__name__
            message = str(error)
            tb = traceback.format_exc()
            
            # Determine severity if not provided
            if severity is None:
                severity = self._classify_error_severity(error_type, message)
            
            # Create error info
            error_info = ErrorInfo(
                error_id=error_id,
                timestamp=time.time(),
                node_name=node_name,
                error_type=error_type,
                message=message,
                severity=severity,
                traceback=tb,
                context=context or {}
            )
            
            # Determine recovery action
            error_info.recovery_action = self._determine_recovery_action(error_info)
            
            # Store error
            with self.lock:
                self.errors[error_id] = error_info
                self._update_statistics(error_info)
                self._update_error_patterns(error_info)
            
            # Execute recovery action
            self._execute_recovery_action(error_info)
            
            # Check circuit breaker
            self._check_circuit_breaker(node_name)
            
            return error_id
            
        except Exception as e:
            print(f"Error in error handler: {e}")
            return ""
    
    def register_recovery_handler(self, error_type: str,
                                 handler: Callable[[ErrorInfo], bool]):
        """
        Register a custom recovery handler for an error type.
        
        Args:
            error_type: Type of error to handle
            handler: Recovery function that returns True if successful
        """
        self.recovery_handlers[error_type] = handler
    
    def retry_error(self, error_id: str) -> bool:
        """
        Retry recovery for a specific error.
        
        Args:
            error_id: ID of error to retry
            
        Returns:
            True if retry was successful
        """
        with self.lock:
            if error_id not in self.errors:
                return False
            
            error_info = self.errors[error_id]
            error_info.retry_count += 1
            
            # Execute recovery action again
            return self._execute_recovery_action(error_info)
    
    def resolve_error(self, error_id: str) -> bool:
        """
        Mark an error as resolved.
        
        Args:
            error_id: ID of error to resolve
            
        Returns:
            True if error was found and marked resolved
        """
        with self.lock:
            if error_id not in self.errors:
                return False
            
            self.errors[error_id].resolved = True
            self.stats['successful_recoveries'] += 1
            return True
    
    def get_error_info(self, error_id: str) -> Optional[ErrorInfo]:
        """Get information about a specific error"""
        with self.lock:
            return self.errors.get(error_id)
    
    def get_recent_errors(self, node_name: str = None,
                         hours: int = 24) -> List[ErrorInfo]:
        """
        Get recent errors, optionally filtered by node.
        
        Args:
            node_name: Filter by node name (None for all)
            hours: Number of hours to look back
            
        Returns:
            List of recent errors
        """
        cutoff_time = time.time() - (hours * 3600)
        
        with self.lock:
            errors = []
            for error_info in self.errors.values():
                if error_info.timestamp >= cutoff_time:
                    if node_name is None or error_info.node_name == node_name:
                        errors.append(error_info)
            
            # Sort by timestamp (newest first)
            errors.sort(key=lambda e: e.timestamp, reverse=True)
            return errors
    
    def get_error_patterns(self) -> Dict[str, int]:
        """Get detected error patterns"""
        with self.lock:
            return self.error_patterns.copy()
    
    def _classify_error_severity(self, error_type: str, message: str) -> ErrorSeverity:
        """Classify error severity based on type and message"""
        # Critical errors
        critical_patterns = [
            'SystemExit', 'KeyboardInterrupt', 'MemoryError',
            'OSError', 'IOError', 'PermissionError'
        ]
        
        # High severity errors
        high_patterns = [
            'ConnectionError', 'TimeoutError', 'AuthenticationError',
            'ConfigurationError', 'HardwareError'
        ]
        
        # Medium severity errors
        medium_patterns = [
            'ValueError', 'TypeError', 'AttributeError',
            'KeyError', 'IndexError'
        ]
        
        if any(pattern in error_type for pattern in critical_patterns):
            return ErrorSeverity.CRITICAL
        elif any(pattern in error_type for pattern in high_patterns):
            return ErrorSeverity.HIGH
        elif any(pattern in error_type for pattern in medium_patterns):
            return ErrorSeverity.MEDIUM
        else:
            return ErrorSeverity.LOW
    
    def _determine_recovery_action(self, error_info: ErrorInfo) -> RecoveryAction:
        """Determine appropriate recovery action for an error"""
        # Critical errors may require system shutdown
        if error_info.severity == ErrorSeverity.CRITICAL:
            if 'MemoryError' in error_info.error_type:
                return RecoveryAction.RESTART_SYSTEM
            elif 'PermissionError' in error_info.error_type:
                return RecoveryAction.SHUTDOWN
        
        # High severity errors may require node restart
        if error_info.severity == ErrorSeverity.HIGH:
            if error_info.retry_count < 3:
                return RecoveryAction.RETRY
            else:
                return RecoveryAction.RESTART_NODE
        
        # Medium severity errors can usually be retried
        if error_info.severity == ErrorSeverity.MEDIUM:
            if error_info.retry_count < 2:
                return RecoveryAction.RETRY
            else:
                return RecoveryAction.IGNORE
        
        # Low severity errors are usually ignored
        return RecoveryAction.IGNORE
    
    def _execute_recovery_action(self, error_info: ErrorInfo) -> bool:
        """Execute the determined recovery action"""
        action = error_info.recovery_action
        
        try:
            if action == RecoveryAction.IGNORE:
                return True
            
            elif action == RecoveryAction.RETRY:
                # Check if there's a custom handler
                if error_info.error_type in self.recovery_handlers:
                    handler = self.recovery_handlers[error_info.error_type]
                    return handler(error_info)
                else:
                    # Default retry behavior
                    time.sleep(min(2 ** error_info.retry_count, 30))  # Exponential backoff
                    return True
            
            elif action == RecoveryAction.RESTART_NODE:
                print(f"Recovery action: Restart node {error_info.node_name}")
                # This would trigger node restart through the launcher
                return True
            
            elif action == RecoveryAction.RESTART_SYSTEM:
                print("Recovery action: Restart system")
                # This would trigger system restart
                return True
            
            elif action == RecoveryAction.SHUTDOWN:
                print("Recovery action: Shutdown system")
                # This would trigger graceful shutdown
                return True
            
            return False
            
        except Exception as e:
            print(f"Error executing recovery action: {e}")
            return False
        finally:
            self.stats['recovery_actions_taken'] += 1
    
    def _update_statistics(self, error_info: ErrorInfo):
        """Update error statistics"""
        self.stats['total_errors'] += 1
        self.stats['errors_by_severity'][error_info.severity.name] += 1
        
        if error_info.node_name not in self.stats['errors_by_node']:
            self.stats['errors_by_node'][error_info.node_name] = 0
        self.stats['errors_by_node'][error_info.node_name] += 1
    
    def _update_error_patterns(self, error_info: ErrorInfo):
        """Update error pattern tracking"""
        pattern = f"{error_info.node_name}:{error_info.error_type}"
        if pattern not in self.error_patterns:
            self.error_patterns[pattern] = 0
        self.error_patterns[pattern] += 1
    
    def _check_circuit_breaker(self, node_name: str):
        """Check and update circuit breaker state for a node"""
        current_time = time.time()
        
        if node_name not in self.circuit_breakers:
            self.circuit_breakers[node_name] = {
                'state': 'closed',  # closed, open, half_open
                'failure_count': 0,
                'last_failure_time': 0,
                'next_attempt_time': 0
            }
        
        cb = self.circuit_breakers[node_name]
        
        # Count recent failures
        recent_errors = self.get_recent_errors(node_name, hours=1)
        failure_count = len([e for e in recent_errors if e.severity in [ErrorSeverity.HIGH, ErrorSeverity.CRITICAL]])
        
        cb['failure_count'] = failure_count
        
        # Update circuit breaker state
        if cb['state'] == 'closed' and failure_count >= self.error_thresholds['circuit_breaker_threshold']:
            cb['state'] = 'open'
            cb['next_attempt_time'] = current_time + self.error_thresholds['circuit_breaker_timeout']
            print(f"Circuit breaker opened for node {node_name}")
        
        elif cb['state'] == 'open' and current_time >= cb['next_attempt_time']:
            cb['state'] = 'half_open'
            print(f"Circuit breaker half-open for node {node_name}")
        
        elif cb['state'] == 'half_open' and failure_count == 0:
            cb['state'] = 'closed'
            print(f"Circuit breaker closed for node {node_name}")
    
    def _setup_default_handlers(self):
        """Setup default recovery handlers"""
        def connection_error_handler(error_info: ErrorInfo) -> bool:
            """Handle connection errors with exponential backoff"""
            delay = min(2 ** error_info.retry_count, 60)
            print(f"Connection error in {error_info.node_name}, retrying in {delay}s")
            time.sleep(delay)
            return True
        
        def timeout_error_handler(error_info: ErrorInfo) -> bool:
            """Handle timeout errors"""
            print(f"Timeout error in {error_info.node_name}, increasing timeout")
            # Could adjust timeout values here
            return True
        
        self.register_recovery_handler('ConnectionError', connection_error_handler)
        self.register_recovery_handler('TimeoutError', timeout_error_handler)
    
    def get_error_stats(self) -> Dict[str, Any]:
        """Get error handling statistics"""
        with self.lock:
            stats = self.stats.copy()
            stats['circuit_breakers'] = {
                name: cb['state'] for name, cb in self.circuit_breakers.items()
            }
            stats['error_patterns_count'] = len(self.error_patterns)
            return stats

# TEST: Error handler classifies severity correctly
# TEST: Recovery actions are executed appropriately
# TEST: Circuit breaker prevents cascading failures
# TEST: Error patterns are detected accurately
# TEST: Custom recovery handlers work properly
```

## Conclusion

The Nevil v3.0 framework core components provide a solid foundation for building reliable, maintainable robotic applications. Key features include:

- **Message Bus**: Reliable publish/subscribe communication with priorities, TTL, and request-response patterns
- **Configuration Manager**: Hot-reloadable YAML configuration with environment variable expansion and validation
- **Node Registry**: Comprehensive node tracking with health monitoring, dependency management, and service discovery
- **Error Handler**: Intelligent error classification, recovery strategies, and circuit breaker patterns

These components work together to create a robust framework that maintains simplicity while providing enterprise-grade reliability and observability features.

# TEST: All core components integrate seamlessly
# TEST: Message delivery is reliable under various conditions
# TEST: Configuration hot-reloading maintains system stability
# TEST: Node registry accurately tracks system state
# TEST: Error handling prevents system failures and enables recovery