# Nevil v3.0 Declarative Node Creation Architecture

## Code Summary

This document contains comprehensive declarative node creation implementations across 2 major code sections:

### Section 3.1: DeclarativeLauncher Class
**Declarative node creation engine** - Complete launcher implementation with node discovery, automatic creation based on .nodes files, dependency resolution, and lifecycle management.

### Section 3.2: EnhancedNevilLauncher Integration
**Framework integration layer** - Enhanced launcher combining system-wide configuration with declarative node creation, process management integration, and monitoring capabilities.

## Overview

This document specifies the declarative node creation system for Nevil v3.0, which extends the declarative messaging pattern to node lifecycle management. Just as `.messages` files handle messaging setup declaratively, individual `.nodes` files handle node creation and configuration declaratively through the Launch process.

**Core Philosophy**: Complete declarative configuration - `.messages` for messaging, `.nodes` for node creation, all handled automatically by their respective systems.

## 1. Architecture Comparison

### 1.1 Current vs. New Pattern

#### Current Pattern (System-wide .nodes file)
```yaml
# Root .nodes file - System configuration
nodes:
  speech_recognition:
    status: live
    priority: high
    restart_policy: always
  speech_synthesis:
    status: live
    priority: high
```

#### New Pattern (Individual .nodes files)
```yaml
# nodes/speech_recognition/.nodes - Individual node configuration
node_name: "speech_recognition"
version: "1.0"
description: "Speech recognition node with OpenAI Whisper integration"

# Node creation specification
creation:
  class_name: "SpeechRecognitionNode"
  module_path: "nodes.speech_recognition.speech_recognition_node"
  auto_start: true
  
# Runtime configuration
runtime:
  status: live
  priority: high
  restart_policy: always
  max_restarts: 5
  restart_delay: 2.0

# Environment and resources
environment:
  OPENAI_API_KEY: "${OPENAI_API_KEY}"
  SPEECH_RECOGNITION_LANGUAGE: "en-US"
  MICROPHONE_DEVICE_INDEX: "1"

resources:
  max_memory_mb: 200
  max_cpu_percent: 25
  max_threads: 10

# Dependencies
dependencies:
  requires: []
  provides: ["speech_recognition"]
  
# Health monitoring
health_check:
  enabled: true
  timeout: 5.0
  interval: 10.0
  failure_threshold: 3
```

### 1.2 Consistency with Declarative Messaging

Both systems follow the same declarative pattern:

| Aspect | `.messages` Files | `.nodes` Files |
|--------|------------------|----------------|
| **Purpose** | Declare messaging interface | Declare node creation |
| **Location** | `nodes/{name}/.messages` | `nodes/{name}/.nodes` |
| **Handler** | NevilNode.init_messages() | Launch.create_nodes() |
| **Timing** | Node initialization | System startup |
| **Scope** | Per-node messaging | Per-node creation |

## 2. Declarative Node Creation Specification

### 2.1 Individual .nodes File Format

```yaml
# nodes/{node_name}/.nodes
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

### 2.2 Speech Synthesis Node Example

```yaml
# nodes/speech_synthesis/.nodes
node_name: "speech_synthesis"
version: "1.0"
description: "Converts text to speech using OpenAI TTS"

creation:
  class_name: "SpeechSynthesisNode"
  module_path: "nodes.speech_synthesis.speech_synthesis_node"
  auto_start: true

runtime:
  status: live
  priority: high
  restart_policy: always
  max_restarts: 5

environment:
  OPENAI_API_KEY: "${OPENAI_API_KEY}"
  TTS_VOICE: "onyx"
  SPEECH_SYNTHESIS_VOLUME: "6"
  SPEECH_SYNTHESIS_RATE: "200"

resources:
  max_memory_mb: 250
  max_cpu_percent: 30
  max_threads: 8

dependencies:
  requires: []
  provides: ["speech_synthesis", "audio_output"]

health_check:
  enabled: true
  timeout: 5.0
  interval: 10.0
  method: "check_audio_output_health"

security:
  allowed_paths:
    - "./audio/"
    - "./logs/"
  network_access: true
  allowed_hosts:
    - "api.openai.com"
```

### 2.3 AI Cognition Node Example

```yaml
# nodes/ai_cognition/.nodes
node_name: "ai_cognition"
version: "1.0"
description: "AI-powered conversation and decision making"

creation:
  class_name: "AICognitionNode"
  module_path: "nodes.ai_cognition.ai_cognition_node"
  auto_start: true

runtime:
  status: live
  priority: medium
  restart_policy: on_failure
  max_restarts: 3

environment:
  OPENAI_API_KEY: "${OPENAI_API_KEY}"
  OPENAI_ASSISTANT_ID: "${OPENAI_ASSISTANT_ID}"
  GPT_MODEL: "gpt-3.5-turbo"
  MAX_TOKENS: "150"
  TEMPERATURE: "0.7"

resources:
  max_memory_mb: 300
  max_cpu_percent: 40
  max_threads: 5

dependencies:
  requires: ["speech_recognition", "speech_synthesis"]
  provides: ["ai_cognition", "conversation_management"]

health_check:
  enabled: true
  timeout: 10.0
  interval: 15.0
  failure_threshold: 2
  method: "check_ai_service_health"
```

## 3. Launch System Integration

### 3.1 Updated Launch Process

```python
# nevil_framework/declarative_launcher.py

class DeclarativeLauncher:
    """
    Enhanced launcher that handles declarative node creation via .nodes files.
    
    Follows the same pattern as declarative messaging:
    - Reads individual .nodes files
    - Creates nodes automatically
    - Manages lifecycle declaratively
    """
    
    def __init__(self, nodes_root_path: str = "nodes"):
        self.nodes_root_path = nodes_root_path
        self.node_configs = {}  # node_name -> config
        self.created_nodes = {}  # node_name -> node_instance
        self.node_processes = {}  # node_name -> process_info
        
    def discover_nodes(self) -> Dict[str, Dict]:
        """
        Discover all .nodes files and load configurations.
        
        Similar to how .messages files are discovered and loaded.
        """
        discovered_configs = {}
        
        nodes_path = Path(self.nodes_root_path)
        if not nodes_path.exists():
            return discovered_configs
        
        # Scan for .nodes files
        for node_dir in nodes_path.iterdir():
            if node_dir.is_dir():
                nodes_file = node_dir / ".nodes"
                if nodes_file.exists():
                    try:
                        config = self._load_node_config(nodes_file)
                        node_name = config.get('node_name', node_dir.name)
                        discovered_configs[node_name] = config
                        
                    except Exception as e:
                        self.logger.error(f"Error loading .nodes file {nodes_file}: {e}")
        
        return discovered_configs
    
    def create_nodes(self) -> bool:
        """
        Create all nodes declaratively based on .nodes files.
        
        This is the equivalent of init_messages() for node creation.
        """
        try:
            # Discover all node configurations
            self.node_configs = self.discover_nodes()
            
            if not self.node_configs:
                self.logger.warning("No .nodes files found")
                return True
            
            # Calculate creation order based on dependencies
            creation_order = self._calculate_creation_order()
            
            # Create nodes in dependency order
            for node_name in creation_order:
                if not self._create_single_node(node_name):
                    self.logger.error(f"Failed to create node: {node_name}")
                    return False
            
            self.logger.info(f"Successfully created {len(self.created_nodes)} nodes declaratively")
            return True
            
        except Exception as e:
            self.logger.error(f"Error in declarative node creation: {e}")
            return False
    
    def _create_single_node(self, node_name: str) -> bool:
        """
        Create a single node based on its .nodes configuration.
        
        Equivalent to setting up a single subscription in init_messages().
        """
        try:
            config = self.node_configs[node_name]
            creation_config = config.get('creation', {})
            
            # Skip if not auto_start
            if not creation_config.get('auto_start', True):
                self.logger.info(f"Skipping node {node_name} (auto_start=false)")
                return True
            
            # Skip if disabled
            runtime_config = config.get('runtime', {})
            if runtime_config.get('status') == 'disabled':
                self.logger.info(f"Skipping disabled node: {node_name}")
                return True
            
            # Import and instantiate node class
            module_path = creation_config.get('module_path')
            class_name = creation_config.get('class_name')
            
            if not module_path or not class_name:
                self.logger.error(f"Missing module_path or class_name for {node_name}")
                return False
            
            # Dynamic import
            module = importlib.import_module(module_path)
            node_class = getattr(module, class_name)
            
            # Prepare initialization parameters
            init_params = creation_config.get('init_params', {})
            init_params['node_name'] = node_name
            
            # Create node instance
            node_instance = node_class(**init_params)
            
            # Store created node
            self.created_nodes[node_name] = node_instance
            
            # Start node if configured for isolated process
            if runtime_config.get('isolated_process', True):
                self._start_node_process(node_name, node_instance, config)
            
            self.logger.info(f"Created node: {node_name}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error creating node {node_name}: {e}")
            return False
    
    def _calculate_creation_order(self) -> List[str]:
        """
        Calculate node creation order based on dependencies.
        
        Similar to message subscription ordering.
        """
        # Build dependency graph
        graph = {}
        in_degree = {}
        
        for node_name, config in self.node_configs.items():
            dependencies = config.get('dependencies', {})
            requires = dependencies.get('requires', [])
            
            graph[node_name] = requires
            in_degree[node_name] = 0
        
        # Calculate in-degrees
        for node_name, deps in graph.items():
            for dep in deps:
                if dep in in_degree:
                    in_degree[node_name] += 1
                else:
                    self.logger.warning(f"Unknown dependency '{dep}' for node '{node_name}'")
        
        # Topological sort (Kahn's algorithm)
        queue = [node for node, degree in in_degree.items() if degree == 0]
        result = []
        
        while queue:
            node = queue.pop(0)
            result.append(node)
            
            for other_node, deps in graph.items():
                if node in deps:
                    in_degree[other_node] -= 1
                    if in_degree[other_node] == 0:
                        queue.append(other_node)
        
        # Check for circular dependencies
        if len(result) != len(self.node_configs):
            remaining = set(self.node_configs.keys()) - set(result)
            raise ValueError(f"Circular dependency detected among nodes: {remaining}")
        
        return result
```

### 3.2 Integration with Existing Launch System

```python
# nevil_framework/enhanced_launcher.py

class EnhancedNevilLauncher(NevilLauncher):
    """
    Enhanced launcher that combines system-wide configuration with 
    declarative node creation.
    """
    
    def __init__(self, config_path: str = ".nodes"):
        super().__init__(config_path)
        self.declarative_launcher = DeclarativeLauncher()
        
    def _start_nodes(self):
        """
        Enhanced node starting that uses declarative creation.
        """
        self.logger.info("Starting declarative node creation...")
        
        # Use declarative launcher to create nodes
        if not self.declarative_launcher.create_nodes():
            raise RuntimeError("Declarative node creation failed")
        
        # Get created nodes and integrate with process management
        created_nodes = self.declarative_launcher.created_nodes
        
        for node_name, node_instance in created_nodes.items():
            # Integrate with existing process management
            self._integrate_declarative_node(node_name, node_instance)
        
        self.logger.info(f"Declarative node creation completed: {len(created_nodes)} nodes")
    
    def _integrate_declarative_node(self, node_name: str, node_instance):
        """
        Integrate declaratively created node with process management.
        """
        # Get node configuration
        config = self.declarative_launcher.node_configs.get(node_name, {})
        runtime_config = config.get('runtime', {})
        
        # Create process record for monitoring
        if runtime_config.get('isolated_process', True):
            # Node runs in separate process - already handled
            pass
        else:
            # Node runs in main process - register for monitoring
            self._register_in_process_node(node_name, node_instance, config)
```

## 4. Removal of Manual Node Management

### 4.1 Eliminated Methods

The following manual node management methods are removed from the framework:

```python
# REMOVED: Manual node creation methods
class NodeManager:
    def add_node(self, node_name: str, node_class: type) -> bool:
        # REMOVED - replaced by declarative creation
        pass
    
    def remove_node(self, node_name: str) -> bool:
        # REMOVED - replaced by declarative lifecycle
        pass
    
    def create_node_instance(self, node_name: str, **kwargs):
        # REMOVED - replaced by .nodes file configuration
        pass
    
    def register_node_class(self, node_class: type):
        # REMOVED - replaced by module_path in .nodes files
        pass
```

### 4.2 Declarative Replacement

All node management is now handled declaratively:

```python
# NEW: Declarative approach
# 1. Create nodes/{node_name}/.nodes file
# 2. Launch system automatically discovers and creates nodes
# 3. No manual method calls needed

# Example: Adding a new node
# OLD WAY:
# node_manager.add_node("new_sensor", NewSensorNode)
# node_manager.create_node_instance("new_sensor", config=sensor_config)

# NEW WAY:
# 1. Create nodes/new_sensor/.nodes file with configuration
# 2. Launch system automatically creates node on next startup
```

## 5. Complete Declarative Pattern

### 5.1 Unified Declarative Architecture

```
nodes/
├── speech_recognition/
│   ├── .nodes                    # Declarative node creation
│   ├── .messages                 # Declarative messaging setup
│   └── speech_recognition_node.py
├── speech_synthesis/
│   ├── .nodes                    # Declarative node creation
│   ├── .messages                 # Declarative messaging setup
│   └── speech_synthesis_node.py
└── ai_cognition/
    ├── .nodes                    # Declarative node creation
    ├── .messages                 # Declarative messaging setup
    └── ai_cognition_node.py
```

### 5.2 Declarative Lifecycle

```
System Startup:
1. Launch system reads all .nodes files
2. Creates nodes in dependency order
3. Each node reads its .messages file
4. Messaging is configured automatically
5. System is ready - no manual setup needed

Node Addition:
1. Create new node directory
2. Add .nodes file with configuration
3. Add .messages file with interface
4. Restart system - node is automatically created

Node Removal:
1. Set status: disabled in .nodes file
2. Node is automatically excluded from creation
```

## 6. Benefits of Declarative Node Creation

### 6.1 Consistency
- Same pattern as declarative messaging
- Configuration over code
- Single source of truth per node

### 6.2 Simplicity
- No manual AddNode/RemoveNode methods
- Automatic dependency resolution
- Self-documenting configuration

### 6.3 Reliability
- Validation at startup
- Dependency checking
- Consistent creation process

### 6.4 Maintainability
- Easy to add/remove nodes
- Clear configuration files
- Version-controlled setup

## 7. Migration Guide

### 7.1 Converting Existing Nodes

```python
# OLD: Manual node creation
class SystemManager:
    def setup_nodes(self):
        self.add_node("speech_recognition", SpeechRecognitionNode)
        self.add_node("speech_synthesis", SpeechSynthesisNode)
        self.add_node("ai_cognition", AICognitionNode)

# NEW: Declarative node creation
# 1. Create .nodes files for each node
# 2. Remove manual setup code
# 3. Let launch system handle creation
```

### 7.2 Configuration Migration

```yaml
# OLD: System-wide .nodes file
nodes:
  speech_recognition:
    status: live
    priority: high

# NEW: Individual .nodes files
# nodes/speech_recognition/.nodes
node_name: "speech_recognition"
creation:
  class_name: "SpeechRecognitionNode"
  module_path: "nodes.speech_recognition.speech_recognition_node"
runtime:
  status: live
  priority: high
```

## Conclusion

The declarative node creation architecture completes the vision of a fully declarative Nevil v3.0 framework:

- **`.messages` files** handle messaging setup declaratively
- **`.nodes` files** handle node creation declaratively  
- **Launch system** orchestrates everything automatically
- **No manual methods** needed for node lifecycle management

This creates a consistent, maintainable, and reliable system where all configuration is declarative and all setup is automatic, achieving the goal of "simple architecture = working robot" through configuration over code.