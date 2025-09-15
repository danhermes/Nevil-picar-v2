# Nevil v3.0 Launch System Architecture - Phase 2 Features

## Overview

This document contains the Phase 2 advanced features for the Nevil v3.0 launch system, including enhanced CLI commands and monitoring capabilities.

## 4. Command Line Interface - Phase 2

### 4.1 Enhanced CLI Commands

The Nevil v3.0 system includes additional Phase 2 CLI commands for advanced monitoring and management:

```bash
./nevil start                 # Start the system
./nevil start --logscope      # Start with monitoring dashboard
./nevil stop                  # Graceful shutdown
./nevil status                # Show system health
./nevil logs [node]           # View logs

# Phase 2 Commands:
./nevil start --logscope      # Start with monitoring 
./nevil monitor               # Launch LogScope dashboard
./nevil validate              # Validate configuration
./nevil init <project>        # Create new project
```

### 4.2 Integration with Launch System

The Phase 2 CLI integrates seamlessly with the NevilLauncher:

1. **Configuration Validation**: Validates `.nodes` and `.messages` files before launch
2. **Process Management**: Manages system and LogScope processes with PID tracking
3. **Health Monitoring**: Displays real-time system status and node health
4. **Log Management**: Provides access to the enhanced logging system with KEEP_LOG_FILES=3
5. **Project Creation**: Scaffolds new projects with proper configuration templates

## Conclusion

The Phase 2 CLI enhancements provide advanced monitoring and management capabilities while maintaining the simplicity of the core launch system.

# TEST: Enhanced CLI commands work correctly in all scenarios
# TEST: LogScope integration launches monitoring dashboard properly
# TEST: Configuration validation catches errors before system start
# TEST: Project initialization creates proper templates

## 5. Detailed Phase 2 CLI Implementation

### 5.1 Monitor Command (Phase 2)

The `monitor` command launches the LogScope monitoring dashboard for advanced system monitoring:

```bash
./nevil monitor
```

**Features:**
- Checks for PyQt6/PyQt5 availability
- Launches GUI dashboard for real-time monitoring
- Provides advanced filtering and search capabilities

### 5.2 LogScope Integration Functions (Phase 2)

#### launch_logscope() Function

```bash
# Launch LogScope monitoring dashboard (Phase 2)
launch_logscope() {
    local auto_launch="$1"

    log_info "Launching LogScope monitoring dashboard..."

    # Check if logs directory exists
    if [ ! -d "logs" ]; then
        log_warning "Logs directory not found, creating..."
        mkdir -p logs
    fi

    # Check if PyQt is available
    if ! $PYTHON_CMD -c "
try:
    import PyQt6
    print('PyQt6 available')
except ImportError:
    try:
        import PyQt5
        print('PyQt5 available')
    except ImportError:
        print('ERROR: PyQt not available')
        import sys
        sys.exit(1)
" 2>/dev/null; then
        log_error "LogScope requires PyQt6 or PyQt5. Install with: pip install PyQt6"
        if [ "$auto_launch" = "auto" ]; then
            log_warning "Continuing without LogScope..."
            return 0
        else
            return 1
        fi
    fi

    # Launch LogScope in background
    $PYTHON_CMD -m nevil_framework.logscope.launcher --log-dir logs --theme dark > /dev/null 2>&1 &
    local logscope_pid=$!

    # Save LogScope PID
    echo $logscope_pid > logscope.pid

    # Brief delay to check if it started successfully
    sleep 2

    if kill -0 "$logscope_pid" 2>/dev/null; then
        log_success "LogScope dashboard launched (PID: $logscope_pid)"
        log_info "LogScope GUI should open shortly..."
    else
        log_error "Failed to start LogScope dashboard"
        rm -f logscope.pid
        return 1
    fi
}

# Stop LogScope dashboard
stop_logscope() {
    if [ -f "logscope.pid" ]; then
        local pid=$(cat logscope.pid)
        if kill -0 "$pid" 2>/dev/null; then
            log_info "Stopping LogScope dashboard (PID: $pid)"
            kill -TERM "$pid" 2>/dev/null

            # Wait briefly for graceful shutdown
            local count=0
            while kill -0 "$pid" 2>/dev/null && [ $count -lt 5 ]; do
                sleep 1
                count=$((count + 1))
            done

            # Force kill if still running
            if kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$pid" 2>/dev/null
            fi

            log_success "LogScope dashboard stopped"
        fi
        rm -f logscope.pid
    fi
}
```

### 5.3 Project Initialization (Phase 2)

#### init_project() Function

```bash
# Initialize new project (phase 2)
init_project() {
    local project_name="$1"

    if [ -z "$project_name" ]; then
        log_error "Project name is required"
        echo "Usage: nevil init <project_name>"
        return 1
    fi

    if [ -d "$project_name" ]; then
        log_error "Directory $project_name already exists"
        return 1
    fi

    log_info "Creating new Nevil v3.0 project: $project_name"

    # Create project structure
    mkdir -p "$project_name"/{nevil_framework,nodes,audio,logs,tests}
    mkdir -p "$project_name"/nodes/{speech_recognition,speech_synthesis,ai_cognition}

    # Create basic configuration files
    cat > "$project_name/.nodes" << 'EOF'
version: "3.0"
description: "New Nevil v3.0 project"

system:
  framework_version: "3.0.0"
  log_level: "INFO"
  health_check_interval: 5.0
  shutdown_timeout: 10.0
  startup_delay: 2.0
  monitoring:
    logscope_enabled: false
    logscope_theme: "dark"
    max_entries: 10000

environment:
  NEVIL_VERSION: "3.0"
  PYTHONPATH: "${PYTHONPATH}:./nevil_framework:./audio"

nodes:
  discovery:
    nodes_directory: "nodes"
    auto_discover: true

  global_overrides:
    all:
      runtime:
        isolated_process: true
        restart_policy: "on_failure"
        max_restarts: 5
        restart_delay: 2.0

  status_overrides:
    disabled_nodes: []
    muted_nodes: []

launch:
  parallel_launch: false
  wait_for_healthy: true
  ready_timeout: 30.0
EOF

    # Create environment template
    cat > "$project_name/.env.example" << 'EOF'
# Nevil v3.0 Environment Variables
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_ASSISTANT_ID=asst_your-assistant-id-here
LOG_LEVEL=INFO
TTS_VOICE=onyx
EOF

    # Create README
    cat > "$project_name/README.md" << EOF
# $project_name

A Nevil v3.0 robot project.

## Setup

1. Copy \`.env.example\` to \`.env\` and fill in your API keys
2. Install dependencies: \`pip install -r requirements.txt\`
3. Start the system: \`./nevil start\`

## Commands

- \`./nevil start\` - Start the robot system
- \`./nevil start --logscope\` - Start with monitoring dashboard
- \`./nevil stop\` - Stop the robot system
- \`./nevil status\` - Show system status
- \`./nevil logs\` - Show system logs
- \`./nevil monitor\` - Launch monitoring dashboard

## Configuration

Edit \`.nodes\` to configure which nodes to run and their settings.
Each node has its own \`.messages\` file defining its communication interface.
EOF

    # Copy nevil script
    cp "$0" "$project_name/nevil"
    chmod +x "$project_name/nevil"

    log_success "Project $project_name created successfully"
    log_info "Next steps:"
    echo "  1. cd $project_name"
    echo "  2. cp .env.example .env"
    echo "  3. Edit .env with your API keys"
    echo "  4. ./nevil start"
}
```

### 5.4 Enhanced Status Display (Phase 2)

The Phase 2 status command includes LogScope monitoring information:

```bash
# LogScope status (phase 2)
echo ""
echo "LogScope Dashboard:"
if [ -f "logscope.pid" ]; then
    local logscope_pid=$(cat logscope.pid)
    if kill -0 "$logscope_pid" 2>/dev/null; then
        log_success "LogScope is running (PID: $logscope_pid)"
    else
        log_warning "LogScope PID file exists but process not running"
        rm -f logscope.pid
    fi
else
    log_info "LogScope is not running"
    log_info "Use 'nevil monitor' to launch LogScope dashboard"
fi
```

### 5.5 Enhanced Help Text (Phase 2)

The Phase 2 help includes additional commands:

```bash
Commands:
    launch/start    Launch the Nevil system
    stop            Stop the Nevil system
    restart         Restart the Nevil system
    status          Show system status
    logs            Show system logs
    node            Node management commands
    -phase 2:
        monitor         Launch LogScope monitoring dashboard
        validate        Validate configuration
        init            Initialize new Nevil project

Examples:
    nevil launch                 # Start the system
    nevil start                 # Start the system dashboard
    nevil stop                  # Stop the system
    nevil status                # Show status
    nevil logs speech_recognition  # Show node logs
    -phase 2:
        nevil validate              # Validate configuration
        nevil init my_robot         # Create new project
        nevil start --logscope      # Start with LogScope 
        nevil monitor               # Launch LogScope 
```

### 5.6 Configuration Integration (Phase 2)

The CLI integrates with the `.nodes` configuration file to enable/disable LogScope monitoring:

```yaml
system:
  monitoring:
    logscope_enabled: true    # Auto-launch LogScope with system
    logscope_theme: "dark"    # UI theme preference
    max_entries: 10000        # Memory limit for LogScope
```

#### check_logscope_config() Function

```bash
# Check LogScope configuration
check_logscope_config() {
    # Check if LogScope is enabled in configuration
    if [ -f ".nodes" ]; then
        $PYTHON_CMD -c "
import yaml
import sys
try:
    with open('.nodes', 'r') as f:
        config = yaml.safe_load(f)

    monitoring = config.get('system', {}).get('monitoring', {})
    logscope_enabled = monitoring.get('logscope_enabled', False)

    if logscope_enabled:
        print('enabled')
    else:
        print('disabled')
except Exception:
    print('disabled')
" 2>/dev/null
    else
        echo "disabled"
    fi
}
```

## Conclusion

The Phase 2 CLI enhancements provide comprehensive project management, advanced monitoring capabilities, and seamless integration with the LogScope dashboard while maintaining the simplicity and reliability of the core Nevil v3.0 command-line interface.

# TEST: All Phase 2 CLI commands function correctly
# TEST: LogScope integration works across different platforms
# TEST: Project initialization creates proper structure and templates
# TEST: Configuration validation prevents invalid system starts

## 6. Advanced Process Management (Phase 2)

### 6.1 Enhanced Process Lifecycle Management

#### Monitoring Phase (Phase 2)
- **Heartbeat monitoring** with configurable timeouts
- **Resource usage tracking** (CPU, memory, file descriptors)
- **Health check execution** and status aggregation
- **Performance metrics collection** and analysis

#### Recovery Phase (Phase 2)
- **Failure detection** through multiple signals (heartbeat, process exit, health checks)
- **Restart policy evaluation** (always, on-failure, never)
- **Exponential backoff** for repeated failures
- **Dependency cascade handling** for related failures

### 6.2 Advanced Health Monitor Implementation (Phase 2)

```python
# nevil_framework/health_monitor.py

import time
import threading
import psutil
from typing import Dict, List, Any
from dataclasses import dataclass, field
from collections import defaultdict, deque

@dataclass
class NodeHealth:
    node_name: str
    status: str = "unknown"
    last_heartbeat: float = 0.0
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    thread_count: int = 0
    error_count: int = 0
    response_time: float = 0.0
    uptime: float = 0.0

@dataclass
class SystemHealth:
    overall_status: str = "unknown"
    node_count: int = 0
    healthy_nodes: int = 0
    unhealthy_nodes: int = 0
    total_cpu_usage: float = 0.0
    total_memory_usage: float = 0.0
    message_throughput: float = 0.0
    error_rate: float = 0.0
    issues: List[str] = field(default_factory=list)

class HealthMonitor:
    """
    Advanced health monitoring system for Phase 2.
    
    Monitors system and node health, providing alerts and metrics.
    """
    
    def __init__(self, message_bus):
        self.message_bus = message_bus
        self.node_health = {}  # node_name -> NodeHealth
        self.health_lock = threading.RLock()
        
        # Monitoring configuration
        self.heartbeat_timeout = 30.0  # seconds
        self.check_interval = 5.0      # seconds
        self.alert_thresholds = {
            'cpu_usage': 80.0,         # percent
            'memory_usage': 85.0,      # percent
            'error_rate': 0.1,         # 10%
            'response_time': 5000.0    # milliseconds
        }
        
        # Metrics collection
        self.metrics_history = defaultdict(lambda: deque(maxlen=100))
        self.message_counts = defaultdict(int)
        self.error_counts = defaultdict(int)
        
        # Control
        self.running = False
        self.monitor_thread = None
        
        # Subscribe to heartbeat messages
        self.message_bus.subscribe("system_monitor", "system_heartbeat", self._on_heartbeat)
    
    def start(self):
        """Start health monitoring"""
        if self.running:
            return
        
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def stop(self):
        """Stop health monitoring"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5.0)
    
    def _on_heartbeat(self, message):
        """Handle heartbeat messages from nodes"""
        try:
            data = message.data
            node_name = data.get('node_name')
            
            if not node_name:
                return
            
            with self.health_lock:
                if node_name not in self.node_health:
                    self.node_health[node_name] = NodeHealth(node_name=node_name)
                
                health = self.node_health[node_name]
                health.status = data.get('status', 'unknown')
                health.last_heartbeat = time.time()
                health.cpu_usage = data.get('cpu_usage', 0.0)
                health.memory_usage = data.get('memory_usage', 0.0)
                health.thread_count = data.get('thread_count', 0)
                health.error_count = data.get('error_count', 0)
                
                # Update metrics history
                self.metrics_history[f"{node_name}_cpu"].append(health.cpu_usage)
                self.metrics_history[f"{node_name}_memory"].append(health.memory_usage)
                
        except Exception as e:
            print(f"Error processing heartbeat: {e}")
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                self._check_node_health()
                self._collect_system_metrics()
                self._check_alerts()
                
                time.sleep(self.check_interval)
                
            except Exception as e:
                print(f"Error in health monitor loop: {e}")
                time.sleep(self.check_interval)
    
    def _check_node_health(self):
        """Check health of all nodes"""
        current_time = time.time()
        
        with self.health_lock:
            for node_name, health in self.node_health.items():
                # Check heartbeat timeout
                if current_time - health.last_heartbeat > self.heartbeat_timeout:
                    if health.status != "timeout":
                        health.status = "timeout"
                        print(f"Node {node_name} heartbeat timeout")
                
                # Update uptime
                if health.last_heartbeat > 0:
                    health.uptime = current_time - health.last_heartbeat
    
    def _collect_system_metrics(self):
        """Collect system-wide metrics"""
        try:
            # System CPU and memory
            system_cpu = psutil.cpu_percent()
            system_memory = psutil.virtual_memory().percent
            
            # Store in metrics history
            self.metrics_history['system_cpu'].append(system_cpu)
            self.metrics_history['system_memory'].append(system_memory)
            
        except Exception as e:
            print(f"Error collecting system metrics: {e}")
    
    def _check_alerts(self):
        """Check for alert conditions"""
        alerts = []
        
        with self.health_lock:
            # Check node-specific alerts
            for node_name, health in self.node_health.items():
                if health.cpu_usage > self.alert_thresholds['cpu_usage']:
                    alerts.append(f"High CPU usage on {node_name}: {health.cpu_usage:.1f}%")
                
                if health.memory_usage > self.alert_thresholds['memory_usage']:
                    alerts.append(f"High memory usage on {node_name}: {health.memory_usage:.1f}%")
                
                if health.status in ['error', 'timeout']:
                    alerts.append(f"Node {node_name} is unhealthy: {health.status}")
        
        # Send alerts if any
        for alert in alerts:
            self._send_alert(alert)
    
    def _send_alert(self, message: str):
        """Send alert message"""
        print(f"ALERT: {message}")
        
        # Could be extended to send to external monitoring systems
        alert_data = {
            "type": "health_alert",
            "message": message,
            "timestamp": time.time()
        }
        
        self.message_bus.publish({
            "topic": "system_alerts",
            "data": alert_data,
            "timestamp": time.time(),
            "source_node": "health_monitor",
            "message_id": f"alert_{time.time()}"
        })
    
    def get_node_health(self, node_name: str) -> NodeHealth:
        """Get health status for a specific node"""
        with self.health_lock:
            return self.node_health.get(node_name, NodeHealth(node_name=node_name))
    
    def get_system_health(self) -> SystemHealth:
        """Get overall system health status"""
        with self.health_lock:
            health = SystemHealth()
            
            health.node_count = len(self.node_health)
            health.healthy_nodes = sum(1 for h in self.node_health.values() if h.status == "running")
            health.unhealthy_nodes = health.node_count - health.healthy_nodes
            
            if health.node_count > 0:
                health.total_cpu_usage = sum(h.cpu_usage for h in self.node_health.values()) / health.node_count
                health.total_memory_usage = sum(h.memory_usage for h in self.node_health.values()) / health.node_count
            
            # Determine overall status
            if health.unhealthy_nodes == 0:
                health.overall_status = "healthy"
            elif health.unhealthy_nodes < health.node_count:
                health.overall_status = "degraded"
            else:
                health.overall_status = "unhealthy"
            
            # Collect issues
            issues = []
            for node_name, node_health in self.node_health.items():
                if node_health.status != "running":
                    issues.append(f"Node {node_name} is {node_health.status}")
                if node_health.cpu_usage > self.alert_thresholds['cpu_usage']:
                    issues.append(f"High CPU on {node_name}: {node_health.cpu_usage:.1f}%")
                if node_health.memory_usage > self.alert_thresholds['memory_usage']:
                    issues.append(f"High memory on {node_name}: {node_health.memory_usage:.1f}%")
            
            health.issues = issues
            return health
    
    def get_metrics_summary(self) -> Dict[str, Any]:
        """Get summary of collected metrics"""
        with self.health_lock:
            summary = {}
            
            for metric_name, values in self.metrics_history.items():
                if values:
                    summary[metric_name] = {
                        'current': values[-1],
                        'average': sum(values) / len(values),
                        'min': min(values),
                        'max': max(values),
                        'count': len(values)
                    }
            
            return summary

# TEST: Health monitor detects node failures correctly
# TEST: Metrics are collected and stored properly
# TEST: Alerts are triggered at correct thresholds
# TEST: System health status is calculated accurately
```

## 7. Advanced Process Recovery (Phase 2)

### 7.1 Intelligent Restart Policies

**Restart Policy Types:**
- **always**: Restart regardless of exit code
- **on-failure**: Restart only on non-zero exit codes
- **never**: No automatic restart
- **unless-stopped**: Restart unless explicitly stopped

**Exponential Backoff Strategy:**
```python
def calculate_restart_delay(restart_count: int, base_delay: float = 2.0) -> float:
    """Calculate restart delay with exponential backoff"""
    max_delay = 300.0  # 5 minutes maximum
    delay = base_delay * (2 ** min(restart_count, 8))
    return min(delay, max_delay)
```

### 7.2 Dependency Cascade Handling

**Cascade Recovery Strategy:**
- **Upstream Failure**: When a dependency fails, dependent nodes are gracefully paused
- **Downstream Impact**: When a node fails, its dependents are notified and can adapt
- **Recovery Coordination**: When dependencies recover, dependent nodes are restarted in order
- **Partial Recovery**: System continues operating with reduced functionality during partial failures

### 7.3 Advanced Failure Detection

**Multi-Signal Detection:**
- **Process Exit**: Immediate detection of process termination
- **Heartbeat Timeout**: Detection of unresponsive but running processes
- **Health Check Failure**: Application-level health validation
- **Resource Exhaustion**: Detection of resource limit violations
- **Performance Degradation**: Detection of performance anomalies

## Conclusion

The Phase 2 launch system enhancements provide enterprise-grade process management, health monitoring, and recovery capabilities while maintaining the simplicity and reliability of the core Nevil v3.0 launch system.

# TEST: Advanced health monitoring provides comprehensive system visibility
# TEST: Intelligent restart policies prevent cascading failures
# TEST: Dependency cascade handling maintains system stability
# TEST: Multi-signal failure detection catches all failure modes
# TEST: Performance metrics enable proactive system management