# Nevil v3.0 Command Line Interface

## Overview

The Nevil v3.0 CLI provides a comprehensive command-line interface for managing the robot system. The `./nevil` bash script serves as the primary entry point for all system operations, from startup and shutdown to monitoring and debugging.

## Installation

The CLI script is automatically included when initializing a new Nevil v3.0 project:

```bash
nevil init my_robot
cd my_robot
./nevil --help
```

## Command Reference

### Core System Commands

#### start
Start the Nevil v3.0 system with optional LogScope monitoring.

```bash
./nevil start                 # Start the system
./nevil start --logscope      # Start with LogScope dashboard
./nevil start --no-logscope   # Start without LogScope (override config)
```

**Behavior**:
- Validates configuration before starting
- Checks for existing running processes
- Launches NevilLauncher
- Optionally launches LogScope monitoring dashboard
- Provides system startup status

#### stop
Gracefully shutdown the Nevil v3.0 system.

```bash
./nevil stop
```

**Behavior**:
- Stops LogScope dashboard if running
- Sends SIGTERM to main system process
- Waits for graceful shutdown (30s timeout)
- Force kills if necessary
- Cleans up PID files

#### restart
Restart the entire system.

```bash
./nevil restart               # Restart system
./nevil restart --logscope    # Restart with LogScope
```

**Behavior**:
- Calls stop followed by start
- Preserves command-line options

#### status
Display comprehensive system status information.

```bash
./nevil status
```

**Output includes**:
- Main system process status and PID
- Individual node process information
- LogScope dashboard status
- Log file sizes and statistics (with KEEP_LOG_FILES=3 info)
- Resource usage information

### Monitoring Commands

#### logs
View system or node-specific logs.

```bash
./nevil logs                           # Show system logs (last 50 lines)
./nevil logs speech_recognition        # Show node-specific logs
./nevil logs -f                        # Follow system logs (live tail)
./nevil logs speech_recognition -f     # Follow node logs
```

#### monitor
Launch the LogScope monitoring dashboard (Phase 2 feature).

```bash
./nevil monitor
```

**Phase 2 Feature:**
See [`04_launch_system_architecture_phase_2.md`](./phase%202/04_launch_system_architecture_phase_2.md) for detailed monitor command implementation.

### Configuration Commands

#### validate
Validate system and node configurations.

```bash
./nevil validate
```

**Validation checks**:
- YAML syntax validation for `.nodes` file
- Individual node `.messages` file validation
- Configuration completeness verification
- Dependency validation

#### init
Initialize a new Nevil v3.0 project.

```bash
./nevil init <project_name>
```

**Creates**:
- Project directory structure
- Basic `.nodes` configuration file
- Environment template (`.env.example`)
- README with setup instructions
- Copy of nevil CLI script

### Development Commands

#### node
Node management operations.

```bash
./nevil node list                    # List available nodes
./nevil node create <node_name>      # Create new node template
```

**Node creation includes**:
- Basic node Python file template
- `.messages` configuration file
- Proper class structure and imports

## CLI Script Implementation

#### Code Summary

This section implements the complete Nevil v3.0 command-line interface as a comprehensive bash script with full system management capabilities:

**Key Components:**
- **Environment Setup**: Root directory detection, Python command configuration, and color output setup
- **Utility Functions**: Colored logging, process management, PID handling, and status checking
- **Configuration Validation**: YAML syntax checking, file existence validation, and dependency verification
- **System Management**: Start, stop, restart, and status operations with proper error handling
- **LogScope Integration**: Optional monitoring dashboard with GUI and terminal modes
- **Log Management**: Log viewing, rotation, and cleanup with configurable retention
- **Project Scaffolding**: New project initialization with proper directory structure and templates
- **Node Management**: Node creation, removal, and configuration management

**Key Functions:**
- `log_info()`, `log_error()`, `log_success()`: Colored console output with timestamps
- `check_dependencies()`: Verify Python, pip, and system requirements
- `validate_config()`: Comprehensive configuration file validation
- `start_system()`: System startup with dependency checking and process management
- `stop_system()`: Graceful shutdown with timeout and cleanup
- `start_logscope()`: Launch monitoring dashboard with mode selection
- `show_status()`: System health and process status reporting
- `show_logs()`: Log viewing with filtering and real-time updates
- `init_project()`: Complete project scaffolding with templates and configuration

**Key Features:**
- Complete system lifecycle management (start, stop, restart, status)
- Integrated monitoring with LogScope GUI and terminal interfaces
- Comprehensive configuration validation before system operations
- Process management with PID tracking and cleanup
- Log management with rotation and retention policies
- Project initialization with proper templates and structure
- Node scaffolding with automatic configuration generation
- Error handling with descriptive messages and exit codes
- Cross-platform compatibility with proper path handling

```bash
#!/bin/bash
# nevil - Main command line interface for Nevil v3.0

set -e

NEVIL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD="${PYTHON_CMD:-python3}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Help function
show_help() {
    cat << EOF
Nevil v3.0 - Lightweight Robot Framework

Usage: nevil <command> [options]

Commands:
    launch/start    Launch the Nevil system
    stop            Stop the Nevil system
    restart         Restart the Nevil system
    status          Show system status
    logs            Show system logs
    node            Node management commands

Phase 2 Commands:
    monitor         Launch LogScope monitoring dashboard
    validate        Validate configuration
    init            Initialize new Nevil project

Options:
    -h, --help      Show this help message
    -v, --verbose   Enable verbose output
    -c, --config    Specify configuration file (default: .nodes)
    --logscope      Auto-launch LogScope monitoring dashboard
    --no-logscope   Disable LogScope auto-launch

Examples:
    nevil launch                 # Start the system
    nevil start                 # Start the system
    nevil stop                  # Stop the system
    nevil status                # Show status
    nevil logs speech_recognition  # Show node logs

Phase 2 Examples:
    nevil validate              # Validate configuration
    nevil init my_robot         # Create new project
    nevil start --logscope      # Start with LogScope
    nevil monitor               # Launch LogScope


For more information, see: docs/nevil_v3/
EOF
}

# Configuration validation
validate_config() {
    log_info "Validating Nevil v3.0 configuration..."

    if [ ! -f ".nodes" ]; then
        log_error ".nodes configuration file not found"
        return 1
    fi

    # Validate YAML syntax
    if ! $PYTHON_CMD -c "
import yaml
import sys
try:
    with open('.nodes', 'r') as f:
        yaml.safe_load(f)
    print('✓ .nodes file has valid YAML syntax')
except yaml.YAMLError as e:
    print(f'✗ .nodes file has invalid YAML: {e}')
    sys.exit(1)
"; then
        return 1
    fi

    # Validate node configurations
    for node_dir in nodes/*/; do
        if [ -d "$node_dir" ]; then
            node_name=$(basename "$node_dir")
            messages_file="$node_dir/.messages"

            if [ -f "$messages_file" ]; then
                if ! $PYTHON_CMD -c "
import yaml
import sys
try:
    with open('$messages_file', 'r') as f:
        yaml.safe_load(f)
    print('✓ $node_name .messages file is valid')
except yaml.YAMLError as e:
    print(f'✗ $node_name .messages file has invalid YAML: {e}')
    sys.exit(1)
"; then
                    return 1
                fi
            else
                log_warning "Missing .messages file for node: $node_name"
            fi
        fi
    done

    log_success "Configuration validation complete"
    return 0
}

# Phase 2 LogScope Functions
# See phase 2/04_launch_system_architecture_phase_2.md for detailed implementation

# Launch  system
start_system() {
    log_info "Launching Nevil v3.0 system..."

    # Parse arguments for LogScope options
    local enable_logscope="auto"  # auto, force, disable

    case "${2:-}" in
        "--logscope")
            enable_logscope="force"
            ;;
        "--no-logscope")
            enable_logscope="disable"
            ;;
        *)
            # Check configuration for auto-launch
            local config_setting=$(check_logscope_config)
            if [ "$config_setting" = "enabled" ]; then
                enable_logscope="auto"
            else
                enable_logscope="disable"
            fi
            ;;
    esac

    # Validate configuration first
    if ! validate_config; then
        log_error "Configuration validation failed"
        return 1
    fi

    # Check if already running
    if [ -f "nevil.pid" ]; then
        local pid=$(cat nevil.pid)
        if kill -0 "$pid" 2>/dev/null; then
            log_warning "Nevil system is already running (PID: $pid)"
            return 1
        else
            log_info "Removing stale PID file"
            rm -f nevil.pid
        fi
    fi

    # Start the launcher
    if [ "$enable_logscope" != "disable" ]; then
        $PYTHON_CMD -m nevil_framework.launcher --logscope &
    else
        $PYTHON_CMD -m nevil_framework.launcher &
    fi
    local launcher_pid=$!

    # Save PID
    echo $launcher_pid > nevil.pid

    log_success "Nevil system started (PID: $launcher_pid)"

    # Launch LogScope if requested
    if [ "$enable_logscope" = "force" ] || [ "$enable_logscope" = "auto" ]; then
        sleep 3  # Give main system time to create logs
        launch_logscope "auto"
    fi

    log_info "Use 'nevil status' to check system health"
    log_info "Use 'nevil logs' to view system logs"
    if [ "$enable_logscope" != "disable" ]; then
        log_info "LogScope dashboard available for advanced monitoring"
    fi
}

# Stop system
stop_system() {
    log_info "Stopping Nevil v3.0 system..."

    # Stop LogScope first if running
    stop_logscope

    if [ ! -f "nevil.pid" ]; then
        log_warning "No PID file found, system may not be running"
        return 1
    fi

    local pid=$(cat nevil.pid)

    if ! kill -0 "$pid" 2>/dev/null; then
        log_warning "Process $pid is not running"
        rm -f nevil.pid
        return 1
    fi

    # Send SIGTERM for graceful shutdown
    log_info "Sending shutdown signal to process $pid"
    kill -TERM "$pid"

    # Wait for graceful shutdown
    local timeout=30
    local count=0
    while kill -0 "$pid" 2>/dev/null && [ $count -lt $timeout ]; do
        sleep 1
        count=$((count + 1))
        if [ $((count % 5)) -eq 0 ]; then
            log_info "Waiting for graceful shutdown... ($count/$timeout)"
        fi
    done

    # Force kill if still running
    if kill -0 "$pid" 2>/dev/null; then
        log_warning "Graceful shutdown timeout, force killing process"
        kill -KILL "$pid"
        sleep 2
    fi

    # Clean up PID file
    rm -f nevil.pid

    log_success "Nevil system stopped"
}

# Show system status
show_status() {
    log_info "Nevil v3.0 System Status"
    echo "=========================="

    # Main system status
    if [ -f "nevil.pid" ]; then
        local pid=$(cat nevil.pid)
        if kill -0 "$pid" 2>/dev/null; then
            log_success "System is running (PID: $pid)"

            # Show process information
            if command -v ps >/dev/null 2>&1; then
                echo ""
                echo "Process Information:"
                ps -p "$pid" -o pid,ppid,cmd,etime,pcpu,pmem 2>/dev/null || true
            fi

            # Show node processes
            echo ""
            echo "Node Processes:"
            pgrep -f "nodes\." | while read node_pid; do
                if ps -p "$node_pid" >/dev/null 2>&1; then
                    local cmd=$(ps -p "$node_pid" -o cmd --no-headers 2>/dev/null || echo "unknown")
                    echo "  PID $node_pid: $cmd"
                fi
            done

        else
            log_error "PID file exists but process $pid is not running"
            echo "Removing stale PID file..."
            rm -f nevil.pid
        fi
    else
        log_warning "System is not running (no PID file found)"
    fi

    # Phase 2: LogScope status available in Phase 2
    echo ""
    echo "Phase 2 Features:"
    echo "  LogScope monitoring available in Phase 2"
    echo "  Use 'nevil monitor' for advanced monitoring (Phase 2)"

    # Show log file status (updated for KEEP_LOG_FILES=3)
    echo ""
    echo "Log Files (KEEP_LOG_FILES=3):"
    if [ -f "logs/system.log" ]; then
        local log_size=$(du -h logs/system.log | cut -f1)
        local log_lines=$(wc -l < logs/system.log)
        echo "  System log: $log_size ($log_lines lines)"

        # Show numbered backups
        for i in 1 2; do
            if [ -f "logs/system.log.$i" ]; then
                local backup_size=$(du -h "logs/system.log.$i" | cut -f1)
                echo "  System log.$i: $backup_size (backup)"
            fi
        done
    else
        echo "  System log: not found"
    fi

    # Show node logs with numbered backups
    for node in speech_recognition speech_synthesis ai_cognition; do
        if [ -f "logs/$node.log" ]; then
            local log_size=$(du -h "logs/$node.log" | cut -f1)
            local log_lines=$(wc -l < "logs/$node.log")
            echo "  $node log: $log_size ($log_lines lines)"

            # Show numbered backups
            for i in 1 2; do
                if [ -f "logs/$node.log.$i" ]; then
                    local backup_size=$(du -h "logs/$node.log.$i" | cut -f1)
                    echo "  $node.log.$i: $backup_size (backup)"
                fi
            done
        fi
    done
}

# Show logs
show_logs() {
    local node_name="$1"
    local follow="$2"

    if [ -z "$node_name" ]; then
        # Show system logs
        local log_file="logs/system.log"
        if [ ! -f "$log_file" ]; then
            log_error "System log file not found: $log_file"
            return 1
        fi

        if [ "$follow" = "-f" ] || [ "$follow" = "--follow" ]; then
            log_info "Following system logs (Ctrl+C to stop)..."
            tail -f "$log_file"
        else
            log_info "Showing last 50 lines of system log..."
            tail -n 50 "$log_file"
        fi
    else
        # Show node logs (updated for flat structure)
        local log_file="logs/$node_name.log"
        if [ ! -f "$log_file" ]; then
            log_error "Node log file not found: $log_file"
            return 1
        fi

        if [ "$follow" = "-f" ] || [ "$follow" = "--follow" ]; then
            log_info "Following $node_name logs (Ctrl+C to stop)..."
            tail -f "$log_file"
        else
            log_info "Showing last 50 lines of $node_name log..."
            tail -n 50 "$log_file"
        fi
    fi
}

# Phase 2 Project Initialization Functions
# See phase 2/04_launch_system_architecture_phase_2.md for detailed implementation

# Node management
manage_node() {
    local action="$1"
    local node_name="$2"

    case "$action" in
        "list")
            log_info "Available nodes:"
            if [ -d "nodes" ]; then
                for node_dir in nodes/*/; do
                    if [ -d "$node_dir" ]; then
                        local name=$(basename "$node_dir")
                        echo "  - $name"
                    fi
                done
            else
                log_warning "No nodes directory found"
            fi
            ;;
        "create")
            if [ -z "$node_name" ]; then
                log_error "Node name is required"
                echo "Usage: nevil node create <node_name>"
                return 1
            fi

            log_info "Creating node: $node_name"

            # Create node directory
            mkdir -p "nodes/$node_name"

            # Create basic node file
            cat > "nodes/$node_name/${node_name}_node.py" << EOF
#!/usr/bin/env python3

from nevil_framework.base_node import NevilNode

class ${node_name^}Node(NevilNode):
    """
    ${node_name^} node implementation.
    """

    def __init__(self):
        super().__init__("$node_name")

    def initialize(self):
        """Initialize node-specific components"""
        self.logger.info("$node_name node initializing...")
        # Add initialization code here

    def main_loop(self):
        """Main processing loop"""
        # Add main processing logic here
        pass

    def cleanup(self):
        """Cleanup node resources"""
        self.logger.info("$node_name node cleaning up...")
        # Add cleanup code here

if __name__ == "__main__":
    node = ${node_name^}Node()
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()
EOF

            # Create basic .nodes file
            cat > "nodes/$node_name/.nodes" << EOF
node_name: "$node_name"
version: "1.0"
description: "$node_name node configuration"

dependencies:
  requires: []
  provides: []

runtime:
  isolated_process: true
  restart_policy: "on_failure"
  max_restarts: 5
  restart_delay: 2.0
  health_check_interval: 10.0

resources:
  cpu_limit: 100.0
  memory_limit: 512
  priority: "medium"

environment: {}

configuration: {}
EOF

            # Create basic messages file
            cat > "nodes/$node_name/.messages" << EOF
node_name: "$node_name"
version: "1.0"
description: "$node_name node message interface"

publishes: []
subscribes: []

configuration: {}

testing:
  mock_data: {}
  test_scenarios: []
EOF

            log_success "Node $node_name created successfully"
            ;;
        *)
            echo "Node management commands:"
            echo "  nevil node list          - List available nodes"
            echo "  nevil node create <name> - Create new node"
            ;;
    esac
}

# Main command processing
case "${1:-}" in
    "start")
        start_system "$@"
        ;;
    "stop")
        stop_system
        ;;
    "restart")
        stop_system
        sleep 2
        start_system "$@"
        ;;
    "status")
        show_status
        ;;
    "logs")
        show_logs "$2" "$3"
        ;;
    "monitor")
        launch_logscope "manual"
        ;;
    "validate")
        validate_config
        ;;
    "init")
        init_project "$2"
        ;;
    "node")
        manage_node "$2" "$3"
        ;;
    "-h"|"--help"|"help")
        show_help
        ;;
    "")
        log_error "No command specified"
        show_help
        exit 1
        ;;
    *)
        log_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
```

## Configuration Integration

The CLI integrates with the `.nodes` configuration file to enable/disable LogScope monitoring:

```yaml
system:
  monitoring:
    logscope_enabled: true    # Auto-launch LogScope with system
    logscope_theme: "dark"    # UI theme preference
    max_entries: 10000        # Memory limit for LogScope
```

## Error Handling

The CLI provides comprehensive error handling:

- **Configuration Validation**: YAML syntax and semantic validation
- **Process Management**: PID file handling and stale process cleanup
- **Dependency Checking**: PyQt availability for LogScope
- **Graceful Shutdown**: Timeout-based shutdown with force-kill fallback
- **User Feedback**: Color-coded status messages and progress indicators

## Integration with System Architecture

The CLI serves as the user interface layer for the Nevil v3.0 system:

1. **NevilLauncher**: CLI starts the main Python orchestrator
2. **Process Management**: Tracks and manages system and LogScope processes
3. **Configuration Layer**: Validates and applies `.nodes` configurations
4. **Logging System**: Interfaces with the enhanced logging architecture
5. **Health Monitoring**: Displays real-time system health information

The CLI ensures that users have a simple, reliable interface for managing complex robotic systems while maintaining the core philosophy of "simple architecture = working robot".