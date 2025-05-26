# Nevil Core

Core functionality and node management for Nevil-picar v2.0.

## Description

The `nevil_core` package provides the central system management and coordination for the Nevil-picar robot. It handles:

- System state management
- Node lifecycle coordination
- Mode switching
- Parameter management
- System diagnostics and monitoring

## Nodes

### System Manager Node

The System Manager Node (`system_manager_node`) is responsible for:

- Coordinating the startup and shutdown sequences of other nodes
- Managing system modes (e.g., autonomous, manual, learning)
- Monitoring system health and resource usage
- Providing a central interface for system-wide commands

## Usage

```bash
# Launch the core system
ros2 launch nevil_core nevil_core.launch.py
```

## Dependencies

- ROS2 Humble
- nevil_interfaces