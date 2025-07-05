# Nevil Navigation

Navigation and movement control for Nevil-picar v2.0.

## Description

The `nevil_navigation` package provides navigation and movement control capabilities for the Nevil-picar robot. It handles:

- Motion control (motors and servos)
- Path planning and execution
- Obstacle avoidance integration
- Navigation behaviors

## Nodes

### Motion Control Node

The Motion Control Node (`motion_control_node`) is responsible for:

- Translating high-level commands to motor/servo controls
- Implementing safety limits and checks
- Providing feedback on motion execution
- Managing expressive behaviors

### Navigation Node

The Navigation Node (`navigation_node`) is responsible for:

- Path planning and execution
- Integrating sensor data for navigation
- Implementing navigation behaviors
- Coordinating with obstacle avoidance

### Navigation Monitor

The Navigation Monitor (`navigation_monitor`) provides real-time monitoring and diagnostics:

- **Real-time Topic Monitoring**: Tracks all navigation-related topics (`/cmd_vel`, `/goal_pose`, `/system_mode`, `/nevil/action_command`, `/planned_path`)
- **Critical Error Detection**: Monitors ROS2 logs for crashes, segfaults, and fatal errors with immediate alerting
- **Node Health Monitoring**: Tracks critical navigation nodes and alerts when missing or unresponsive
- **System Resource Monitoring**: Watches CPU, memory usage, and zombie processes
- **Multi-level Alerting**: Color-coded alerts with severity levels (Fatal/Error/Warning)
- **Rich Terminal Interface**: Enhanced display with multiple panels when `rich` library is available
- **Logging Support**: Optional file logging for post-analysis and debugging

#### Quick Start
```bash
# Start basic monitoring
./src/nevil_navigation/scripts/start_monitor.sh

# With logging and custom refresh rate
./src/nevil_navigation/scripts/start_monitor.sh --log-file /tmp/nav_monitor.log --refresh-rate 50
```

#### Key Features
- **Fatal Error Detection**: Immediate alerts for system crashes and segfaults
- **Topic Timeout Detection**: Alerts when critical topics stop publishing (>30s = fatal)
- **Node Health Checks**: Monitors critical nodes every 5 seconds
- **System Resource Alerts**: High memory (>90%) and CPU (>95%) warnings
- **Interactive Controls**: Reset counters (r), pause/resume (p), clean shutdown (Ctrl+C)

For detailed documentation, see [`scripts/README_navigation_monitor.md`](scripts/README_navigation_monitor.md).

## Usage

```bash
# Launch the navigation system
ros2 launch nevil_navigation nevil_navigation.launch.py
```

## Dependencies

- ROS2 Humble
- nevil_interfaces
- nevil_core
- nevil_perception (for sensor data)