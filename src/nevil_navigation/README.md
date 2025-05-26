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