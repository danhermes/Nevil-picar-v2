# Nevil Interfaces

Custom message, service, and action definitions for Nevil-picar v2.0.

## Description

The `nevil_interfaces` package provides custom ROS2 interfaces for the Nevil-picar robot. It includes:

- Custom message definitions
- Custom service definitions
- Custom action definitions

These interfaces are used by the other Nevil packages to communicate with each other.

## Messages

### SystemStatus.msg

Status information about the overall system, including:
- Current mode
- Battery level
- Error states
- System health

### BehaviorStatus.msg

Status information about the robot's behaviors, including:
- Current behavior
- Behavior parameters
- Execution status

## Services

### CheckObstacle.srv

Service to check if there is an obstacle in a specific direction:
- Request: Direction to check
- Response: Obstacle presence and distance

## Actions

### NavigateToPoint.action

Action to navigate to a specific point:
- Goal: Target position and parameters
- Feedback: Current position, distance to goal
- Result: Success/failure and final position

### PerformBehavior.action

Action to perform a specific behavior:
- Goal: Behavior type and parameters
- Feedback: Execution progress
- Result: Success/failure and outcome

## Usage

```cpp
// Include the interfaces in your C++ code
#include "nevil_interfaces/msg/system_status.hpp"
#include "nevil_interfaces/srv/check_obstacle.hpp"
#include "nevil_interfaces/action/navigate_to_point.hpp"
```

```python
# Import the interfaces in your Python code
from nevil_interfaces.msg import SystemStatus
from nevil_interfaces.srv import CheckObstacle
from nevil_interfaces.action import NavigateToPoint