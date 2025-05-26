# Nevil Navigation API

## Overview

The Nevil Navigation API is a ROS2 adaptation of the original `action_helper.py` API from Nevil v1.0, designed to work with the ROS2 and PREEMPT-RT architecture in Nevil-picar v2.0. This API provides a seamless transition path for existing code while leveraging the new architecture's capabilities.

## Key Features

- **API Compatibility**: Maintains the same function signatures as the original `action_helper.py` where possible
- **ROS2 Integration**: Adapts the implementation to use ROS2 actions, services, and topics
- **Real-Time Support**: Integrates with the real-time components from the `nevil_realtime` package
- **Compatibility Layer**: Allows existing code from v1.0 to work with minimal changes
- **Async Support**: Provides both synchronous and asynchronous API options
- **Error Handling**: Robust error handling and recovery mechanisms
- **Thread Safety**: Ensures thread-safe operation in multi-threaded environments

## Architecture

The API is organized into several modules:

- **core.py**: The main API implementation with synchronous functions
- **async_api.py**: Asynchronous version of the API using `async`/`await`
- **compat.py**: Compatibility layer for v1.0 code
- **rt_integration.py**: Utilities for real-time integration
- **examples/**: Example scripts demonstrating API usage

## Installation

The API is part of the `nevil_navigation` package. No additional installation steps are required beyond the standard ROS2 workspace setup.

## Basic Usage

### Synchronous API

```python
import rclpy
from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI

# Initialize ROS2
rclpy.init()

# Create a node
node = rclpy.create_node('my_node')

# Create the API instance
api = NevilNavigationAPI(node)

# Use the API
api.move_forward_this_way(30, 0.5)  # Move forward 30cm at 50% speed
api.turn_left()                      # Turn left
api.stop()                           # Stop

# Clean up
api.shutdown()
node.destroy_node()
rclpy.shutdown()
```

### Asynchronous API

```python
import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from nevil_navigation.nevil_navigation_api.async_api import AsyncNevilNavigationAPI

async def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create a node
    node = rclpy.create_node('my_async_node')
    
    # Set up the executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create the API instance
    api = AsyncNevilNavigationAPI(node)
    
    # Use the API
    await api.move_forward_this_way(30, 0.5)  # Move forward 30cm at 50% speed
    await api.turn_left()                      # Turn left
    await api.stop()                           # Stop
    
    # Clean up
    api.shutdown()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

# Run the async main function
asyncio.run(main())
```

### Compatibility Layer for v1.0 Code

```python
from nevil_navigation.nevil_navigation_api.compat import NevilV1Adapter
from nevil_navigation.nevil_navigation_api.compat import move_forward_this_way, turn_left, stop

# Create the adapter
car = NevilV1Adapter()

# Use the adapter directly
car.forward(50)  # Move forward at 50% speed
car.stop()       # Stop

# Use the compatibility functions
move_forward_this_way(car, 30, 50)  # Move forward 30cm at 50% speed
turn_left(car)                      # Turn left
stop(car)                           # Stop
```

## API Reference

### Core Movement Functions

#### `move_forward_this_way(distance_cm, speed=None, check_obstacles=True)`

Move forward a specific distance at given speed.

- **Parameters**:
  - `distance_cm`: Distance to move in centimeters
  - `speed`: Speed to move at (0.0 to 1.0)
  - `check_obstacles`: Whether to check for obstacles during movement
- **Returns**: `True` if movement completed successfully, `False` otherwise

#### `move_backward_this_way(distance_cm, speed=None)`

Move backward a specific distance at given speed.

- **Parameters**:
  - `distance_cm`: Distance to move in centimeters
  - `speed`: Speed to move at (0.0 to 1.0)
- **Returns**: `True` if movement completed successfully, `False` otherwise

#### `turn_left()`

Turn the robot left.

- **Returns**: `True` if turn completed successfully, `False` otherwise

#### `turn_right()`

Turn the robot right.

- **Returns**: `True` if turn completed successfully, `False` otherwise

#### `stop()`

Stop the robot.

- **Returns**: `True` if stop command was sent successfully

#### `set_dir_servo_angle(angle)`

Set the direction servo angle.

- **Parameters**:
  - `angle`: Angle in degrees
- **Returns**: `True` if command was sent successfully

#### `set_cam_pan_angle(angle)`

Set the camera pan angle.

- **Parameters**:
  - `angle`: Angle in degrees
- **Returns**: `True` if command was sent successfully

#### `set_cam_tilt_angle(angle)`

Set the camera tilt angle.

- **Parameters**:
  - `angle`: Angle in degrees
- **Returns**: `True` if command was sent successfully

### Behavior Functions

#### `perform_behavior(behavior_name, duration=5.0, **params)`

Perform a predefined behavior.

- **Parameters**:
  - `behavior_name`: Name of the behavior to perform
  - `duration`: Maximum duration of the behavior in seconds
  - `**params`: Additional parameters for the behavior
- **Returns**: `True` if behavior completed successfully, `False` otherwise

#### Predefined Behaviors

The following behaviors are available as convenience methods:

- `wave_hands()`
- `resist()`
- `act_cute()`
- `rub_hands()`
- `think()`
- `keep_think()`
- `shake_head()`
- `nod()`
- `depressed()`
- `twist_body()`
- `celebrate()`
- `honk()`
- `start_engine()`

### Sensor Functions

#### `get_distance()`

Get the current distance from the ultrasonic sensor.

- **Returns**: Distance in meters

#### `check_obstacle(direction=0.0, max_distance=1.0)`

Check for obstacles in the specified direction.

- **Parameters**:
  - `direction`: Direction to check (in radians, 0 = forward)
  - `max_distance`: Maximum distance to check (in meters)
- **Returns**: A tuple of (obstacle_detected, distance, direction)

### Real-Time Integration

The `rt_integration` module provides utilities for working with real-time components:

#### `RTContext`

Context manager for real-time execution contexts.

```python
from nevil_navigation.nevil_navigation_api.rt_integration import RTContext

# Create a real-time context
rt_context = RTContext(node, priority=90)

# Use the context manager
with rt_context:
    # This code runs with real-time priority
    perform_critical_operation()

# Run a function in a real-time context
result = rt_context.run_in_rt_context(my_function, arg1, arg2)

# Create a real-time thread
thread = rt_context.create_rt_thread(target=my_function, args=(arg1, arg2))
thread.start()
```

#### `RTNode`

Extended ROS2 Node with real-time features.

```python
from nevil_navigation.nevil_navigation_api.rt_integration import RTNode

# Create a real-time node
node = RTNode('my_rt_node', priority=90)

# Create a real-time timer
node.create_rt_timer(0.1, my_callback)

# Create a real-time subscription
node.create_rt_subscription(
    msg_type,
    topic,
    my_callback,
    qos_profile
)
```

## Differences from v1.0

### API Changes

| v1.0 | v2.0 | Notes |
|------|------|-------|
| `move_forward_this_way(car, distance_cm, speed)` | `api.move_forward_this_way(distance_cm, speed)` | Object-oriented API |
| `move_backward_this_way(car, distance_cm, speed)` | `api.move_backward_this_way(distance_cm, speed)` | Object-oriented API |
| `turn_left(car)` | `api.turn_left()` | Object-oriented API |
| `turn_right(car)` | `api.turn_right()` | Object-oriented API |
| `stop(car)` | `api.stop()` | Object-oriented API |
| `car.forward(speed)` | `api.move_forward_this_way(10, speed/100.0)` | Speed scale changed from 0-100 to 0-1 |
| `car.backward(speed)` | `api.move_backward_this_way(10, speed/100.0)` | Speed scale changed from 0-100 to 0-1 |
| `car.stop()` | `api.stop()` | No change in behavior |
| `car.get_distance()` | `api.get_distance() * 100.0` | Returns meters instead of centimeters |
| `car.set_dir_servo_angle(angle)` | `api.set_dir_servo_angle(angle)` | No change in behavior |
| `car.set_cam_pan_angle(angle)` | `api.set_cam_pan_angle(angle)` | No change in behavior |
| `car.set_cam_tilt_angle(angle)` | `api.set_cam_tilt_angle(angle)` | No change in behavior |
| `car.set_motor_speed(motor, speed)` | N/A | Direct motor control not available in v2.0 |
| `car.reset()` | N/A | Use individual reset functions |

### Architecture Changes

- **ROS2 Integration**: v2.0 uses ROS2 actions, services, and topics for communication
- **Real-Time Support**: v2.0 integrates with PREEMPT-RT for deterministic performance
- **Multi-Threading**: v2.0 uses a multi-threaded executor for parallel processing
- **Error Handling**: v2.0 has more robust error handling and recovery mechanisms
- **Async Support**: v2.0 provides asynchronous API options using `async`/`await`

## Migration Guide

### Step 1: Update Imports

Replace imports from the original `action_helper.py` with imports from the new API:

```python
# v1.0
from helpers.action_helper import move_forward_this_way, turn_left, stop

# v2.0
from nevil_navigation.nevil_navigation_api.compat import move_forward_this_way, turn_left, stop
```

### Step 2: Initialize ROS2 and Create an Adapter

Add ROS2 initialization and create a `NevilV1Adapter` instance:

```python
# v1.0
from picarlibs.picarx import Picarx
car = Picarx()

# v2.0
import rclpy
from nevil_navigation.nevil_navigation_api.compat import NevilV1Adapter

rclpy.init()
car = NevilV1Adapter()
```

### Step 3: Update Function Calls (Optional)

If you want to use the new API directly instead of the compatibility layer:

```python
# v1.0
move_forward_this_way(car, 30, 50)
turn_left(car)
stop(car)

# v2.0
api = car.api  # Get the API instance from the adapter
api.move_forward_this_way(30, 0.5)  # Note: speed is now 0-1 instead of 0-100
api.turn_left()
api.stop()
```

### Step 4: Clean Up

Add cleanup code at the end of your script:

```python
# v2.0
car.api.shutdown()
rclpy.shutdown()
```

### Step 5: Handle ROS2 Spin

If your script needs to run for a while and receive callbacks:

```python
# v2.0
import threading

# Create a thread to spin the ROS2 executor
spin_thread = threading.Thread(target=rclpy.spin, args=(car.api.node,))
spin_thread.daemon = True
spin_thread.start()

# Your code here...

# Clean up
car.api.shutdown()
rclpy.shutdown()
```

## Examples

See the `examples/` directory for complete example scripts:

- `basic_movement.py`: Demonstrates basic movement commands
- `complex_navigation.py`: Demonstrates complex navigation sequences
- `sensor_integration.py`: Demonstrates integration with sensor data
- `error_handling.py`: Demonstrates error handling and recovery

## Advanced Usage

### Using the Async API

The async API provides non-blocking versions of all functions, which can be useful for more complex applications:

```python
import asyncio
import rclpy
from nevil_navigation.nevil_navigation_api.async_api import AsyncNevilNavigationAPI

async def main():
    rclpy.init()
    node = rclpy.create_node('async_example')
    
    # Set up the executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create the API instance
    api = AsyncNevilNavigationAPI(node)
    
    # Perform a sequence of movements without blocking
    await api.move_forward_this_way(30, 0.5)
    await api.turn_left()
    await api.move_forward_this_way(20, 0.5)
    await api.turn_right()
    await api.move_forward_this_way(10, 0.5)
    await api.stop()
    
    # Clean up
    api.shutdown()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

# Run the async main function
asyncio.run(main())
```

### Real-Time Integration

For critical operations that require real-time performance:

```python
import rclpy
from nevil_navigation.nevil_navigation_api.core import NevilNavigationAPI
from nevil_navigation.nevil_navigation_api.rt_integration import RTContext, RTNode

# Create a real-time node
node = RTNode('rt_example', priority=90)

# Create the API instance
api = NevilNavigationAPI(node)

# Create a real-time context
rt_context = RTContext(node, priority=90)

# Use the context manager for critical operations
with rt_context:
    # This code runs with real-time priority
    api.move_forward_this_way(30, 0.5)
    api.turn_left()
    api.stop()

# Clean up
api.shutdown()
node.destroy_node()
rclpy.shutdown()
```

## Troubleshooting

### Common Issues

#### "Action server not available" error

Make sure the required action servers are running:

```bash
# Check if the action servers are running
ros2 action list
```

#### "Service not available" error

Make sure the required services are running:

```bash
# Check if the services are running
ros2 service list
```

#### "Failed to set thread priority" error

Make sure you have the necessary permissions to set real-time priorities:

```bash
# Run with sudo or use chrt
sudo python3 my_script.py

# Or use chrt
chrt -f 90 python3 my_script.py
```

#### "Emergency stop triggered" error

Check for obstacles in front of the robot:

```python
# Check the distance
distance = api.get_distance()
print(f"Distance: {distance:.2f}m")
```

### Debugging

Enable debug logging for more detailed information:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

Use the `rqt_graph` tool to visualize the ROS2 node graph:

```bash
ros2 run rqt_graph rqt_graph
```

## Contributing

Contributions to the Nevil Navigation API are welcome! Please follow the standard ROS2 development practices and submit pull requests to the main repository.

## License

This software is released under the MIT License.