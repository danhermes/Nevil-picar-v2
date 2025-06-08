# RTHardwareInterface

## Overview

The `RTHardwareInterface` class provides a thread-safe interface to the PiCar-X hardware for the Nevil-picar v2.0 system. It serves as the hardware abstraction layer (HAL) between the ROS2 nodes and the physical robot hardware.

## Features

- Thread-safe hardware access with proper mutex handling for real-time performance
- Support for both hardware and simulation modes
- Comprehensive error handling and logging
- Integration with the PiCar-X platform using the robot_hat library

## Hardware Functions

### Motor Control
- `set_motor_speeds(left, right)`: Set the speeds of the left and right motors (-1.0 to 1.0)
- `stop()`: Stop all motors

### Steering
- `set_steering_angle(angle)`: Set the steering angle (-30 to 30 degrees)

### Sensors
- `get_distance()`: Get distance from ultrasonic sensor (in centimeters)

### Camera Control
- `set_camera_pan(angle)`: Set camera pan angle (-90 to 90 degrees)
- `set_camera_tilt(angle)`: Set camera tilt angle (-35 to 65 degrees)

### System Functions
- `reset()`: Reset the hardware to default state
- `cleanup()`: Clean up hardware resources

## Usage

### Initialization

```python
from nevil_realtime.rt_hardware_interface import RTHardwareInterface
from rclpy.node import Node

# Create a ROS2 node
node = Node('my_node')

# Create the hardware interface
hw = RTHardwareInterface(node)
```

### Motor Control Example

```python
# Move forward at half speed
hw.set_motor_speeds(0.5, 0.5)

# Turn right (differential drive)
hw.set_motor_speeds(0.5, 0.2)

# Stop
hw.stop()
```

### Steering Example

```python
# Turn right
hw.set_steering_angle(30)

# Turn left
hw.set_steering_angle(-30)

# Center steering
hw.set_steering_angle(0)
```

### Cleanup

```python
# Always call cleanup when done
hw.cleanup()
```

## Testing

You can test the hardware interface using the built-in test function:

```bash
cd ~/nevil/src && source install/setup.bash && python3 -c "from nevil_realtime.rt_hardware_interface import main; main()"
```

This will run a series of tests to verify the hardware functionality, including:
- Motor control
- Steering
- Distance sensing
- Camera control

## Implementation Details

### Thread Safety

All hardware access is protected by a mutex to ensure thread safety in a real-time environment. This prevents race conditions when multiple nodes might try to access the hardware simultaneously.

```python
with self.hardware_mutex:
    # Hardware access here
```

### Simulation Mode

The interface automatically falls back to simulation mode if hardware initialization fails. This allows for testing and development without the physical hardware.

```python
if self.simulation_mode:
    # Simulate hardware behavior
else:
    # Access actual hardware
```

### Error Handling

All hardware operations are wrapped in try-except blocks to handle potential hardware errors gracefully:

```python
try:
    with self.hardware_mutex:
        # Hardware operation
except Exception as e:
    self.logger.error(f'Error message: {e}')
```

## Integration with ROS2

The `RTHardwareInterface` is used by the `rt_motor_control_node` to control the physical robot based on velocity commands received via the `cmd_vel` topic. This provides a clean separation between the ROS2 control logic and the hardware-specific implementation.