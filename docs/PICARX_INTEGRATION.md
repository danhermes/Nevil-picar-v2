# PiCar-X Integration for Nevil-picar v2.0

This document describes the integration of the PiCar-X robot platform with the Nevil robot system, providing a comprehensive hardware abstraction layer for movement control and sensor access.

## Overview

The PiCar-X integration provides:
- **Movement Control**: Direct motor control, steering, and basic movement primitives
- **Sensor Access**: Ultrasonic distance sensor, grayscale line sensors, and camera control
- **Hardware Abstraction**: Seamless switching between physical hardware and simulation
- **Safety Features**: Emergency stop, hardware detection, and error handling

## Architecture

```
Nevil Robot System
├── Hardware Manager (Factory)
├── Movement Interface (Abstract)
│   ├── PiCar-X Physical Driver
│   ├── Simulation Driver
│   └── Mock Driver
└── Sensor Interface (Abstract)
    ├── PiCar-X Physical Driver
    ├── Simulation Driver
    └── Mock Driver
```

## Hardware Requirements

### PiCar-X Platform
- PiCar-X robot kit with Raspberry Pi
- Robot HAT board for GPIO control
- Ultrasonic sensor (HC-SR04)
- Grayscale sensors (3x line detection)
- Camera servo mount (pan/tilt)
- Motors and steering servo

### Software Dependencies
- `picarx` library (from `/home/dan/picar-x`)
- `robot_hat` library for hardware control
- Python 3.7+ with GPIO access
- ROS2 (optional, for ROS integration)

## Installation

### 1. Install PiCar-X Library
```bash
cd /home/dan/picar-x
sudo python3 setup.py install
```

### 2. Install Dependencies
```bash
# Install robot_hat if not already installed
pip3 install robot-hat

# Install other dependencies
pip3 install RPi.GPIO  # For Raspberry Pi GPIO access
```

### 3. Hardware Setup
- Ensure PiCar-X is properly assembled
- Connect all sensors and actuators
- Calibrate servos using the provided calibration scripts
- Test basic functionality with PiCar-X examples

## Usage

### Basic Usage

```python
from nevil_hardware.hardware_manager import HardwareManager

# Create hardware manager
manager = HardwareManager()

# Get interfaces (auto-detects hardware)
movement = manager.get_movement_interface()
sensor = manager.get_sensor_interface()

# Basic movement
movement.forward(0.5)  # 50% speed
movement.set_steering_angle(15)  # 15 degrees right
movement.stop()

# Sensor reading
distance = sensor.get_ultrasonic_reading()
grayscale = sensor.get_grayscale_reading()
```

### Force Hardware Backend

```python
# Force physical hardware
movement = manager.get_movement_interface(backend_type='physical')

# Force simulation
movement = manager.get_movement_interface(backend_type='simulation')

# Force simulation mode
movement = manager.get_movement_interface(force_simulation=True)
```

### ROS2 Integration

```python
import rclpy
from nevil_hardware.hardware_manager import get_both_interfaces_from_params

def main():
    rclpy.init()
    node = rclpy.create_node('picarx_node')
    
    # Get interfaces from ROS parameters
    movement, sensor = get_both_interfaces_from_params(node)
    
    # Use interfaces...
    
    rclpy.shutdown()
```

## API Reference

### Movement Interface

#### Basic Movement
- `forward(speed: float)` - Move forward at specified speed (0.0-1.0)
- `backward(speed: float)` - Move backward at specified speed (0.0-1.0)
- `turn_left(speed: float)` - Turn left in place
- `turn_right(speed: float)` - Turn right in place
- `stop()` - Stop all movement immediately

#### Advanced Control
- `set_motor_speeds(left: float, right: float)` - Set individual motor speeds
- `set_steering_angle(angle: float)` - Set steering angle (-30 to 30 degrees)
- `reset()` - Reset to default state

#### Status and Control
- `get_status()` - Get current movement status
- `is_available()` - Check if hardware is available
- `get_backend_type()` - Get backend type (physical/simulation/mock)

### Sensor Interface

#### Ultrasonic Sensor
- `get_ultrasonic_reading()` - Get distance reading in cm
- `is_obstacle_detected(threshold: float)` - Check for obstacles within threshold

#### Grayscale Sensors
- `get_grayscale_reading()` - Get raw grayscale sensor values
- `is_line_detected()` - Check if line is detected
- `is_cliff_detected()` - Check if cliff is detected
- `calibrate_grayscale(values: List[float])` - Calibrate sensors

#### Camera Control
- `set_camera_angles(pan: float, tilt: float)` - Set camera position
- `get_camera_status()` - Get current camera status

### PiCar-X Specific Features

When using physical hardware, additional PiCar-X specific methods are available:

```python
# Movement driver specific
movement.emergency_stop()  # Activate emergency stop
movement.clear_emergency_stop()  # Clear emergency stop
movement.get_distance()  # Direct ultrasonic access
movement.set_camera_angle(pan=45, tilt=30)  # Camera control

# Sensor driver specific
sensor.set_cliff_reference([500, 500, 500])  # Set cliff detection threshold
sensor.reset_camera_position()  # Reset camera to center
```

## Configuration

### Hardware Configuration

```python
config = {
    'speed_scale': 0.8,        # Scale factor for motor speeds
    'steering_scale': 1.0,     # Scale factor for steering
    'safety_timeout': 2.0,     # Safety timeout in seconds
    'line_reference': [1000, 1000, 1000],    # Line detection reference
    'cliff_reference': [500, 500, 500],      # Cliff detection reference
}

movement = manager.get_movement_interface(config=config)
```

### ROS2 Parameters

```yaml
# In your launch file or parameter file
hardware_backend: "auto"      # auto, physical, simulation, mock
force_simulation: false       # Force simulation mode
speed_scale: 0.8             # Motor speed scaling
steering_scale: 1.0          # Steering angle scaling
```

## Testing

### Integration Test
```bash
# Run comprehensive integration test
python3 test_picarx_integration.py
```

### Basic Usage Example
```bash
# Run basic usage demonstration
cd examples
python3 picarx_basic_usage.py
```

### Hardware-Only Test
```bash
# Test only physical hardware (will fail gracefully if not available)
python3 -c "
from nevil_hardware.hardware_manager import HardwareManager
manager = HardwareManager()
movement = manager.get_movement_interface(backend_type='physical')
print(f'Hardware available: {movement.is_available()}')
"
```

## Troubleshooting

### Common Issues

#### 1. PiCar-X Library Not Found
```
ImportError: No module named 'picarx'
```
**Solution**: Ensure the PiCar-X library path is correct and the library is installed.

#### 2. GPIO Permission Denied
```
RuntimeError: No access to /dev/mem
```
**Solution**: Run with sudo or add user to gpio group:
```bash
sudo usermod -a -G gpio $USER
```

#### 3. Hardware Not Detected
```
Physical hardware not available, using simulation
```
**Solution**: Check hardware connections and ensure robot_hat library is installed.

#### 4. Servo Calibration Issues
**Solution**: Run the servo calibration script:
```bash
cd /home/dan/picar-x/example
python3 servo_zeroing.py
```

### Debug Mode

Enable debug logging for detailed troubleshooting:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

manager = HardwareManager()
# Detailed logs will show hardware detection and initialization
```

## Safety Considerations

### Emergency Stop
- Always implement emergency stop functionality
- Use `movement.stop()` in exception handlers
- Consider implementing watchdog timers for autonomous operation

### Hardware Limits
- Motor speeds are limited to -1.0 to 1.0 range
- Steering angles are limited to -30 to 30 degrees
- Camera pan: -90 to 90 degrees, tilt: -35 to 65 degrees

### Error Handling
```python
try:
    movement.forward(0.5)
    time.sleep(2)
except Exception as e:
    movement.stop()  # Always stop on error
    logger.error(f"Movement failed: {e}")
```

## Performance Notes

- Sensor readings are not cached - call frequency affects performance
- Hardware initialization takes ~1-2 seconds
- Emergency stop response time is <100ms
- Ultrasonic sensor has ~30ms update rate
- Grayscale sensors can be read at ~100Hz

## Integration with Nevil System

The PiCar-X integration is designed to work seamlessly with the broader Nevil robot system:

- **Navigation**: Provides movement primitives for path planning
- **Perception**: Sensor data feeds into obstacle detection and line following
- **AI Interface**: Can be controlled via voice commands and AI decision making
- **Simulation**: Allows development and testing without physical hardware

## Future Enhancements

Planned improvements include:
- Camera image capture and processing
- IMU sensor integration
- Battery monitoring
- Advanced motor control (PID, acceleration curves)
- Multi-robot coordination
- ROS2 action servers for complex behaviors

## Contributing

When contributing to the PiCar-X integration:

1. Follow the existing interface patterns
2. Add comprehensive error handling
3. Include unit tests for new features
4. Update documentation
5. Test on both physical hardware and simulation

## License

This integration follows the same license as the Nevil-picar project. The PiCar-X library itself is licensed under GNU GPL.