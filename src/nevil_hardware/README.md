# Nevil Hardware Package v2.0

A comprehensive hardware abstraction layer for the Nevil-picar robot, providing clean separation between physical hardware and simulation modes through a well-defined interface architecture.

## Overview

The `nevil_hardware` package implements a modular hardware abstraction system that enables:

- **Clean Interface Separation**: Abstract base classes define clear contracts for hardware components
- **Runtime Backend Selection**: Automatic detection and configuration-driven selection between physical and simulation modes
- **Graceful Degradation**: System automatically falls back to simulation when physical hardware is unavailable
- **Extensible Architecture**: Easy addition of new hardware types and implementations

## Architecture

### Core Components

```
nevil_hardware/
├── interfaces/           # Abstract base classes
│   └── movement_interface.py
├── drivers/             # Concrete implementations
│   ├── physical/        # Physical hardware drivers
│   │   └── picarx_movement_driver.py
│   └── simulation/      # Simulation drivers
│       └── simulation_movement_driver.py
└── hardware_manager.py  # Factory and configuration manager
```

### Key Classes

#### MovementInterface (Abstract Base Class)
```python
from nevil_hardware.interfaces.movement_interface import MovementInterface

class MovementInterface(ABC):
    def initialize(self) -> bool
    def set_motor_speeds(self, left: float, right: float) -> bool
    def set_steering_angle(self, angle: float) -> bool
    def stop(self) -> bool
    def reset(self) -> bool
    def get_status(self) -> MovementStatus
    def cleanup(self) -> None
```

#### HardwareManager (Factory)
```python
from nevil_hardware.hardware_manager import HardwareManager

manager = HardwareManager()
movement = manager.get_movement_interface()
```

## Usage

### Basic Usage

```python
from nevil_hardware import HardwareManager

# Create hardware manager
manager = HardwareManager()

# Get movement interface (auto-detects backend)
movement = manager.get_movement_interface()

# Use the interface
if movement.initialize():
    movement.forward(0.5)  # 50% speed forward
    time.sleep(2)
    movement.stop()
    movement.cleanup()
```

### ROS2 Integration

```python
import rclpy
from nevil_hardware.hardware_manager import get_movement_interface_from_params

def main():
    rclpy.init()
    node = rclpy.create_node('hardware_test')
    
    # Get interface from ROS2 parameters
    movement = get_movement_interface_from_params(node)
    
    # Use the interface
    movement.set_motor_speeds(0.3, 0.3)  # Move forward
```

### Configuration-Driven Selection

```python
# Force specific backend
manager = HardwareManager()
movement = manager.get_movement_interface(
    backend_type="simulation",
    config={'update_rate': 100.0}
)

# Force simulation mode
movement = manager.get_movement_interface(
    force_simulation=True
)
```

## Backend Types

### Physical Hardware Backend
- **Type**: `"physical"`
- **Implementation**: `PiCarXMovementDriver`
- **Requirements**: 
  - `robot_hat` library
  - GPIO access
  - PiCar-X hardware libraries
- **Features**:
  - Direct hardware control
  - Real sensor feedback
  - Hardware-specific optimizations

### Simulation Backend
- **Type**: `"simulation"`
- **Implementation**: `SimulationMovementDriver`
- **Requirements**: None (pure Python)
- **Features**:
  - Virtual physics simulation
  - Realistic movement constraints
  - Configurable parameters
  - Thread-safe operation

### Mock Backend
- **Type**: `"mock"`
- **Implementation**: `MockMovementInterface`
- **Requirements**: None
- **Features**:
  - Minimal implementation for testing
  - No actual movement
  - Status tracking

## Configuration

### Launch Parameters

```xml
<launch>
  <node pkg="your_package" exec="your_node">
    <param name="hardware_backend" value="auto"/>  <!-- auto, physical, simulation, mock -->
    <param name="force_simulation" value="false"/>
  </node>
</launch>
```

### Simulation Configuration

```python
config = {
    'update_rate': 50.0,        # Hz
    'max_speed': 1.0,           # m/s
    'max_angular_speed': 2.0,   # rad/s
    'acceleration_limit': 2.0,  # m/s²
    'wheel_base': 0.15,         # meters
    'wheel_radius': 0.033       # meters
}

movement = manager.get_movement_interface(
    backend_type="simulation",
    config=config
)
```

## Hardware Detection

The system automatically detects available hardware:

```python
manager = HardwareManager()
availability = manager.detect_hardware_availability()

print(f"Robot HAT: {availability['robot_hat']}")
print(f"GPIO: {availability['gpio']}")
print(f"PiCar-X libs: {availability['picarx_libs']}")
```

## Status Monitoring

```python
status = movement.get_status()
print(f"Backend: {status.backend_type}")
print(f"Hardware Available: {status.hardware_available}")
print(f"Moving: {status.is_moving}")
print(f"Left Motor: {status.left_motor_speed}")
print(f"Right Motor: {status.right_motor_speed}")
print(f"Steering: {status.steering_angle}")
```

## Error Handling

```python
try:
    movement = manager.get_movement_interface()
    if not movement.initialize():
        print(f"Initialization failed: {movement.get_status().error_message}")
except ImportError as e:
    print(f"Backend not available: {e}")
except RuntimeError as e:
    print(f"Hardware error: {e}")
```

## Testing

### Unit Tests
```bash
# Run all tests
colcon test --packages-select nevil_hardware

# Run specific test
python -m pytest test/test_movement_interface.py
```

### Manual Testing
```python
# Test simulation backend
from nevil_hardware.drivers.simulation import SimulationMovementDriver

driver = SimulationMovementDriver()
driver.initialize()
driver.forward(0.5)
print(driver.get_virtual_position())
```

## Integration with Existing System

### Replacing RTHardwareInterface

The new architecture replaces the existing `RTHardwareInterface`:

```python
# Old approach
from nevil_realtime.rt_hardware_interface import RTHardwareInterface
rt_hw = RTHardwareInterface()

# New approach
from nevil_hardware import HardwareManager
manager = HardwareManager()
movement = manager.get_movement_interface()
```

### Migration Path

1. **Phase 1**: Install new package alongside existing system
2. **Phase 2**: Update motion control nodes to use new interfaces
3. **Phase 3**: Remove old RTHardwareInterface dependencies
4. **Phase 4**: Add launch parameters for backend selection

## Dependencies

### Required
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 messages
- `geometry_msgs` - Geometry-related messages

### Optional (Physical Hardware)
- `robot_hat` - PiCar-X hardware library
- `RPi.GPIO` - Raspberry Pi GPIO control

### Development
- `pytest` - Testing framework
- `ament_lint_auto` - Code linting
- `ament_lint_common` - Common linting rules

## Contributing

### Adding New Hardware Drivers

1. Create new driver class inheriting from `MovementInterface`
2. Implement all abstract methods
3. Add to appropriate driver module
4. Update `HardwareManager` factory logic
5. Add tests and documentation

### Example New Driver

```python
from nevil_hardware.interfaces.movement_interface import MovementInterface

class MyCustomDriver(MovementInterface):
    def initialize(self) -> bool:
        # Initialize your hardware
        return True
    
    def set_motor_speeds(self, left: float, right: float) -> bool:
        # Control your motors
        return True
    
    # Implement other required methods...
```

## Troubleshooting

### Common Issues

1. **Hardware not detected**: Check GPIO permissions and library installation
2. **Import errors**: Ensure all dependencies are installed
3. **Simulation not working**: Check Python threading support
4. **Performance issues**: Adjust simulation update rate

### Debug Mode

```python
import logging
logging.basicConfig(level=logging.DEBUG)

manager = HardwareManager(logger=logging.getLogger(__name__))
```

## License

MIT License - see LICENSE file for details.

## Version History

- **v2.0.0**: Complete architecture redesign with interface abstraction
- **v1.0.0**: Original tightly-coupled implementation