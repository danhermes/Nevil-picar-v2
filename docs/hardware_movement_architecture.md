# Hardware Movement Architecture Design

## Overview
This document defines the decoupled hardware movement architecture for Nevil-picar v2.0, enabling seamless switching between physical hardware and simulation modes.

## Architecture Principles

### 1. Interface Segregation
- **Hardware Interface**: Abstract base class defining movement operations
- **Physical Driver**: Concrete implementation for real hardware
- **Simulation Driver**: Concrete implementation for virtual environment

### 2. Dependency Inversion
- High-level navigation logic depends on abstractions, not concrete implementations
- Hardware drivers implement interfaces, not the other way around

### 3. Strategy Pattern
- Runtime selection of hardware backend based on configuration
- No conditional logic in business code

## Component Hierarchy

```
MovementController (ROS2 Node)
├── HardwareManager (Factory/Registry)
│   ├── MovementInterface (Abstract Base Class)
│   │   ├── PhysicalMovementDriver (robot_hat implementation)
│   │   └── SimulationMovementDriver (virtual implementation)
│   └── SensorInterface (Abstract Base Class)
│       ├── PhysicalSensorDriver (ultrasonic, camera)
│       └── SimulationSensorDriver (virtual sensors)
```

## Interface Definitions

### MovementInterface
```python
class MovementInterface(ABC):
    @abstractmethod
    def set_motor_speeds(self, left: float, right: float) -> None
    
    @abstractmethod
    def set_steering_angle(self, angle: float) -> None
    
    @abstractmethod
    def stop(self) -> None
    
    @abstractmethod
    def reset(self) -> None
    
    @abstractmethod
    def get_status(self) -> MovementStatus
```

### SensorInterface
```python
class SensorInterface(ABC):
    @abstractmethod
    def get_distance(self) -> float
    
    @abstractmethod
    def set_camera_pan(self, angle: float) -> None
    
    @abstractmethod
    def set_camera_tilt(self, angle: float) -> None
    
    @abstractmethod
    def get_sensor_data(self) -> SensorData
```

## Configuration-Driven Selection

### Launch Parameters
```yaml
hardware_mode: "physical"  # or "simulation"
physical_backend: "picarx"  # or "custom"
simulation_backend: "gazebo"  # or "virtual"
```

### Runtime Detection
```python
def detect_hardware_mode():
    if robot_hat_available() and gpio_accessible():
        return "physical"
    return "simulation"
```

## Implementation Strategy

### Phase 1: Interface Definition
1. Create abstract base classes
2. Define data structures (MovementStatus, SensorData)
3. Establish error handling patterns

### Phase 2: Physical Implementation
1. Refactor existing RTHardwareInterface to implement MovementInterface
2. Separate robot_hat dependencies into PhysicalMovementDriver
3. Add proper error handling and status reporting

### Phase 3: Simulation Implementation
1. Create SimulationMovementDriver with virtual physics
2. Implement sensor simulation with realistic data
3. Add visualization capabilities

### Phase 4: Integration
1. Create HardwareManager factory class
2. Update MovementController to use interfaces
3. Add configuration-driven selection logic

## Benefits

### Development
- **Testability**: Unit tests can use mock implementations
- **Debugging**: Simulation mode for development without hardware
- **CI/CD**: Automated testing without physical dependencies

### Deployment
- **Flexibility**: Same codebase runs on hardware or simulation
- **Reliability**: Graceful degradation when hardware unavailable
- **Scalability**: Easy to add new hardware backends

### Maintenance
- **Separation of Concerns**: Hardware logic isolated from business logic
- **Extensibility**: New hardware types without core changes
- **Modularity**: Components can be developed independently

## File Structure

```
src/nevil_hardware/
├── nevil_hardware/
│   ├── interfaces/
│   │   ├── movement_interface.py
│   │   ├── sensor_interface.py
│   │   └── data_types.py
│   ├── drivers/
│   │   ├── physical/
│   │   │   ├── picarx_movement_driver.py
│   │   │   └── picarx_sensor_driver.py
│   │   └── simulation/
│   │       ├── virtual_movement_driver.py
│   │       └── virtual_sensor_driver.py
│   ├── manager/
│   │   └── hardware_manager.py
│   └── controller/
│       └── movement_controller_node.py
├── config/
│   ├── physical_config.yaml
│   └── simulation_config.yaml
└── launch/
    ├── physical_hardware.launch.py
    └── simulation_hardware.launch.py
```

## Migration Path

### Immediate (Current Sprint)
1. Create interface definitions
2. Refactor existing code to implement interfaces
3. Add configuration selection logic

### Short Term (Next Sprint)
1. Implement simulation driver
2. Add comprehensive testing
3. Update launch files

### Long Term (Future Sprints)
1. Add Gazebo integration
2. Implement additional hardware backends
3. Add hardware health monitoring

This architecture ensures clean separation between simulation and physical hardware while maintaining the flexibility to add new backends in the future.