# Nevil Simulation

Digital twin simulation for Nevil-picar v2.0 based on the ARCHES-PiCar-X project.

## Description

The `nevil_simulation` package provides a virtual environment for testing and development of the Nevil-picar v2.0 robot without requiring physical hardware. It includes:

- A hardware abstraction layer compatible with the real PiCar-X hardware
- A physics-based simulation of the PiCar-X robot
- Virtual sensors (camera, ultrasonic, etc.) that publish data to ROS2 topics
- Simulated actuators (motors, servos) that respond to ROS2 commands
- A visualization system for monitoring the robot's state and environment
- Configurable virtual environments with obstacles and navigation challenges

## Components

### Simulation Hardware Interface

The Simulation Hardware Interface provides the same API as the real hardware interface, allowing seamless switching between simulation and real hardware. It implements:

- Motor control simulation
- Servo control simulation
- Ultrasonic sensor simulation
- Camera simulation
- Other sensor simulations

### Physics Engine

The Physics Engine simulates the physical behavior of the robot, including:

- Motion dynamics
- Collision detection
- Sensor interactions with the environment
- Realistic motor and servo behavior

### Visualization System

The Visualization System provides a visual representation of the robot and its environment, including:

- Robot model visualization
- Sensor data visualization
- Environment visualization
- Debug information display

### Environment Generator

The Environment Generator creates virtual environments for testing, including:

- Obstacle courses
- Navigation challenges
- Different terrain types
- Configurable environments via YAML files

## Usage

### Launch the Simulation

```bash
# Launch the basic simulation
ros2 launch nevil_simulation nevil_simulation.launch.py

# Launch with a specific environment
ros2 launch nevil_simulation nevil_simulation.launch.py environment:=obstacle_course

# Launch with visualization
ros2 launch nevil_simulation nevil_simulation_with_viz.launch.py
```

### Switch Between Real and Simulated Hardware

The system can be configured to use either real or simulated hardware:

```bash
# Use simulated hardware
ros2 launch nevil_core nevil_system.launch.py use_sim:=true

# Use real hardware
ros2 launch nevil_core nevil_system.launch.py use_sim:=false
```

### Create Custom Environments

Custom environments can be created by defining YAML configuration files:

```yaml
# Example environment configuration
name: obstacle_course
size: [10.0, 10.0]
obstacles:
  - type: box
    position: [2.0, 3.0, 0.0]
    dimensions: [1.0, 1.0, 1.0]
  - type: cylinder
    position: [5.0, 5.0, 0.0]
    radius: 0.5
    height: 1.0
```

## Dependencies

- ROS2 Humble
- nevil_core
- nevil_navigation
- nevil_perception
- nevil_realtime
- nevil_interfaces

## License

Apache License 2.0