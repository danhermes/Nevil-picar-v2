# Nevil Realtime

This package provides real-time components for the Nevil-picar v2.0 robot using the PREEMPT-RT Linux kernel. It enables deterministic execution of critical tasks with low latency and jitter.

## Features

- Real-time thread management utilities
- Priority management for critical tasks
- Real-time safe communication mechanisms
- Real-time executor for ROS2 nodes
- Real-time safe hardware interface for PiCar-X
- Configuration system for real-time parameters
- Example nodes demonstrating real-time capabilities

## Requirements

- Raspberry Pi 4/5 with PREEMPT-RT patched Linux kernel
- ROS2 Humble
- Cyclone DDS (recommended for real-time performance)

## Installation

### PREEMPT-RT Kernel

Follow the instructions in the [Nevil PREEMPT-RT Integration Guide](../../docs/Nevil_PREEMPT_RT_Integration.md) to install and configure the PREEMPT-RT kernel on your Raspberry Pi.

### Package Installation

```bash
cd ~/ros2_ws
colcon build --packages-select nevil_realtime
source install/setup.bash
```

## Usage

### Configuration

Edit the configuration file to set thread priorities and other real-time parameters:

```bash
ros2 run nevil_realtime rt_config_manager --edit
```

### Running Example Nodes

```bash
# Launch all real-time components
ros2 launch nevil_realtime nevil_realtime.launch.py

# Run individual nodes with real-time priority
sudo chrt -f 90 ros2 run nevil_realtime rt_motor_control_node
sudo chrt -f 85 ros2 run nevil_realtime rt_sensor_node
```

### Priority Inheritance Demo

```bash
ros2 run nevil_realtime priority_inheritance_demo
```

## Components

### Real-time Thread Management

The `RTThreadUtils` class provides utilities for managing real-time threads, including:

- Setting thread priority and scheduling policy
- Managing CPU affinity
- Locking memory to prevent paging
- Thread synchronization with priority inheritance

### Real-time Executor

The `RTExecutor` class extends ROS2's executor model to provide deterministic execution of callbacks with proper priority handling.

### Real-time Hardware Interface

The `RTHardwareInterface` class provides low-latency access to the PiCar-X hardware with proper mutex handling and bounded execution time.

### Configuration System

The `RTConfigManager` allows configuring real-time parameters through a YAML file and ROS2 parameters.

## Best Practices

- Keep callbacks short and non-blocking
- Use ROS2 timers instead of sleep functions
- Avoid dynamic memory allocation in real-time code paths
- Use proper mutex handling with priority inheritance
- Monitor latency and jitter using the provided tools

## License

Apache License 2.0