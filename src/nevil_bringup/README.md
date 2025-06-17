# Nevil Bringup

Integration package for the Nevil-picar v2.0 project. This package provides launch files and configuration for different system configurations.

## Overview

The `nevil_bringup` package serves as the main integration point for the Nevil-picar v2.0 system. It provides:

- Launch files for different system configurations
- Configuration files for various scenarios
- Command-line interface for system management
- System startup and shutdown procedures
- Integration testing tools

## Quick Start

### Launch the Physical Robot System
```bash
./nevil launch nevil_bringup physical_robot.launch.py
```

### Shutdown All Nodes
```bash
# Graceful shutdown (if launch terminal is active)
Ctrl+C

# Force shutdown all Nevil processes
sudo pkill -f "nevil"
```

ros2 node list | grep ai_interface


## Launch Configurations

### **Physical Robot Launch** (Recommended)
Starts the complete system on the physical robot hardware with all components:

```bash
# Using nevil wrapper (recommended)
./nevil launch nevil_bringup physical_robot.launch.py

#diagnositc

ros2 node list
ros2 topic info /nevil/text_command


# Direct ROS2 launch
source install/setup.bash
ros2 launch nevil_bringup physical_robot.launch.py
```

**Includes:**
- AI Interface System (speech recognition, synthesis, dialog management)
- Real-time Components (motor control, sensor processing)
- Hardware Interface (I2C, motors, servos, sensors, camera)
- System Monitoring (battery, system status)

### **Launch Arguments**
```bash
# Enable/disable voice interface
./nevil launch nevil_bringup physical_robot.launch.py enable_voice:=true

# Enable/disable real-time features
./nevil launch nevil_bringup physical_robot.launch.py enable_rt:=true

# Custom configuration file
./nevil launch nevil_bringup physical_robot.launch.py config_file:=/path/to/config.yaml
```

### **Other Launch Configurations**

#### **Full System Launch**
```bash
ros2 launch nevil_bringup full_system.launch.py
```

#### **Minimal System Launch**
```bash
ros2 launch nevil_bringup minimal_system.launch.py
```

#### **Simulation Mode**
```bash
ros2 launch nevil_bringup simulation.launch.py
```

#### **Development Mode**
```bash
ros2 launch nevil_bringup development.launch.py
```

## System Shutdown

### **Graceful Shutdown Methods**

#### **Method 1: Keyboard Interrupt (Recommended)**
If you have the launch terminal active:
```bash
Ctrl+C
```

#### **Method 2: Kill All Nevil Processes**
```bash
pkill -f "nevil"
```

#### **Method 3: Selective Shutdown**
```bash
# Kill specific node types
pkill -f "ai_interface_node"
pkill -f "speech_recognition_node"
pkill -f "speech_synthesis_node"
pkill -f "rt_sensor_node"
pkill -f "rt_motor_control_node"
pkill -f "system_manager"
```

#### **Method 4: ROS2 Process Cleanup**
```bash
# Kill all ROS2 processes
pkill -f "ros2"

# Kill Python-based nodes
pkill -f "python.*nevil"
```

### **Verify Shutdown**
```bash
# Check for remaining processes
ps aux | grep nevil
ps aux | grep ros2

# Should return no results if shutdown was successful
```

### **Emergency Shutdown**
If nodes are unresponsive:
```bash
# Force kill (use with caution)
sudo pkill -9 python3
sudo pkill -9 ros2
```

## System Status Monitoring

### **Check Running Nodes**
```bash
# List all ROS2 nodes
ros2 node list

# Check specific node status
ros2 node info /ai_interface
ros2 node info /rt_sensor
```

### **Monitor System Topics**
```bash
# List active topics
ros2 topic list

# Monitor battery status
ros2 topic echo /nevil/battery_status

# Monitor system status
ros2 topic echo /nevil/system_status
```

### **View Logs**
```bash
# Real-time log viewing
ros2 log view

# Check latest launch logs
ls -la ~/.ros/log/ | tail -5
```

## Configuration System

The configuration system allows for easy customization of system parameters:

- Default configurations are provided in the `config` directory
- Parameters can be overridden via launch arguments
- Feature toggling is supported (e.g., enabling/disabling voice interface)

## Command-line Interface

The `nevil_cli` tool provides easy access to common operations:

```
ros2 run nevil_bringup nevil_cli --help
```

## Integration Testing

Integration tests are provided to verify system functionality:

```
ros2 launch nevil_bringup integration_test.launch.py
### Configuration Files

- `physical_robot_config.yaml` - Physical robot hardware configuration
- `simulation_config.yaml` - Simulation environment settings
- `development_config.yaml` - Development and debugging settings

### Parameter Override Examples

```bash
# Override specific parameters
./nevil launch nevil_bringup physical_robot.launch.py \
  battery_threshold:=7.0 \
  control_rate:=60.0 \
  max_speed:=0.8
```

## System Architecture

The bringup package coordinates the following system components:

### Core Components
- **System Manager**: Central coordination and state management
- **Hardware Interface**: Low-level hardware control and monitoring
- **Configuration Manager**: Parameter management and validation

### AI Interface Components
- **Speech Recognition**: Voice-to-text processing
- **Speech Synthesis**: Text-to-speech output
- **Dialog Manager**: Conversation state management
- **AI Interface**: Central AI coordination

### Real-time Components
- **Motor Control**: Real-time motor control with deterministic timing
- **Sensor Processing**: Real-time sensor data acquisition and filtering
- **Configuration Manager**: Real-time parameter management

```

## Troubleshooting

### Common Issues

#### Launch Fails with Import Errors
```bash
# Rebuild affected packages
./nevil build --packages-select nevil_interfaces_ai_msgs nevil_interfaces_ai

# Verify Python path
echo $PYTHONPATH
```

#### Hardware Initialization Fails
```bash
# Check I2C permissions
ls -la /dev/i2c-*

# Verify picar-x library
ls -la /home/dan/picar-x/picarx
```

#### Real-time Nodes Fail to Start
```bash
# Check configuration format
cat install/nevil_realtime/share/nevil_realtime/config/rt_config.yaml

# Verify callback group imports
grep -r "RealtimeCallbackGroup" install/nevil_realtime/
```

### Log Analysis

```bash
# Find latest logs
ls -la ~/.ros/log/ | tail -5

# Check for errors
grep -i error ~/.ros/log/[latest-directory]/launch.log

# Monitor real-time performance
ros2 topic echo /rt_sensor/latency_stats
```

## Development

### Adding New Launch Configurations

1. Create new launch file in `launch/` directory
2. Follow existing patterns for parameter handling
3. Add configuration file in `config/` directory
4. Update this README with usage instructions

### Extending System Components

1. Add new packages to launch dependencies
2. Update launch files to include new components
3. Add appropriate configuration parameters
4. Test integration with existing components

## Support

For additional support and documentation:

- [Launch and System Management Guide](../docs/launch_system_guide.md)
- [Build Status Documentation](../docs/build/BUILD_STATUS.md)
- [AI Interface Architecture](../docs/ai_interface_architecture.md)
- [Troubleshooting Guide](../docs/Nevil2.0%20tech/7_troubleshooting.md)

## Package Dependencies

- `nevil_core` - Core system components
- `nevil_interfaces_ai` - AI interface nodes
- `nevil_interfaces_ai_msgs` - AI message definitions
- `nevil_realtime` - Real-time components
- `nevil_simulation` - Simulation components (optional)
- `nevil_perception` - Perception components (optional)
- `nevil_navigation` - Navigation components (optional)