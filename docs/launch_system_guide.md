# Nevil-picar v2.0 Launch and System Management Guide

## Table of Contents

1. [Quick Start](#quick-start)
2. [System Launch](#system-launch)
3. [System Shutdown](#system-shutdown)
4. [System Monitoring](#system-monitoring)
5. [Troubleshooting](#troubleshooting)
6. [Advanced Operations](#advanced-operations)

## Quick Start

### Launch Physical Robot System
```bash
cd ~/Nevil-picar-v2
./nevil launch nevil_bringup physical_robot.launch.py
```

### Shutdown System
```bash
# Graceful shutdown
Ctrl+C

# Force shutdown
pkill -f "nevil"
```

## System Launch

### Primary Launch Command

The recommended way to start the Nevil-picar v2.0 system:

```bash
./nevil launch nevil_bringup physical_robot.launch.py
```

This command:
- Sources the ROS2 environment automatically
- Starts all system components
- Provides proper error handling
- Includes dependency management

### Launch Components

When you run the physical robot launch, the following components start:

#### Core System Components
- **System Manager** (`system_manager.py`) - Central system coordination
- **Hardware Initialization** (`hardware_init`) - I2C, motors, servos, sensors setup
- **System Monitor** (`system_monitor`) - System health monitoring
- **Battery Monitor** (`battery_monitor`) - Power management and alerts

#### AI Interface Components
- **AI Interface Node** (`ai_interface_node`) - Central AI coordination
- **Speech Recognition** (`speech_recognition_node`) - Voice-to-text processing
- **Speech Synthesis** (`speech_synthesis_node`) - Text-to-speech output
- **Dialog Manager** (`dialog_manager_node`) - Conversation management

#### Real-time Components
- **Motor Control** (`rt_motor_control_node`) - Real-time motor control
- **Sensor Processing** (`rt_sensor_node`) - Real-time sensor data processing
- **Configuration Manager** (`rt_config_manager`) - Real-time system configuration

### Launch Arguments

Customize system behavior with launch arguments:

```bash
# Enable/disable voice interface
./nevil launch nevil_bringup physical_robot.launch.py enable_voice:=true

# Enable/disable real-time features
./nevil launch nevil_bringup physical_robot.launch.py enable_rt:=true

# Use custom configuration
./nevil launch nevil_bringup physical_robot.launch.py config_file:=/path/to/config.yaml

# Combined arguments
./nevil launch nevil_bringup physical_robot.launch.py enable_voice:=true enable_rt:=false
```

### Alternative Launch Methods

#### Direct ROS2 Launch
```bash
source install/setup.bash
ros2 launch nevil_bringup physical_robot.launch.py
```

#### Manual Environment Setup
```bash
source ~/ros2_humble/install/setup.bash
source install/setup.bash
export PYTHONPATH="/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"

# Ensure .env file is present for AI interface configuration
ls -la .env

ros2 launch nevil_bringup physical_robot.launch.py
```

## System Shutdown

### Graceful Shutdown Methods

#### Method 1: Keyboard Interrupt (Recommended)
If the launch terminal is active:
```bash
Ctrl+C
```

This method:
- Sends SIGINT to all processes
- Allows nodes to clean up resources
- Properly closes hardware connections
- Saves configuration state

#### Method 2: Process Termination
```bash
# Kill all Nevil-related processes
pkill -f "nevil"
```

#### Method 3: Selective Node Shutdown
```bash
# Shutdown specific components
pkill -f "ai_interface_node"
pkill -f "speech_recognition_node"
pkill -f "speech_synthesis_node"
pkill -f "dialog_manager_node"
pkill -f "rt_sensor_node"
pkill -f "rt_motor_control_node"
pkill -f "system_manager"
pkill -f "battery_monitor"
pkill -f "hardware_init"
```

#### Method 4: ROS2 Process Cleanup
```bash
# Kill all ROS2 processes
pkill -f "ros2"

# Kill Python-based ROS2 nodes
pkill -f "python.*nevil"

# Kill launch processes
pkill -f "launch"
```

### Emergency Shutdown

If nodes become unresponsive:

```bash
# Force kill Python processes (use with caution)
sudo pkill -9 python3

# Force kill ROS2 processes
sudo pkill -9 ros2

# Nuclear option - restart system
sudo reboot
```

### Shutdown Verification

Confirm all processes have stopped:

```bash
# Check for remaining Nevil processes
ps aux | grep nevil

# Check for ROS2 processes
ps aux | grep ros2

# Check for Python processes
ps aux | grep python | grep nevil

# Should return no results if shutdown was successful
```

## System Monitoring

### Navigation Monitor (Recommended)

The Navigation Monitor provides comprehensive real-time monitoring with critical error detection:

```bash
# Start navigation monitoring dashboard
./src/nevil_navigation/scripts/start_monitor.sh

# With logging for analysis
./src/nevil_navigation/scripts/start_monitor.sh --log-file /tmp/nav_monitor.log

# Custom refresh rate (faster updates)
./src/nevil_navigation/scripts/start_monitor.sh --refresh-rate 50
```

#### Key Features
- **Real-time Topic Monitoring**: Live updates for `/cmd_vel`, `/goal_pose`, `/system_mode`, `/nevil/action_command`, `/planned_path`
- **Critical Error Detection**: Immediate alerts for crashes, segfaults, and fatal errors
- **Node Health Monitoring**: Tracks critical nodes (`navigation_node`, `ai_interface_node`, `dialog_manager_node`, etc.)
- **System Resource Alerts**: Memory (>90%) and CPU (>95%) warnings
- **Interactive Controls**: Reset counters (r), pause/resume (p), clean shutdown (Ctrl+C)

#### Alert Levels
- 💀 **FATAL**: System crashes, segfaults - immediate attention required
- 🔴 **ERROR**: Failed operations, missing nodes - investigation needed
- 🟡 **WARNING**: Performance issues, timeouts - monitor closely

For detailed documentation, see [Navigation Monitor Documentation](../src/nevil_navigation/scripts/README_navigation_monitor.md).

### Real-time Status Monitoring

#### Check Running Nodes
```bash
# List all active ROS2 nodes
ros2 node list

# Check specific node information
ros2 node info /ai_interface
ros2 node info /rt_sensor
ros2 node info /system_manager
```

#### Monitor System Topics
```bash
# List all active topics
ros2 topic list

# Monitor battery status
ros2 topic echo /nevil/battery_status

# Monitor system status
ros2 topic echo /nevil/system_status

# Monitor AI interface status
ros2 topic echo /nevil/ai_status
```

#### View System Logs
```bash
# Real-time log viewing
ros2 log view

# Check latest launch logs
ls -la ~/.ros/log/ | tail -5

# View specific log file
cat ~/.ros/log/[latest-log-directory]/launch.log
```

### Performance Monitoring

#### Real-time Sensor Telemetry
Monitor sensor performance in real-time:
```bash
# Watch sensor latency reports
ros2 topic echo /rt_sensor/latency_stats

# Expected output:
# [INFO] [rt_sensor]: Sensor callback latency (ms): min=0.286, max=136.840, avg=0.675
```

#### System Resource Usage
```bash
# Monitor CPU and memory usage
htop

# Check specific process resources
ps aux | grep nevil | awk '{print $3, $4, $11}'
```

### Health Checks

#### Battery Status
```bash
# Check battery voltage
ros2 topic echo /nevil/battery_status --once

# Monitor for critical battery warnings
ros2 topic echo /nevil/battery_status | grep CRITICAL
```

#### Hardware Status
```bash
# Check hardware initialization status
ros2 service call /nevil/hardware_status std_srvs/srv/Trigger

# Monitor I2C bus status
ros2 topic echo /nevil/hardware_status
```

## Troubleshooting

### Common Launch Issues

#### Issue: "No module named 'nevil_interfaces_ai.msg'"
**Solution**: Import paths have been fixed. Rebuild if needed:
```bash
./nevil build --packages-select nevil_interfaces_ai
```

#### Issue: "YAML parameter parsing error"
**Solution**: Configuration format has been corrected. Verify config file:
```bash
cat install/nevil_realtime/share/nevil_realtime/config/rt_config.yaml
```

#### Issue: "RealtimeCallbackGroup not found"
**Solution**: Callback groups have been updated. Rebuild realtime package:
```bash
./nevil build --packages-select nevil_realtime
```

#### Issue: "picar library not found"
**Solution**: Ensure picar-x and robot_hat libraries are accessible:
```bash
ls -la /home/dan/picar-x/picarx
ls -la /home/dan/robot-hat
export PYTHONPATH="/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"
```

#### Issue: "OpenAI API key not found" or AI interface errors
**Solution**: Verify .env file exists and contains required configuration:
```bash
# Check .env file exists
ls -la .env

# Verify OpenAI API key is set
grep OPENAI_API_KEY .env

# Check all AI configuration variables
cat .env
```

### Node-Specific Troubleshooting

#### AI Interface Nodes
```bash
# Test individual AI interface node
./install/nevil_interfaces_ai/lib/nevil_interfaces_ai/ai_interface_node --ros-args -r __node:=test_ai

# Check message interface availability
python3 -c "from nevil_interfaces_ai_msgs.msg import TextCommand; print('Messages available')"
```

#### Real-time Nodes
```bash
# Test sensor node individually
./install/nevil_realtime/lib/nevil_realtime/rt_sensor_node --ros-args -r __node:=test_sensor

# Check real-time configuration
cat install/nevil_realtime/share/nevil_realtime/config/rt_config.yaml
```

#### Hardware Nodes
```bash
# Test hardware initialization
./install/nevil_bringup/lib/nevil_bringup/hardware_init --ros-args -r __node:=test_hardware

# Check I2C permissions
ls -la /dev/i2c-*
```

### Log Analysis

#### Find Recent Logs
```bash
# Find latest log directory
ls -la ~/.ros/log/ | grep "$(date +%Y-%m-%d)" | tail -1

# View launch log
cat ~/.ros/log/[latest-directory]/launch.log
```

#### Common Log Patterns
```bash
# Check for successful node starts
grep "process started" ~/.ros/log/[latest-directory]/launch.log

# Check for node failures
grep "process has died" ~/.ros/log/[latest-directory]/launch.log

# Check for clean exits
grep "process has finished cleanly" ~/.ros/log/[latest-directory]/launch.log
```

## Advanced Operations

### Custom Launch Configurations

#### Create Custom Launch File
```python
# custom_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nevil_interfaces_ai',
            executable='ai_interface_node',
            name='custom_ai_interface',
            parameters=[{'custom_param': 'value'}]
        )
    ])
```

#### Launch with Custom Configuration
```bash
ros2 launch custom_launch.py
```

### Development and Debugging

#### Launch with Debug Output
```bash
# Enable debug logging
ros2 launch nevil_bringup physical_robot.launch.py --ros-args --log-level DEBUG

# Launch specific nodes with debug
ros2 run nevil_interfaces_ai ai_interface_node --ros-args --log-level DEBUG
```

#### Performance Profiling
```bash
# Monitor system performance during launch
top -p $(pgrep -d',' -f nevil)

# Profile memory usage
valgrind --tool=massif ros2 launch nevil_bringup physical_robot.launch.py
```

### System Integration Testing

#### Verify Complete System
```bash
# Launch system
./nevil launch nevil_bringup physical_robot.launch.py

# In another terminal, verify all components
ros2 node list | grep -E "(ai_interface|rt_sensor|system_manager|battery_monitor)"

# Test voice interface
ros2 topic pub /nevil/text_command std_msgs/msg/String "data: 'Hello Nevil'"

# Monitor response
ros2 topic echo /nevil/text_response
```

#### Integration Test Script
```bash
#!/bin/bash
# integration_test.sh

echo "Starting Nevil system..."
./nevil launch nevil_bringup physical_robot.launch.py &
LAUNCH_PID=$!

sleep 10

echo "Checking node status..."
if ros2 node list | grep -q "ai_interface"; then
    echo "✅ AI Interface: Running"
else
    echo "❌ AI Interface: Failed"
fi

if ros2 node list | grep -q "rt_sensor"; then
    echo "✅ Real-time Sensor: Running"
else
    echo "❌ Real-time Sensor: Failed"
fi

echo "Shutting down..."
kill $LAUNCH_PID
```

---

## Summary

This guide provides comprehensive instructions for launching, monitoring, and shutting down the Nevil-picar v2.0 system. The system is designed to be robust and user-friendly, with multiple fallback options for both startup and shutdown procedures.

For additional support, refer to:
- [Build Status Documentation](BUILD_STATUS.md)
- [AI Interface Architecture](ai_interface_architecture.md)
- [Troubleshooting Guide](Nevil2.0%20tech/7_troubleshooting.md)