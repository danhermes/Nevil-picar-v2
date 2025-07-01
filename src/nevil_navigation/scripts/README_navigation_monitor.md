# Nevil Navigation Monitor

A real-time navigation monitoring dashboard for the Nevil robot that provides comprehensive visibility into navigation topics, critical system errors, and node health status.

## 🚨 Critical Features

The monitor prioritizes **fatal errors, failed processes, and node failures** with:

- **Fatal Error Detection**: Monitors ROS2 logs for crashes, segfaults, and fatal errors
- **Node Health Monitoring**: Tracks critical navigation nodes and alerts when missing
- **System Resource Monitoring**: Watches CPU, memory usage, and zombie processes
- **Topic Timeout Detection**: Alerts when critical topics stop publishing
- **Real-time Alerting**: Color-coded alerts with severity levels (Fatal/Error/Warning)

## 📡 Monitored Topics

- `/cmd_vel` - Movement commands (linear.x, angular.z)
- `/goal_pose` - Navigation goals (position x,y,z)
- `/system_mode` - Active/standby status
- `/nevil/action_command` - AI action commands
- `/planned_path` - Path planning output

## 🏥 Monitored Nodes

- `navigation_node` - Main navigation controller
- `ai_interface_node` - AI command interface
- `dialog_manager_node` - Dialog management
- `hardware_bridge_node` - Hardware interface
- `rt_motor_control_node` - Real-time motor control

## 🚀 Quick Start

### Basic Usage
```bash
# Start basic monitoring
./src/nevil_navigation/scripts/start_monitor.sh

# With logging
./src/nevil_navigation/scripts/start_monitor.sh --log-file /tmp/nav_monitor.log

# Fast refresh rate
./src/nevil_navigation/scripts/start_monitor.sh --refresh-rate 50
```

### Direct Python Usage
```bash
# Basic monitoring
python3 src/nevil_navigation/scripts/navigation_monitor.py

# With options
python3 src/nevil_navigation/scripts/navigation_monitor.py \
    --log-file /tmp/nav_monitor.log \
    --refresh-rate 100
```

## 📊 Display Modes

### Rich Terminal (Recommended)
When the `rich` library is available, the monitor displays:
- **Critical Alerts Panel** (top priority) - Shows fatal errors and warnings
- **Navigation Topics Panel** - Real-time topic monitoring
- **Node Health Panel** - Status of critical nodes

### Basic Terminal
Fallback mode with text-based display showing:
- Critical alerts section (prioritized at top)
- Navigation topics table
- Node health status
- System statistics

## 🎛️ Controls

- **Ctrl+C**: Clean shutdown
- **r**: Reset message counters
- **p**: Pause/resume monitoring

## 🚨 Alert Severity Levels

| Level | Icon | Description | Action Required |
|-------|------|-------------|-----------------|
| **FATAL** | 💀 | System crashes, segfaults | Immediate attention |
| **ERROR** | 🔴 | Failed operations, exceptions | Investigation needed |
| **WARNING** | 🟡 | Performance issues, timeouts | Monitor closely |

## 📝 Logging

Enable logging to capture all events for later analysis:

```bash
./start_monitor.sh --log-file /tmp/nevil_navigation.log
```

Log format: `timestamp,topic,value,count` for topics and `ERROR,timestamp,node,message` for errors.

## 🔧 Installation Requirements

### Required
- ROS2 Humble
- Python 3.8+
- `rclpy`
- `psutil`

### Optional (Enhanced Display)
```bash
pip3 install rich
```

## 🏗️ Architecture

```
NavigationMonitor (ROS2 Node)
├── Topic Subscribers
│   ├── /cmd_vel (Twist)
│   ├── /goal_pose (PoseStamped)
│   ├── /system_mode (String)
│   ├── /nevil/action_command (AICommand)
│   ├── /planned_path (Path)
│   └── /rosout (Log) - For error detection
├── Health Monitors
│   ├── Node Health Checker
│   ├── System Resource Monitor
│   └── Topic Timeout Detector
└── Display Engine
    ├── Rich Terminal (Multi-panel)
    └── Basic Terminal (Text-based)
```

## 🚨 Critical Error Detection

The monitor actively watches for:

### Fatal Errors
- Segmentation faults
- System crashes
- Process aborts
- Memory corruption

### Node Failures
- Missing critical nodes
- Node startup failures
- Communication timeouts

### System Issues
- High memory usage (>90%)
- High CPU usage (>95%)
- Zombie processes
- Topic timeouts (>30s)

### ROS2 Errors
- Failed service calls
- Message parsing errors
- QoS policy violations

## 📈 Performance

- **Refresh Rate**: Configurable (default: 100ms)
- **Memory Usage**: ~10-20MB
- **CPU Impact**: <1% on typical systems
- **Alert History**: Limited to 50 recent alerts

## 🔍 Troubleshooting

### Monitor Won't Start
```bash
# Check ROS2 environment
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check dependencies
python3 -c "import rclpy, psutil"
```

### No Topics Visible
```bash
# Verify topics are publishing
ros2 topic list
ros2 topic echo /cmd_vel --once
```

### Rich Display Issues
```bash
# Install rich library
pip3 install rich

# Or use basic mode
python3 navigation_monitor.py  # Will auto-fallback
```

## 🤝 Integration

### With Launch Files
```python
# Add to your launch file
Node(
    package='nevil_navigation',
    executable='navigation_monitor.py',
    name='navigation_monitor',
    parameters=[
        {'log_file': '/tmp/nav_monitor.log'},
        {'refresh_rate': 100}
    ]
)
```

### With System Monitoring
The monitor integrates with system monitoring tools and can export metrics for:
- Prometheus/Grafana dashboards
- Log aggregation systems
- Alert management platforms

## 📋 Example Output

```
🚨 CRITICAL ALERTS 🚨
Severity   | Type           | Message                                    | Count | Time
💀 FATAL   | FATAL_ERROR    | navigation_node: Segmentation fault       | 1     | 12:34:56
🔴 ERROR   | NODE_MISSING   | Critical node ai_interface_node not found | 3     | 12:34:55

📡 NAVIGATION TOPICS 📡
Topic                | Latest Value           | Count | Last Update | Status
/cmd_vel            | lin:0.25 ang:0.0      | 1234  | 12:34:56   | 🟢 Active
/goal_pose          | x:2.5 y:1.2 z:0.0     | 5     | 12:34:55   | 🟢 Active
/system_mode        | active                 | 1     | 12:34:50   | 🟢 Active
/nevil/action_command| forward               | 23    | 12:34:54   | 🟢 Active
/planned_path       | 12 waypoints          | 8     | 12:34:53   | 🟢 Active

🏥 NODE HEALTH 🏥
navigation_node              | 🟢 ACTIVE
ai_interface_node           | 🔴 MISSING
dialog_manager_node         | 🟢 ACTIVE
```

## 📚 Related Documentation

- [Navigation Node Documentation](../README.md)
- [System Architecture](../../docs/architecture.md)
- [Troubleshooting Guide](../../docs/troubleshooting.md)

---

**Author**: Nevil Navigation Team  
**Version**: 1.0  
**Last Updated**: January 2025