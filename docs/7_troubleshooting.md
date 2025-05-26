# Nevil-picar v2.0: Troubleshooting Guide

This guide provides solutions to common problems and issues that may arise when using the Nevil-picar v2.0 system. It covers hardware issues, software issues, and system-specific problems.

## Table of Contents

- [Diagnostic Tools](#diagnostic-tools)
- [Hardware Issues](#hardware-issues)
- [Software Issues](#software-issues)
- [ROS2 Issues](#ros2-issues)
- [Real-Time Issues](#real-time-issues)
- [Navigation Issues](#navigation-issues)
- [Vision Issues](#vision-issues)
- [Voice Interface Issues](#voice-interface-issues)
- [AI Processing Issues](#ai-processing-issues)
- [Simulation Issues](#simulation-issues)
- [Common Error Messages](#common-error-messages)
- [Performance Issues](#performance-issues)
- [Recovery Procedures](#recovery-procedures)
- [Getting Help](#getting-help)

## Diagnostic Tools

Nevil-picar v2.0 includes several diagnostic tools to help identify and resolve issues:

### System Diagnostics

Run the system diagnostics tool to check the overall health of the system:

```bash
ros2 run nevil_core system_diagnostics
```

This tool checks:
- Node status
- Topic activity
- Parameter values
- Resource usage
- Hardware connections

### Hardware Diagnostics

Run the hardware diagnostics tool to check the hardware components:

```bash
ros2 run nevil_core hardware_diagnostics
```

This tool checks:
- Motor functionality
- Sensor readings
- Camera operation
- Microphone and speaker functionality
- Battery status

### Network Diagnostics

Run the network diagnostics tool to check network connectivity:

```bash
ros2 run nevil_core network_diagnostics
```

This tool checks:
- Wi-Fi connectivity
- API access
- ROS2 DDS communication
- Network latency

### Performance Monitoring

Monitor system performance in real-time:

```bash
ros2 run nevil_core performance_monitor
```

This tool displays:
- CPU usage
- Memory usage
- Disk usage
- Node execution times
- Message latency

### Log Viewer

View system logs to identify issues:

```bash
ros2 run nevil_core log_viewer [--level LEVEL] [--node NODE]
```

Options:
- `--level LEVEL`: Log level (debug, info, warn, error)
- `--node NODE`: Filter logs by node name
## Hardware Issues

### Motors Not Responding

**Symptoms:**
- Robot doesn't move when commanded
- No motor noise when movement commands are issued
- Motor status shows errors

**Possible Causes:**
1. Motor controller not powered
2. Motor connections loose or disconnected
3. Motor controller failure
4. GPIO pin configuration incorrect

**Solutions:**
1. Check power connections to the motor controller
   ```bash
   ros2 run nevil_core check_power_status
   ```

2. Verify motor connections
   ```bash
   ros2 run nevil_core check_motor_connections
   ```

3. Test motors directly
   ```bash
   ros2 run nevil_navigation test_motors
   ```

4. Check GPIO pin configuration in `hardware_config.yaml`
   ```yaml
   hardware:
     motors:
       left_pin: 27
       right_pin: 17
   ```

5. Reset the motor controller
   ```bash
   ros2 service call /system_manager/reset_hardware std_srvs/srv/Trigger "{}"
   ```

### Ultrasonic Sensor Not Working

**Symptoms:**
- No distance readings
- Constant zero or maximum distance readings
- Obstacle avoidance not functioning

**Possible Causes:**
1. Sensor connections loose or disconnected
2. GPIO pin configuration incorrect
3. Sensor failure
4. Software configuration issues

**Solutions:**
1. Check sensor connections
   ```bash
   ros2 run nevil_core check_sensor_connections
   ```

2. Verify GPIO pin configuration in `hardware_config.yaml`
   ```yaml
   hardware:
     ultrasonic:
       trigger_pin: 23
       echo_pin: 24
   ```

3. Test the sensor directly
   ```bash
   ros2 run nevil_perception test_ultrasonic
   ```

4. Calibrate the sensor
   ```bash
   ros2 run nevil_perception calibrate_ultrasonic
   ```

5. Check if the sensor data is being published
   ```bash
   ros2 topic echo /ultrasonic_data
   ```

### Camera Not Working

**Symptoms:**
- No camera feed
- Black or corrupted images
- Vision processing not functioning

**Possible Causes:**
1. Camera not connected properly
2. Camera not enabled in Raspberry Pi configuration
3. Camera driver issues
4. Software configuration issues

**Solutions:**
1. Check camera connection
   - Ensure the ribbon cable is properly seated
   - Check for damage to the cable

2. Enable the camera in Raspberry Pi configuration
   ```bash
   sudo raspi-config
   # Navigate to "Interface Options" > "Camera" and select "Enable"
   sudo reboot
   ```

3. Test the camera directly
   ```bash
   ros2 run nevil_perception test_camera
   ```

4. Check if camera data is being published
   ```bash
   ros2 topic echo /camera/image_raw
   ```

5. Verify camera configuration in `hardware_config.yaml`
   ```yaml
   hardware:
     camera:
       resolution:
         width: 640
         height: 480
       framerate: 30
   ```

### Microphone or Speaker Not Working

**Symptoms:**
- No audio input or output
- Voice commands not recognized
- Robot doesn't speak

**Possible Causes:**
1. Audio devices not connected properly
2. Audio device not recognized by system
3. Volume settings incorrect
4. Software configuration issues

**Solutions:**
1. Check audio device connections
   - Ensure USB devices are properly connected
   - Try different USB ports

2. List audio devices
   ```bash
   arecord -l  # List recording devices
   aplay -l    # List playback devices
   ```
## Software Issues

### System Fails to Start

**Symptoms:**
- Nodes don't start
- Error messages during startup
- System hangs during initialization

**Possible Causes:**
1. Missing dependencies
2. Configuration errors
3. Hardware initialization failures
4. ROS2 environment issues

**Solutions:**
1. Check for missing dependencies
   ```bash
   cd ~/nevil_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Verify configuration files
   - Check for syntax errors in YAML files
   - Ensure all required parameters are present

3. Check system logs
   ```bash
   ros2 run nevil_core log_viewer --level error
   ```

4. Verify ROS2 environment
   ```bash
   printenv | grep ROS
   ```

5. Try starting with minimal configuration
   ```bash
   ros2 launch nevil_core minimal.launch.py
   ```

### Node Crashes

**Symptoms:**
- Node terminates unexpectedly
- Error messages in logs
- Functionality stops working

**Possible Causes:**
1. Software bugs
2. Resource limitations
3. Hardware failures
4. Configuration issues

**Solutions:**
1. Check node logs
   ```bash
   ros2 run nevil_core log_viewer --node crashed_node_name
   ```

2. Monitor resource usage
   ```bash
   ros2 run nevil_core performance_monitor
   ```

3. Restart the node
   ```bash
   ros2 service call /system_manager/restart_node nevil_interfaces/srv/RestartNode "{node_name: 'crashed_node_name'}"
   ```

4. Run the node with debug output
   ```bash
   ros2 run --prefix 'gdb -ex run --args' nevil_package crashed_node_name
   ```

5. Check for hardware issues related to the node's functionality

### Configuration Issues

**Symptoms:**
- Unexpected behavior
- Error messages about missing or invalid parameters
- Features not working as expected

**Possible Causes:**
1. Incorrect parameter values
2. Missing parameters
3. YAML syntax errors
4. File permission issues

**Solutions:**
1. Verify parameter values
   ```bash
   ros2 param list
   ros2 param get /node_name parameter_name
   ```

2. Check configuration file syntax
   ```bash
   python3 -c "import yaml; yaml.safe_load(open('path/to/config.yaml'))"
   ```

3. Reset to default configuration
   ```bash
   ros2 service call /system_manager/reset_configuration std_srvs/srv/Trigger "{}"
   ```

4. Check file permissions
   ```bash
   ls -la path/to/config/directory
   ```

5. Update parameters at runtime
   ```bash
   ros2 param set /node_name parameter_name value
   ```

### Software Update Issues

**Symptoms:**
- Update process fails
- System unstable after update
- New features not available

**Possible Causes:**
## ROS2 Issues

### Node Communication Problems

**Symptoms:**
- Nodes not receiving messages
- Service calls failing
- Action goals not being processed

**Possible Causes:**
1. DDS configuration issues
2. Network configuration problems
3. Node namespace issues
4. QoS incompatibilities

**Solutions:**
1. Check DDS discovery
   ```bash
   ros2 daemon status
   ros2 topic list
   ros2 node list
   ```

2. Verify network configuration
   ```bash
   ros2 doctor --report
   ```

3. Check for namespace issues
   ```bash
   ros2 node info /node_name
   ```

4. Test communication directly
   ```bash
   # Publisher
   ros2 topic pub /test_topic std_msgs/msg/String "data: 'test'" -1
   
   # Subscriber
   ros2 topic echo /test_topic
   ```

5. Set the RMW implementation explicitly
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

### Parameter Issues

**Symptoms:**
- Parameters not taking effect
- Parameter-related error messages
- Default values being used instead of configured values

**Possible Causes:**
1. Parameter files not loaded
2. Parameter namespace issues
3. Type conversion problems
4. Parameter declaration issues

**Solutions:**
1. Check if parameters are loaded
   ```bash
   ros2 param list
   ros2 param get /node_name parameter_name
   ```

2. Verify parameter file syntax
   ```bash
   cat path/to/params.yaml
   ```

3. Set parameters manually
   ```bash
   ros2 param set /node_name parameter_name value
   ```

4. Check node logs for parameter-related messages
   ```bash
   ros2 run nevil_core log_viewer --node node_name
   ```

5. Restart the node with explicit parameter file
   ```bash
   ros2 run nevil_package node_name --ros-args --params-file path/to/params.yaml
   ```

### Launch File Issues

**Symptoms:**
- Nodes not starting from launch file
- Launch file errors
- Unexpected node configuration

**Possible Causes:**
1. Launch file syntax errors
2. Missing dependencies
3. Path issues
4. Parameter passing problems

**Solutions:**
1. Check launch file syntax
   ```bash
   python3 path/to/launch_file.py
   ```

2. Run with verbose output
   ```bash
   ros2 launch --verbose nevil_package launch_file.py
   ```

3. Check for missing dependencies
   ```bash
   ros2 pkg list | grep required_package
   ```

4. Verify file paths in the launch file
   ```bash
   ros2 pkg prefix nevil_package
   ```

5. Test launching nodes individually
   ```bash
   ros2 run nevil_package node_name
   ```
## Real-Time Issues

### High Latency

**Symptoms:**
- Delayed response to commands
- Jerky movement
- Missed deadlines in real-time nodes

**Possible Causes:**
1. PREEMPT-RT kernel not running
2. Priority configuration issues
3. Resource contention
4. Blocking operations in real-time code

**Solutions:**
1. Verify PREEMPT-RT kernel is running
   ```bash
   uname -a  # Should show "PREEMPT RT"
   ```

2. Check real-time scheduling configuration
   ```bash
   cat /proc/sys/kernel/sched_rt_runtime_us  # Should be -1
   ```

3. Monitor latency
   ```bash
   sudo cyclictest -l1000000 -m -Sp90 -i200 -h400 -q
   ```

4. Check thread priorities
   ```bash
   ps -eLo pid,rtprio,comm | grep ros2
   ```

5. Set CPU affinity for critical threads
   ```bash
   sudo taskset -c 3 ros2 run nevil_realtime rt_motor_control_node
   ```

6. Isolate a CPU core for real-time tasks
   ```bash
   # Add to /boot/cmdline.txt
   isolcpus=3 nohz_full=3 rcu_nocbs=3
   ```

### Priority Inversion

**Symptoms:**
- High-priority tasks blocked by lower-priority tasks
- Unpredictable execution times
- Sporadic latency spikes

**Possible Causes:**
1. Shared resources without priority inheritance
2. Mutex locks in real-time code
3. System calls in real-time code
4. Inappropriate priority assignments

**Solutions:**
1. Use priority inheritance mutexes
   ```cpp
   pthread_mutexattr_t attr;
   pthread_mutexattr_init(&attr);
   pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
   pthread_mutex_init(&mutex, &attr);
   ```

2. Minimize critical sections
   - Keep locked regions as short as possible
   - Avoid system calls in critical sections

3. Use lock-free algorithms where possible
   - Atomic operations
   - Wait-free data structures

4. Review priority assignments
   ```bash
   ros2 run nevil_realtime rt_priority_analyzer
   ```

### Memory Management Issues

**Symptoms:**
- Latency spikes during memory allocation
- Out-of-memory errors
- Page faults in real-time code

**Possible Causes:**
1. Dynamic memory allocation in real-time code
2. Memory fragmentation
3. Swapping
4. Large memory footprint

**Solutions:**
1. Pre-allocate memory
   - Allocate all needed memory during initialization
   - Use memory pools for dynamic allocations

## Getting Help

If you've tried the troubleshooting steps in this guide and are still experiencing issues, there are several resources available to help:

### Documentation

- Review the [API Reference](5_api_reference.md) for detailed information on system interfaces
- Check the [Developer Guide](6_developer_guide.md) for information on extending and modifying the system
- Consult the [Core Concepts](3_core_concepts.md) document for a better understanding of the system architecture

### Community Resources

- **GitHub Issues**: Check existing issues or create a new one on the project's GitHub repository
- **ROS2 Community**: The ROS2 community has extensive resources and forums for troubleshooting
  - [ROS Discourse](https://discourse.ros.org/)
  - [ROS Answers](https://answers.ros.org/)
  - [ROS2 Documentation](https://docs.ros.org/en/humble/)

### Diagnostic Reports

When seeking help, it's useful to provide diagnostic information:

1. Generate a system report:
   ```bash
   ros2 run nevil_core generate_system_report
   ```

2. Include relevant logs:
   ```bash
   ros2 run nevil_core log_saver --output ~/nevil_logs.txt
   ```

3. Capture the system state:
   ```bash
   ros2 run nevil_core capture_system_state
   ```

### Contact Information

- **Project Maintainers**: [contact information]
- **Technical Support**: [contact information]
- **Community Chat**: [link to Discord/Slack/etc.]

## Conclusion

This troubleshooting guide covers the most common issues you might encounter when working with Nevil-picar v2.0. By following the diagnostic procedures and solutions provided, you should be able to resolve most problems.

Remember that Nevil-picar v2.0 is a complex system with many interacting components. When troubleshooting, it's important to isolate the problem by testing components individually and systematically working through potential causes.

If you discover new issues or better solutions, consider contributing to this guide to help other users of the system.
2. Lock memory to prevent paging
   ```cpp
   #include <sys/mman.h>
   mlockall(MCL_CURRENT | MCL_FUTURE);
   ```

3. Monitor memory usage
   ```bash
   ros2 run nevil_realtime rt_memory_monitor
   ```

4. Disable swapping
   ```bash
   sudo swapoff -a
   ```

5. Use real-time memory allocators
   - TLSF (Two-Level Segregated Fit)
   - jemalloc with appropriate configuration
1. Incomplete update
2. Dependency conflicts
3. Configuration incompatibilities
4. Build errors

**Solutions:**
1. Clean and rebuild the workspace
   ```bash
   cd ~/nevil_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

2. Update dependencies
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Check for configuration changes
   - Compare configuration files with examples
   - Update configuration files as needed

4. Roll back to previous version if necessary
   ```bash
   cd ~/nevil_ws/src/nevil-picar-v2
   git checkout previous_version_tag
   cd ~/nevil_ws
   colcon build
   source install/setup.bash
   ```

3. Test the microphone
   ```bash
   ros2 run nevil_interfaces_ai test_microphone
   ```

4. Test the speaker
   ```bash
   ros2 run nevil_interfaces_ai test_speaker
   ```

5. Verify audio configuration in `ai_config.yaml`
   ```yaml
   ai:
     voice:
       tts_provider: "openai"
       stt_provider: "openai"
   ```

6. Check audio device permissions
   ```bash
   sudo usermod -a -G audio $USER
   sudo reboot
   ```

### Battery Issues

**Symptoms:**
- Robot powers off unexpectedly
- Reduced operating time
- Battery status shows low voltage

**Possible Causes:**
1. Battery not charged
2. Battery connections loose
3. Battery degradation
4. Power consumption too high

**Solutions:**
1. Charge the battery
   - Connect to charger and wait for full charge
   - Check charging indicator lights

2. Check battery connections
   - Ensure connections are secure
   - Look for corrosion or damage

3. Check battery status
   ```bash
   ros2 topic echo /battery_status
   ```

4. Monitor power consumption
   ```bash
   ros2 run nevil_core power_monitor
   ```

5. Implement power-saving measures
   ```bash
   ros2 service call /system_manager/set_power_mode std_msgs/msg/String "{data: 'power_saving'}"
   ```