# Nevil-picar v2.0

A multi-threaded, real-time robotic system built on the PiCar-X platform that integrates ROS2 with PREEMPT-RT for deterministic performance.

## Overview

Nevil-picar v2.0 is a sophisticated robotic system that combines:

- **Multi-threaded ROS2 Architecture**: Enables true parallel processing across multiple nodes
- **PREEMPT-RT Integration**: Provides deterministic, low-latency performance for critical operations
- **Digital Twin Simulation**: Allows development and testing without physical hardware
- **Hybrid AI Processing**: Combines cloud-based and local AI models for robust operation
- **Multi-modal Interaction**: Supports voice, text, and visual interfaces

## Hardware Requirements

- Raspberry Pi 4/5 (8GB RAM recommended)
- PiCar-X platform
- Camera module
- Ultrasonic sensor
- Microphone/Speaker
- Optional IMU
- **Storage**: At least 20GB free disk space for building ROS2 from source

## Important Considerations for Raspberry Pi

### Disk Space Requirements
Building ROS2 from source requires significant disk space (at least 20GB free). Consider:
- Using a larger SD card (64GB+)
- Using external storage (USB drive)
- Cleaning up unnecessary packages before building: `sudo apt clean && sudo apt autoremove`

### Build Time
The build process takes several hours on a Raspberry Pi. The script is designed to be resumable, so you can continue from where it left off if interrupted.

### Resource Constraints
The build process is resource-intensive. The script uses a sequential executor to reduce memory usage, but you may still encounter resource limitations.

## Quick Start Guide

### Installation

#### 1. Clone the Repository

```bash
git clone https://github.com/username/nevil-picar-v2.git
cd nevil-picar-v2
```

#### 2. Install All PiCar Modules (Important)

Before installing ROS2, you must install all the required PiCar modules and configure the audio hardware. This is essential for the robot's speaker and hardware functionality.

**Make sure you are connected to the Internet and update your system:**

```bash
sudo apt update
sudo apt upgrade
```

**Note:** Python3 related packages must be installed if you are using the Lite version OS:

```bash
sudo apt install git python3-pip python3-setuptools python3-smbus
```

**Install robot-hat:**

```bash
cd ~/
git clone -b v2.0 https://github.com/sunfounder/robot-hat.git
cd robot-hat
sudo python3 setup.py install
```

**Download and install the vilib module:**

```bash
cd ~/
git clone -b picamera2 https://github.com/sunfounder/vilib.git
cd vilib
sudo python3 install.py
```

**Download and install the picar-x module:**

```bash
cd ~/
git clone -b v2.0 https://github.com/sunfounder/picar-x.git --depth 1
cd picar-x
sudo python3 setup.py install
```

This step will take a little while, so please be patient.

**Configure Audio Hardware (Critical for Robot Speaker):**

Finally, you need to run the script `i2samp.sh` to install the components required by the i2s amplifier, otherwise the picar-x will have no sound:

```bash
cd ~/picar-x
sudo bash i2samp.sh
sudo reboot
```

**Important:** This audio setup is required for Nevil v2.0's robot speaker to work. Without this configuration, audio will only work through HDMI or the Pi's 3.5mm jack, not the robot's built-in speaker.

For complete PiCar-X documentation, see:
- **PDF Guide**: [`docs/picar/docs-sunfounder-com-picar-x-en-latest.pdf`](docs/picar/docs-sunfounder-com-picar-x-en-latest.pdf)
- **Online Documentation**: [https://docs.sunfounder.com/projects/picar-x-v20/en/latest/](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/)

#### 3. Install ROS2 on Raspberry Pi (ARM64)

For Raspberry Pi and other ARM64 devices, we provide a specialized installation script that builds ROS2 from source:

```bash
# Make the script executable
chmod +x start.py

# Run the setup script
./start.py
```

This script will:
1. Detect your system architecture
2. Install necessary dependencies
3. For ARM64 devices, create a build script for ROS2
4. Set up the workspace structure

#### 3. Build ROS2 from Source (ARM64 only)

If you're on an ARM64 device (like Raspberry Pi), the script will create a build script. Run it to build ROS2 from source:

```bash
# Make the script executable
chmod +x build_ros2_arm64.sh

# Option 1: Build ROS2 from source directly (this will take several hours)
./build_ros2_arm64.sh

# Option 2: Run the build in the background (recommended for SSH sessions)
chmod +x run_build_in_background.sh
./run_build_in_background.sh
```

The background build option is recommended if you're connecting via SSH, as it allows the build to continue even if your connection drops. You can monitor the build progress with:

```bash
# Check the build status (shows progress, completed packages, etc.)
chmod +x check_build_status.sh
./check_build_status.sh

# View the build log in real-time
tail -f build_log.txt
```

The build process will automatically handle package failures by skipping problematic packages and continuing with the rest of the build. This ensures you get a working ROS2 installation even if some non-essential packages fail to build.

#### 4. Complete the Setup

After building ROS2, complete the setup:

```bash
# Make the script executable
chmod +x complete_setup.sh

# Complete the setup
./complete_setup.sh
```

### Building Nevil-picar-v2

We've created a comprehensive build system to handle all the dependencies and build issues for the Nevil-picar-v2 project. This system includes scripts to fix common issues and automate the build process.

#### 1. Run the Automated Build Script

```bash
# Make the script executable
chmod +x build/build_nevil_all.sh

# Run the build script
./build/build_nevil_all.sh
```

This script will:
1. Install all required dependencies
2. Fix CMakeLists.txt files in problematic packages
3. Create minimal implementations for missing files
4. Build the entire project
5. Fix issues with spaces in the project path

The build script supports the following command line options:

```bash
# Skip packages that have already been built (useful for resuming interrupted builds)
./build/build_nevil_all.sh --skip-existing

# Display help information
./build/build_nevil_all.sh --help
```

#### 2. Alternative: Step-by-Step Build

If you prefer to build the project step by step, you can run the individual scripts in the `build` directory:

```bash
# Install dependencies
./build/ROS2_humble/install_ros2_python_deps.sh
./build/ROS2_humble/install_numpy.sh
./build/ROS2_humble/fix_pyttsx3_dependency.sh
./build/ROS2/resolve_dependencies.sh

# Fix CMakeLists.txt files
./build/nevil/fix_all_cmake_files.sh

# Fix package-specific issues
./build/nevil/create_minimal_perception_package.sh
./build/nevil/fix_nevil_perception_camera.sh
# ... and other package-specific scripts

# Build the project
cd src
colcon build
cd ..

# Fix path space issues
./build/fix_path_space_issue.sh
```

For a complete list of available scripts and their purposes, see the [Build Summary](build/NEVIL_BUILD_SUMMARY.md) and [Build README](build/README.md).

### Usage

#### 1. Source the Environment

For future terminal sessions, source the environment:

```bash
# Source the ROS2 environment
source ~/ros2_humble/install/setup.bash

# Source the Nevil workspace
source ~/nevil-picar-v2/src/install/setup.bash
```

If your project path contains spaces, use one of the solutions provided by the `fix_path_space_issue.sh` script:

```bash
# Option 1: Use the symbolic link
cd ~/nevil && source src/install/setup.bash

# Option 2: Use the wrapper script
./nevil launch nevil_bringup full_system.launch.py

# Option 3: Use the shell alias (after restarting your shell)
nevil launch nevil_bringup full_system.launch.py
```

### Path Issues and Solutions

The Nevil-picar-v2 project has several path-related issues that can affect building, testing, and running the system. These issues stem from the complex interaction between ROS2 package structure, Python module resolution, and the dual directory structure of the project.

#### Common Path Issues

1. **Module Not Found Errors**: Python modules not being found due to PYTHONPATH issues
2. **Launch File Errors**: Executables not found in the expected directories
3. **Configuration File Not Found**: Configuration files not being found in the expected locations
4. **Dual Directory Structure Confusion**: Confusion between `/home/dan/Documents/Cursor Projects/Nevil-picar-v2` and `/home/dan/nevil`

#### Solutions

We've created several tools and documents to help resolve these issues:

1. **Wrapper Scripts**:
   - `nevil_direct.sh`: Runs nodes directly from source without requiring installation
   - `nevil_installed.sh`: Runs nodes from installed packages
   - `rebuild_nevil_interfaces_ai.sh`: Rebuilds specific packages with updated setup files

2. **Documentation**:
   - [Python Path Solutions](docs/PYTHON_PATH_SOLUTIONS.md): Solutions for common Python path issues
   - [Path Resolution Guide](docs/PATH_RESOLUTION_GUIDE.md): Comprehensive guide to resolving path issues
   - [Build Status](docs/BUILD_STATUS.md): Status of package builds and known issues

3. **Best Practices**:
   - Use `colcon build --symlink-install` to build packages
   - Include all necessary files in setup.py data_files
   - Use entry points in setup.py for executable scripts
   - Use consistent paths in all scripts and configuration files

For more detailed information, refer to the [Path Resolution Guide](docs/PATH_RESOLUTION_GUIDE.md).

#### 2. Run the System

Run the system in simulation mode:

```bash
ros2 launch nevil_simulation nevil_system_with_simulation.launch.py
```

Run the system on physical hardware:

```bash
ros2 launch nevil_bringup full_system.launch.py
```

## System Monitoring

### Navigation Monitor

The Navigation Monitor provides comprehensive real-time monitoring and critical error detection for the Nevil-picar v2.0 system:

#### Key Capabilities
- **Real-time Topic Monitoring**: Tracks all navigation-related topics with live updates
- **Critical Error Detection**: Monitors ROS2 logs for crashes, segfaults, and fatal errors
- **Node Health Monitoring**: Tracks critical navigation nodes and alerts when missing
- **System Resource Monitoring**: Watches CPU, memory usage, and zombie processes
- **Multi-level Alerting**: Color-coded alerts with severity levels (Fatal/Error/Warning)

#### Quick Start
```bash
# Start navigation monitoring
./src/nevil_navigation/scripts/start_monitor.sh

# With logging for analysis
./src/nevil_navigation/scripts/start_monitor.sh --log-file /tmp/nav_monitor.log
```

#### Monitored Components
- **Topics**: `/cmd_vel`, `/goal_pose`, `/system_mode`, `/nevil/action_command`, `/planned_path`
- **Nodes**: `navigation_node`, `ai_interface_node`, `dialog_manager_node`, `hardware_bridge_node`, `rt_motor_control_node`
- **System**: Memory usage, CPU usage, zombie processes, topic timeouts

For detailed documentation, see [Navigation Monitor Documentation](src/nevil_navigation/scripts/README_navigation_monitor.md).

## Nevil Bringup

Integration package for the Nevil-picar v2.0 project. This package provides launch files and configuration for different system configurations.

## Overview

The `nevil_bringup` package serves as the main integration point for the Nevil-picar v2.0 system. It provides:

- Launch files for different system configurations
- Configuration files for various scenarios
- Command-line interface for system management
- Integration testing tools

## Build Nevil (better than colcon as it copies in necessary stubs)
### ALL
./nevil build --cmake-clean-cache --event-handlers console_cohesion+
### One Package
./nevil build --packages-select nevil_navigation

## Launch Configurations

The following launch configurations are available:

1. **Full System Launch**: Starts all components of the Nevil-picar v2.0 system
   ```
   ./nevil launch nevil_bringup full_system.launch.py cd..
   ./nevil launch nevil_bringup full_system.launch.py
   cd ~/nevil/src && source install/setup.bash && python3 -c "from nevil_realtime.rt_hardware_interface import main; main()"
   ros2 launch nevil_bringup full_system.launch.py
   ```

2. **Minimal System Launch**: Starts only the core components
   ```
   ros2 launch nevil_bringup minimal_system.launch.py
   ```

3. **Simulation-only Launch**: Starts the system in simulation mode
   ```
   ros2 launch nevil_bringup simulation.launch.py
   ```

4. **Physical Robot Launch**: Starts the system on the physical robot
   ```
   ros2 launch nevil_bringup physical_robot.launch.py
   ```

5. **Development Mode Launch**: Starts the system with debugging tools
   ```
   ros2 launch nevil_bringup development.launch.py
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


## Documentation

For more detailed information, refer to the documentation:

- [Project Overview](docs/1_overview_project.md)
- [Installation and Setup](docs/2_installation_setup.md)
- [Core Concepts](docs/3_core_concepts.md)
- [User Guide](docs/4_user_guide.md)
- [API Reference](docs/5_api_reference.md)
- [Developer Guide](docs/6_developer_guide.md)
- [Troubleshooting](docs/7_troubleshooting.md)
- [Build System](build/README.md)
- [Build Summary](build/NEVIL_BUILD_SUMMARY.md)
- [Environment Variables](docs/BUILD_ENVIRONMENT_VARIABLES.md)
- [Build Status](docs/BUILD_STATUS.md)
- [Python Path Solutions](docs/PYTHON_PATH_SOLUTIONS.md)
- [Path Resolution Guide](docs/PATH_RESOLUTION_GUIDE.md)
- [Wrapper Script Template System](docs/WRAPPER_SCRIPT_TEMPLATE_SYSTEM.md)
- [Hardware Library Setup](docs/HARDWARE_LIBRARY_SETUP.md)
- [Testing Status](docs/tests/TESTING_STATUS.md)

## License

[License information]

## Acknowledgments

- SunFounder for the PiCar-X platform
- ROS2 community
- PREEMPT-RT developers
- OpenAI for API access
- Contributors to the ARCHES-PiCar-X project