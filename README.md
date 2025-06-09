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

#### 2. Install ROS2 on Raspberry Pi (ARM64)

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

#### 2. Run the System

Run the system in simulation mode:

```bash
ros2 launch nevil_simulation nevil_system_with_simulation.launch.py
```

Run the system on physical hardware:

```bash
ros2 launch nevil_bringup full_system.launch.py
```

## Nevil Bringup

Integration package for the Nevil-picar v2.0 project. This package provides launch files and configuration for different system configurations.

## Overview

The `nevil_bringup` package serves as the main integration point for the Nevil-picar v2.0 system. It provides:

- Launch files for different system configurations
- Configuration files for various scenarios
- Command-line interface for system management
- Integration testing tools

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

## License

[License information]

## Acknowledgments

- SunFounder for the PiCar-X platform
- ROS2 community
- PREEMPT-RT developers
- OpenAI for API access
- Contributors to the ARCHES-PiCar-X project