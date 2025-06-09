# Nevil 2.0 Directory Structure and Build System Reference

## Table of Contents
- [Project Overview](#project-overview)
- [Symbolic Link Structure](#symbolic-link-structure)
- [Source Directory Structure](#source-directory-structure)
- [Install Directory Structure](#install-directory-structure)
- [Build Artifacts Organization](#build-artifacts-organization)
- [Node Structure and Wrapper Scripts](#node-structure-and-wrapper-scripts)
- [Symlink Install Behavior](#symlink-install-behavior)
- [Package Map and Node Relationships](#package-map-and-node-relationships)
- [Running Nodes](#running-nodes)
- [Common Issues](#common-issues)

## Project Overview

Nevil 2.0 is a ROS2-based robotics project organized into multiple packages, each responsible for specific functionality. The project follows a modular architecture with the following key components:

- **nevil_bringup**: System startup and configuration
- **nevil_core**: Core functionality and central coordination
- **nevil_interfaces**: Common message and service definitions
- **nevil_interfaces_ai**: AI-related interfaces and nodes
- **nevil_navigation**: Navigation and motion planning
- **nevil_perception**: Sensors and perception systems
- **nevil_realtime**: Real-time control systems
- **nevil_simulation**: Simulation environments
- **nevil_testing**: Testing utilities and test nodes

## Symbolic Link Structure

- There is a symlink between `~/nevil` and `/home/dan/Documents/Cursor Projects/Nevil-picar-v2`
- These are NOT separate folders or builds - they point to the same location
- When the script sources `~/nevil/src/install/setup.bash`, it's actually sourcing from the Cursor Projects directory
- This symlink structure allows for convenient access to the project from different locations

## Source Directory Structure

The source code is organized in a hierarchical structure following ROS2 conventions with some Nevil-specific customizations:

```
/home/dan/Documents/Cursor Projects/Nevil-picar-v2/
├── src/                              # Main source directory
│   ├── nevil_run.sh                  # Main execution script
│   ├── test_dialog_manager_node.py   # Test script
│   ├── install/                      # Built packages (created by colcon)
│   ├── build/                        # Build artifacts (created by colcon)
│   │   ├── COLCON_IGNORE             # Prevents nested builds
│   │   ├── nevil_bringup/            # Build artifacts for each package
│   │   ├── nevil_core/
│   │   ├── nevil_interfaces/
│   │   ├── nevil_interfaces_ai/      # Example build directory
│   │   │   ├── cmake_args.last       # Last used CMake arguments
│   │   │   ├── CMakeCache.txt        # CMake cache
│   │   │   ├── Makefile              # Generated makefile
│   │   │   ├── colcon_build.rc       # Colcon build result
│   │   │   ├── colcon_command_prefix_build.sh  # Build command prefix
│   │   │   ├── install_manifest.txt  # List of installed files
│   │   │   ├── libnevil_interfaces_ai__*.so    # Generated libraries
│   │   │   ├── rosidl_*__arguments.json        # Code generation arguments
│   │   │   ├── ament_cmake_*/        # Ament CMake artifacts
│   │   │   ├── CMakeFiles/           # CMake build files
│   │   │   ├── nevil_interfaces_ai__py/        # Python bindings
│   │   │   ├── rosidl_adapter/       # Message adaptation artifacts
│   │   │   ├── rosidl_generator_c/   # C code generation artifacts
│   │   │   ├── rosidl_generator_cpp/ # C++ code generation artifacts
│   │   │   ├── rosidl_generator_py/  # Python code generation artifacts
│   │   │   └── rosidl_typesupport_*/ # Type support artifacts
│   │   ├── nevil_navigation/
│   │   ├── nevil_perception/
│   │   ├── nevil_realtime/
│   │   ├── nevil_simulation/
│   │   └── nevil_testing/
│   ├── log/                          # Build and runtime logs
│   ├── nevil_bringup/                # System startup package
│   ├── nevil_core/                   # Core functionality package
│   ├── nevil_interfaces/             # Common interfaces package
│   ├── nevil_interfaces_ai/          # AI interfaces package
│   │   ├── CMakeLists.txt            # Build configuration
│   │   ├── package.xml               # Package metadata
│   │   ├── setup.py                  # Python package setup
│   │   ├── setup.cfg                 # Python package configuration
│   │   ├── nevil_interfaces_ai/      # Python module directory
│   │   │   ├── __init__.py
│   │   │   ├── dialog_manager_node.py
│   │   │   ├── speech_recognition_node.py
│   │   │   └── speech_synthesis_node.py
│   │   ├── scripts/                  # Executable wrapper scripts
│   │   │   ├── dialog_manager_node.py
│   │   │   ├── speech_recognition_node.py
│   │   │   └── speech_synthesis_node.py
│   │   ├── launch/                   # Launch files
│   │   │   ├── speech_interface.launch.py
│   │   │   └── direct_speech_interface.launch.py
│   │   └── msg/                      # Message definitions
│   │       ├── AICommand.msg
│   │       ├── AIStatus.msg
│   │       └── Audio.msg
│   ├── nevil_navigation/             # Navigation package
│   ├── nevil_perception/             # Perception package
│   ├── nevil_realtime/               # Real-time control package
│   ├── nevil_simulation/             # Simulation package
│   └── nevil_testing/                # Testing utilities package
```

Each package follows a similar structure with package-specific files and directories.

## Install Directory Structure

After building with `colcon build`, the install directory contains the built packages with the following structure:

```
src/install/
├── setup.bash                        # Main setup script
├── setup.sh                          # Shell-specific setup scripts
├── setup.zsh
├── setup.ps1
├── local_setup.bash                  # Local setup scripts
├── local_setup.sh
├── local_setup.zsh
├── local_setup.ps1
├── _local_setup_util_ps1.py          # Setup utilities
├── _local_setup_util_sh.py
├── nevil_bringup/                    # Installed packages
├── nevil_core/
├── nevil_interfaces/
├── nevil_interfaces_ai/              # Example installed package
│   ├── include/                      # C++ headers (if any)
│   ├── lib/                          # Libraries and executables
│   │   ├── libnevil_interfaces_ai__rosidl_generator_c.so
│   │   ├── libnevil_interfaces_ai__rosidl_generator_py.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_c.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_cpp.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_fastrtps_c.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_fastrtps_cpp.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_introspection_c.so
│   │   ├── libnevil_interfaces_ai__rosidl_typesupport_introspection_cpp.so
│   │   ├── nevil_interfaces_ai/      # Node executables
│   │   │   ├── ai_interface_node.py
│   │   │   ├── dialog_manager_node.py
│   │   │   ├── speech_recognition_node.py
│   │   │   ├── speech_synthesis_node.py
│   │   │   └── text_command_processor.py
│   │   └── python3/                  # Python modules
│   │       └── site-packages/
│   │           ├── __init__.py
│   │           ├── audio_hardware_interface.py
│   │           ├── dialog_manager_node.py
│   │           ├── speech_recognition_node.py
│   │           ├── speech_synthesis_node.py
│   │           ├── text_command_processor.py
│   │           └── examples/
│   ├── local/                        # Local resources
│   └── share/                        # Shared resources
│       ├── ament_index/              # Package index
│       ├── colcon-core/              # Colcon metadata
│       └── nevil_interfaces_ai/      # Package-specific resources
│           ├── cmake/                # CMake files
│           ├── environment/          # Environment hooks
│           ├── hook/                 # Setup hooks
│           ├── launch/               # Launch files
│           │   ├── nevil_interfaces_ai_with_simulation.launch.py
│           │   ├── nevil_interfaces_ai.launch.py
│           │   └── speech_interface.launch.py
│           ├── msg/                  # Message definitions
│           │   ├── AICommand.idl
│           │   ├── AICommand.msg
│           │   ├── AIStatus.idl
│           │   └── AIStatus.msg
│           ├── scripts/              # Scripts
│           ├── srv/                  # Service definitions
│           ├── local_setup.bash      # Package-specific setup
│           ├── local_setup.dsv
│           ├── local_setup.sh
│           ├── local_setup.zsh
│           ├── package.bash
│           ├── package.dsv
│           ├── package.sh
│           ├── package.xml
│           └── package.zsh
├── nevil_navigation/
├── nevil_perception/
├── nevil_realtime/
├── nevil_simulation/
└── nevil_testing/
```

## Build Artifacts Organization

Nevil 2.0 follows the ROS2 build system with some customizations:

### Build Directory Structure

The `src/build/` directory contains intermediate build artifacts created during the compilation process:

- **CMake Files**: Configuration and cache files used by CMake
  - `CMakeCache.txt`: Cache of CMake variables
  - `cmake_args.last`: Last used CMake arguments
  - `Makefile`: Generated makefile for building the package
  - `CMakeFiles/`: Directory containing CMake-generated files

- **Colcon Files**: Files used by the colcon build system
  - `colcon_build.rc`: Build result status
  - `colcon_command_prefix_build.sh`: Command prefix for the build
  - `COLCON_IGNORE`: File that prevents nested builds

- **Generated Code**: Files created by the ROS2 code generation system
  - `rosidl_adapter/`: Message adaptation artifacts
  - `rosidl_generator_c/`: Generated C code for messages
  - `rosidl_generator_cpp/`: Generated C++ code for messages
  - `rosidl_generator_py/`: Generated Python code for messages
  - `rosidl_typesupport_*/`: Type support for different middleware implementations

- **Compiled Libraries**: Shared libraries created during compilation
  - `libnevil_interfaces_ai__rosidl_generator_c.so`: C message generation library
  - `libnevil_interfaces_ai__rosidl_typesupport_*.so`: Type support libraries

The build directory is used during the compilation process but is not needed at runtime. The final artifacts are copied or symlinked to the `install/` directory.

### Standard ROS2 Directories

- **include/**: C++ header files
- **lib/**: Compiled libraries and executables
  - **lib/[package_name]/**: Node executables
  - **lib/python3/site-packages/**: Python modules
- **share/**: Shared resources
  - **share/ament_index/**: Package index for discovery
  - **share/[package_name]/**: Package-specific resources
    - **cmake/**: CMake configuration
    - **launch/**: Launch files
    - **msg/**: Message definitions
    - **srv/**: Service definitions

### Nevil-Specific Customizations

- **Dual Node Locations**: Nevil places node executables in both:
  - `lib/[package_name]/` (ROS2 standard location)
  - `lib/python3/site-packages/` (when using Python modules)

- **Wrapper Scripts**: Nevil uses wrapper scripts in `scripts/` that import and call the main function from the implementation files

## Node Structure and Wrapper Scripts

Nevil uses a specific pattern for ROS2 nodes:

1. **Implementation Files**: `/src/nevil_[component_name]/nevil_[component_name]/[node_name].py`
   - Contains the actual node implementation
   - Defines a `main()` function for entry point

2. **Wrapper Scripts**: `/src/nevil_[component_name]/scripts/[node_name].py`
   - Simple entry points that import and call the main function
   - Example:
     ```python
     #!/usr/bin/env python3
     
     import rclpy
     from nevil_interfaces_ai.dialog_manager_node import main
     
     if __name__ == '__main__':
         main()
     ```

3. **CMakeLists.txt Configuration**:
   - Must install wrapper scripts to the `lib/` directory:
     ```cmake
     install(PROGRAMS
       scripts/dialog_manager_node.py
       scripts/speech_recognition_node.py
       scripts/speech_synthesis_node.py
       DESTINATION lib/${PROJECT_NAME}
     )
     ```

## Symlink Install Behavior

When using `colcon build --symlink-install`:

1. **Python Files**: Instead of copying Python files, symlinks are created in the install directory pointing to the source files
   - This allows for editing Python files without rebuilding
   - Located in `install/[package_name]/lib/python3/site-packages/`

2. **Launch Files**: Symlinks are created for launch files
   - Located in `install/[package_name]/share/[package_name]/launch/`

3. **Message Definitions**: NOT symlinked - these require code generation
   - Located in `install/[package_name]/share/[package_name]/msg/`

4. **Wrapper Scripts**: Symlinked to the source scripts
   - Located in `install/[package_name]/lib/[package_name]/`

5. **Resource Files**: Symlinked to source files
   - Located in appropriate directories under `install/[package_name]/share/[package_name]/`

## Package Map and Node Relationships

Nevil 2.0 consists of the following packages and their primary nodes:

### nevil_bringup
- **system_bringup_node**: Coordinates system startup
- **configuration_node**: Manages system configuration

### nevil_core
- **central_control_node**: Main control node
- **state_manager_node**: Manages system state

### nevil_interfaces
- Contains common message and service definitions
- No nodes, only interfaces

### nevil_interfaces_ai
- **dialog_manager_node**: Manages dialog interactions
- **speech_recognition_node**: Handles speech recognition
- **speech_synthesis_node**: Generates speech output
- **text_command_processor**: Processes text commands

### nevil_navigation
- **path_planning_node**: Plans robot paths
- **motion_control_node**: Controls robot motion
- **localization_node**: Determines robot position

### nevil_perception
- **camera_node**: Processes camera input
- **sensor_fusion_node**: Combines sensor data

### nevil_realtime
- **motor_control_node**: Real-time motor control
- **sensor_processing_node**: Real-time sensor processing

### nevil_simulation
- **simulation_node**: Runs robot simulation
- **virtual_environment_node**: Simulates environment

### nevil_testing
- **test_harness_node**: Runs automated tests
- **performance_monitor_node**: Monitors system performance

## Executable Locations and Launch System

### Where Executables Are Located

Yes, all ROS2 executables that can be launched by the ROS2 launch system or called by Nevil 2.0 must be in the `src/install/` directory. Specifically:

1. **Python Node Executables**:
   - Located in `src/install/[package_name]/lib/[package_name]/`
   - Example: `src/install/nevil_interfaces_ai/lib/nevil_interfaces_ai/dialog_manager_node.py`

2. **C++ Node Executables**:
   - Located in `src/install/[package_name]/lib/[package_name]/`
   - Example: `src/install/nevil_navigation/lib/nevil_navigation/path_planning_node`

3. **Python Modules**:
   - Located in `src/install/[package_name]/lib/python3/site-packages/`
   - Example: `src/install/nevil_interfaces_ai/lib/python3/site-packages/dialog_manager_node.py`

The `src/build/` directory contains only intermediate build artifacts and is not used at runtime.

### How the Launch System Finds Executables

When you run a ROS2 launch file or use the Nevil run script:

1. The system sources the setup script: `source src/install/setup.bash`
2. This sets up the ROS2 environment variables, including:
   - `PATH`: Updated to include `src/install/[package_name]/lib/[package_name]/`
   - `PYTHONPATH`: Updated to include `src/install/[package_name]/lib/python3/site-packages/`
   - `LD_LIBRARY_PATH`: Updated to include `src/install/[package_name]/lib/`

3. When a launch file references a node:
   ```python
   Node(
       package='nevil_interfaces_ai',
       executable='dialog_manager_node.py',
       name='dialog_manager'
   )
   ```
   
   The launch system:
   - Looks for the executable in `src/install/nevil_interfaces_ai/lib/nevil_interfaces_ai/dialog_manager_node.py`
   - Uses the environment variables set by the setup script to find it

### Why Use the Install Directory

The install directory provides a clean, organized runtime environment:

1. Only contains files needed at runtime (not build artifacts)
2. Has a consistent structure across all packages
3. Properly sets up Python module paths and library paths
4. Works with the ROS2 package discovery system

### Nevil Run Script

The `./nevil` script simplifies this process:

```bash
./nevil run nevil_interfaces_ai dialog_manager_node
```

This script:
1. Sources the correct setup script
2. Resolves the package and node name to the correct executable path
3. Runs the executable with the proper environment

## Running Nodes

- Always use the `./nevil` script to run nodes: `./nevil run [package_name] [node_name]`
- This ensures proper environment setup and path resolution
- Example: `./nevil run nevil_interfaces_ai dialog_manager_node`

## Common Issues

### "No executable found" errors
These often occur when:
1. The node isn't properly registered in setup.py
2. The package hasn't been rebuilt after changes
3. The wrapper scripts aren't properly installed to the `lib/` directory
4. There's a mismatch between the entry point in setup.py and the actual file location

### Build failures with message definitions
1. Missing message definitions in CMakeLists.txt
2. Incorrect import paths for message types
3. Missing dependencies in package.xml

### Python module not found
1. Package not properly installed
2. Missing or incorrect dependencies
3. Incorrect import paths

### Launch file errors
1. Node executable not found
2. Incorrect node namespace
3. Missing or incorrect parameters
