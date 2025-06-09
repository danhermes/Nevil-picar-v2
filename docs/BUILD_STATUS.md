# Nevil-picar-v2 Build Status

This document tracks the build status of all packages in the Nevil-picar-v2 project, including dependencies, issues blocking builds, and recommendations for improving the build process.

## Package List and Build Status

| Package Name | Dependencies | Build Status | Issues |
|--------------|--------------|--------------|--------|
| nevil_interfaces | std_msgs, geometry_msgs, sensor_msgs, action_msgs | ✅ Built | None |
| nevil_core | rclcpp, rclpy, std_msgs, nevil_interfaces | ✅ Built | None |
| nevil_realtime | rclcpp, rclpy, std_msgs, sensor_msgs, nevil_interfaces, nevil_core | ✅ Built | None |
| nevil_navigation | rclcpp, std_msgs, geometry_msgs, nav_msgs, sensor_msgs, nevil_interfaces, nevil_core, nevil_realtime, action_msgs, rcl_interfaces, tf2, tf2_ros, tf2_geometry_msgs, tf2_msgs | ✅ Built | None |
| nevil_perception | rclcpp, rclpy, std_msgs, sensor_msgs, cv_bridge, image_transport, nevil_interfaces, nevil_core, opencv_python | ✅ Built | None |
| nevil_interfaces_ai | rclpy, rclcpp, std_msgs, geometry_msgs, action_msgs, nevil_interfaces, nevil_navigation, nevil_core | ✅ Built | Fixed Python package directory name collision |
| nevil_simulation | rclcpp, rclpy, std_msgs, sensor_msgs, geometry_msgs, visualization_msgs, tf2, tf2_ros, nevil_interfaces, nevil_core, nevil_navigation, nevil_perception, nevil_realtime | ✅ Built | None |
| nevil_bringup | rclpy, std_msgs, launch, launch_ros, nevil_core, nevil_interfaces, nevil_interfaces_ai, nevil_navigation, nevil_perception, nevil_realtime, nevil_simulation, nevil_testing | ✅ Built | Successfully built after nevil_interfaces_ai fix |
| nevil_testing | rclcpp, rclpy, std_msgs, geometry_msgs, sensor_msgs, nevil_interfaces, nevil_interfaces_ai, nevil_core, nevil_navigation, nevil_perception, nevil_realtime, nevil_simulation | ✅ Built | None |
| picarx | (external library) | ✅ Built | Fixed setup.py to handle missing README.md |

## Issues and Solutions

### 1. nevil_interfaces_ai CMake Target Name Collision

**Issue**: The nevil_interfaces_ai package failed to build due to duplicate CMake target errors when both generating interfaces using `rosidl_generate_interfaces` and installing a Python package with `ament_python_install_package(${PROJECT_NAME})`.

**Root Cause**: When the package name (`PROJECT_NAME`) matches both the interface generation namespace and Python package directory, the CMake/ament build system generates conflicting targets with the same name. This only affected this package because other packages did either interface generation **or** Python packaging, but not both.

**Solution**:
1. Renamed the internal Python package directory to avoid name collision with CMake project name:
   ```bash
   cd ~/Nevil-picar-v2/src/nevil_interfaces_ai
   mv nevil_interfaces_ai nevil_ai_pkg
   ```

2. Updated CMakeLists.txt to use the new directory name:
   ```cmake
   ament_python_install_package(nevil_ai_pkg)
   ```

3. Cleaned and rebuilt the workspace:
   ```bash
   cd ~/Nevil-picar-v2
   rm -rf build/ install/ log/
   colcon build
   ```

For a detailed technical explanation of this issue and solution, see [HYBRID_PACKAGE_BUILD_SOLUTION.md](HYBRID_PACKAGE_BUILD_SOLUTION.md).

### 2. nevil_interfaces_ai Launch Files Not Installed

**Issue**: The launch files for the nevil_interfaces_ai package were not being properly installed to the install directory. This was causing the `nevil launch nevil_interfaces_ai nevil_interfaces_ai.launch.py` command to fail.

**Root Cause**: The setup.py file didn't include the launch files in the data_files list.

**Solution**:
1. Created a workaround script `nevil_direct.sh` that uses the direct_speech_interface.launch.py file, which doesn't require the package to be installed.
2. Fixed the setup.py file to include the launch files in the data_files list:

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Include launch files
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*.launch.py'))),
    # Include scripts directory
    (os.path.join('share', package_name, 'scripts'),
     glob(os.path.join('scripts', '*.py'))),
],
```

3. Created a rebuild script `rebuild_nevil_interfaces_ai.sh` to rebuild the package with the updated setup.py file.
4. Created a modified script `nevil_installed.sh` that sources the local setup.bash file and uses the installed package.

### 2. Path Resolution in Launch Files

**Issue**: The launch files are using different approaches to find the Python files, which can lead to confusion and errors.

**Solution**:
1. Standardize the approach across all launch files.
2. Use the entry points defined in setup.py when possible, as this is the recommended ROS2 approach.
3. For development and testing, use the direct_speech_interface.launch.py approach, which doesn't require the package to be installed.

## Build Instructions

### Building the Workspace

To build the workspace properly, run the following commands:

```bash
# Option 1: Use the rebuild script for nevil_interfaces_ai only
./rebuild_nevil_interfaces_ai.sh

# Option 2: Build the entire workspace
colcon build --symlink-install
```

### Running the nevil_interfaces_ai Package

There are three ways to run the nevil_interfaces_ai package:

1. **Using the direct script (no installation required):**
   ```bash
   ./nevil_direct.sh
   ```

2. **Using the installed script (after building):**
   ```bash
   ./nevil_installed.sh
   ```

3. **Using the original nevil script (after fixing and building):**
   ```bash
   cd ~/nevil
   ./nevil launch nevil_interfaces_ai speech_interface.launch.py
   ```

## Next Steps

1. ✅ Fixed the setup.py file to properly install the launch files.
2. ✅ Created rebuild script to rebuild the package with the updated setup.py file.
3. ✅ Created alternative scripts to run the package in different ways.
4. ✅ Built the nevil_bringup package now that nevil_interfaces_ai is fixed.
5. ✅ Fixed the picarx build issue by modifying v1.0/setup.py to handle missing README.md.
6. ✅ Successfully built all packages with `colcon build --symlink-install`.
7. ✅ Fixed the nevil_interfaces_ai CMake target name collision by renaming the Python package directory.
8. ✅ Created detailed documentation on hybrid package build issues and solutions.
9. ✅ All packages now build successfully.
10. Test the complete system with all packages built and installed.
11. Consider implementing a more permanent solution to exclude non-ROS2 packages from the build process.
12. Apply the hybrid package naming pattern to any future packages that combine interface generation and Python code.

## Path Issues Analysis

The Nevil-picar-v2 project has numerous path issues that stem from several interconnected factors. Understanding these issues is crucial for maintaining and extending the project.

### Root Causes of Path Issues

1. **ROS2 Package Structure Complexity**
   - ROS2 uses a specific workspace model with `src`, `build`, and `install` directories
   - Packages need to be properly built and installed before they can be used
   - The build system (colcon) creates a specific directory structure in the install directory
   - Message, service, and action definitions generate code that must be properly referenced

2. **Multiple Python Module Resolution Approaches**
   - Direct imports from the source directory
   - Imports using the ROS2 package system
   - Relative imports within packages
   - Entry points defined in setup.py
   - Environment variable manipulation (PYTHONPATH)

3. **Inconsistent Launch File Strategies**
   - `speech_interface.launch.py`: Uses Node action with entry points
   - `direct_speech_interface.launch.py`: Uses ExecuteProcess with direct paths
   - `source_speech_interface.launch.py`: Uses ExecuteProcess with Python modules

4. **Dual Directory Structure**
   - `/home/dan/Documents/Cursor Projects/Nevil-picar-v2` (current workspace)
   - `/home/dan/nevil` (referenced in scripts)

5. **Incomplete Build/Install Process**
   - Launch files not properly installed
   - Entry points not properly registered
   - Generated message/service/action code not found

6. **Script Wrapper Complexity**
   - `nevil` script that sources setup.bash
   - `nevil_direct.sh` that uses direct paths
   - `nevil_installed.sh` that sources local setup.bash
   - `rebuild_nevil_interfaces_ai.sh` for rebuilding specific packages

7. **Mixed Development and Deployment Approaches**
   - Some files expect to run from source
   - Others expect to run from installed packages
   - Some try to accommodate both with complex path manipulation

### Recommended Path Resolution Strategy

To address these issues systematically, we recommend:

1. **Standardize the build and install process**
   - Use `colcon build --symlink-install` consistently for all packages
   - Ensure all packages have proper setup.py files with correct data_files entries
   - Use entry points in setup.py for all executable scripts

2. **Choose one primary approach for Python module resolution**
   - For development: Use the source directory with PYTHONPATH adjustments
   - For deployment: Use the installed packages with proper entry points

3. **Standardize launch file approach**
   - Use the ROS2 Node action with entry points for installed packages
   - Use ExecuteProcess with direct paths for development/testing

4. **Clarify directory structure**
   - Document the relationship between the two directory structures
   - Use consistent paths in all scripts and configuration files

5. **Complete the build/install process**
   - Ensure all packages are built and installed correctly
   - Verify that all necessary files are included in the install directory

6. **Simplify script wrappers**
   - Create a single script with options for different modes (development, deployment)
   - Document the purpose and usage of each script

7. **Separate development and deployment concerns**
    - Create separate launch files for development and deployment
    - Use environment variables to switch between modes

### Comprehensive Path Resolution Guide

For a comprehensive guide to resolving path issues in the Nevil-picar-v2 project, see the [Path Resolution Guide](PATH_RESOLUTION_GUIDE.md). This guide provides:

1. **Detailed explanation of all path issues**
2. **Step-by-step solutions for common problems**
3. **Best practices for path resolution**
4. **Code examples for different scenarios**
5. **Troubleshooting tips for specific errors**

## Implementation Status

| Solution | Status | Notes |
|----------|--------|-------|
| Fixed setup.py for nevil_interfaces_ai | ✅ Complete | Launch files now properly installed |
| Created wrapper scripts | ✅ Complete | nevil_direct.sh, nevil_installed.sh |
| Created rebuild script | ✅ Complete | rebuild_nevil_interfaces_ai.sh |
| Standardized launch file approach | ⚠️ In Progress | Multiple approaches still in use |
| Clarified directory structure | ✅ Complete | Documented in PATH_RESOLUTION_GUIDE.md |
| Simplified script wrappers | ⚠️ In Progress | Multiple scripts still in use |
| Separated development and deployment | ⚠️ In Progress | Working on standardizing approach |

## Build System Configuration

We've implemented the following solutions to address build issues:

### 1. Fixed External Library Issues

1. **picarx**: A hardware library for the PiCar-X platform
   - Modified `v1.0/setup.py` to handle missing README.md file:
   ```python
   # Read the contents of the README file if it exists, otherwise use a default description
   try:
       with open('README.md', encoding='utf-8') as f:
           long_description = f.read()
   except FileNotFoundError:
       long_description = 'Picarx gait Library for Raspberry Pi'
   ```

2. **numpy test examples**: Test examples in numpy that were causing Cython compilation errors
   - Created `COLCON_IGNORE` files in:
     - `nevil_venv/lib/python3.11/site-packages/numpy/_core/tests/examples/cython/`
     - `nevil_venv/lib/python3.11/site-packages/numpy/_core/tests/examples/limited_api/`

### 2. Alternative Approaches (Not Implemented)

We also explored these approaches, which could be useful in other scenarios:

1. **Using colcon.meta for Package Configuration**:
   ```json
   {
     "names": {
       "picarx": {
         "skip": true
       }
     }
   }
   ```

2. **Using COLCON_IGNORE files** in directories containing packages that should be excluded from the build.

3. **Using --packages-skip option** with colcon build:
   ```bash
   colcon build --symlink-install --packages-skip picarx
   ```

### 3. Build Commands

The workspace can now be built with:

```bash
colcon build --symlink-install
```

This successfully builds all packages, including the previously problematic picarx package.


## CMake/Python Hybrid Package Issues

### Understanding the Problem

When creating ROS2 packages that combine both interface generation (using `rosidl_generate_interfaces`) and Python package installation (using `ament_python_install_package`), name collisions can occur if the same name is used for both the CMake project and the Python package directory.

### Symptoms of the Problem

The build fails with errors like:

```
CMake Error: add_custom_target cannot create target "ament_cmake_python_copy_nevil_interfaces_ai" because another target with the same name already exists.
```

This happens because:

1. `rosidl_generate_interfaces(${PROJECT_NAME} ...)` creates CMake targets using the project name
2. `ament_python_install_package(${PROJECT_NAME})` also creates CMake targets using the project name
3. When both are used in the same package, they conflict

### Solution Pattern

To avoid this issue, use a different name for the Python package directory:

1. Name your Python package directory differently from your CMake project name
2. Update CMakeLists.txt to use the new directory name:
   ```cmake
   ament_python_install_package(your_python_pkg_name)
   ```

### Example Implementation

For the nevil_interfaces_ai package:

```cmake
# In CMakeLists.txt
project(nevil_interfaces_ai)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AICommand.msg"
  # ... other interface files
)

# Install Python package with a different name
ament_python_install_package(nevil_ai_pkg)
```

With the Python code organized in a directory named `nevil_ai_pkg` instead of `nevil_interfaces_ai`.

### Best Practices for Hybrid Packages

1. Keep interface definitions and Python implementation in separate packages when possible
2. If combining them, use distinct names for the CMake project and Python package
3. Document the relationship between the CMake project and Python package names
4. Consider using a naming convention like `<package>_py` for Python packages