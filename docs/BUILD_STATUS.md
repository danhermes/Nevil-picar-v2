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
| nevil_interfaces_ai | rclpy, rclcpp, std_msgs, geometry_msgs, action_msgs, nevil_interfaces, nevil_navigation, nevil_core | ✅ Fixed | Updated setup.py to install launch files |
| nevil_simulation | rclcpp, rclpy, std_msgs, sensor_msgs, geometry_msgs, visualization_msgs, tf2, tf2_ros, nevil_interfaces, nevil_core, nevil_navigation, nevil_perception, nevil_realtime | ✅ Built | None |
| nevil_bringup | rclpy, std_msgs, launch, launch_ros, nevil_core, nevil_interfaces, nevil_interfaces_ai, nevil_navigation, nevil_perception, nevil_realtime, nevil_simulation, nevil_testing | ⚠️ Pending | Ready to build after nevil_interfaces_ai fix |
| nevil_testing | rclcpp, rclpy, std_msgs, geometry_msgs, sensor_msgs, nevil_interfaces, nevil_interfaces_ai, nevil_core, nevil_navigation, nevil_perception, nevil_realtime, nevil_simulation | ✅ Built | None |

## Issues and Solutions

### 1. nevil_interfaces_ai Launch Files Not Installed

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
4. Build the nevil_bringup package now that nevil_interfaces_ai is fixed.
5. Test the complete system with all packages built and installed.