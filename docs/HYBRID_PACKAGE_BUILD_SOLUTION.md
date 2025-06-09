# Nevil-picar-v2 Build Solution

## Issue Summary

When attempting to launch the full system using `ros2 launch nevil_bringup full_system.launch.py`, we encountered two issues:

1. **Node constructor missing 'executable' parameter**:
   ```
   TypeError: Node.__init__() missing 1 required keyword-only argument: 'executable'
   ```

2. **Executable not found in libexec directory**:
   ```
   executable 'system_monitor_wrapper' not found on the libexec directory '/home/dan/Nevil-picar-v2/install/nevil_bringup/lib/nevil_bringup'
   ```

## Root Causes

1. In the `full_system.launch.py` file, the Node constructor was using the `cmd` parameter instead of the required `executable` parameter. This is likely due to changes in the ROS2 API.

2. The `system_monitor_wrapper` script was not being installed to the expected location because it was not included in the list of programs to install in the `CMakeLists.txt` file.

## Solutions

1. **Fixed Node constructor parameter**:
   - Changed `cmd` parameter to `executable` parameter in the Node constructor in `src/nevil_bringup/launch/full_system.launch.py`.

2. **Added script to installation list**:
   - Added the `system_monitor_wrapper` script to the list of programs to install in `src/nevil_bringup/CMakeLists.txt`.

## Steps to Reproduce the Fix

1. Modify the `full_system.launch.py` file:
   ```python
   # Change from
   system_monitor_node = Node(
       package='nevil_bringup',
       name='system_monitor',
       output='screen',
       parameters=[
           {'config_file': config_file}
       ],
       cmd=['python3', system_monitor_script]
   )

   # To
   system_monitor_node = Node(
       package='nevil_bringup',
       name='system_monitor',
       output='screen',
       parameters=[
           {'config_file': config_file}
       ],
       executable='system_monitor_wrapper'
   )
   ```

2. Modify the `CMakeLists.txt` file:
   ```cmake
   # Change from
   install(PROGRAMS
     scripts/nevil_cli.py
     scripts/system_monitor.py
     DESTINATION lib/${PROJECT_NAME}
   )

   # To
   install(PROGRAMS
     scripts/nevil_cli.py
     scripts/system_monitor.py
     scripts/system_monitor_wrapper
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

3. Rebuild the package:
   ```bash
   colcon build --packages-select nevil_bringup
   ```

4. Source the workspace setup file:
   ```bash
   source install/setup.bash
   ```

5. Launch the full system:
   ```bash
   ros2 launch nevil_bringup full_system.launch.py
   ```

## Lessons Learned

1. **ROS2 Node API Changes**: The ROS2 Node constructor requires an `executable` parameter, not a `cmd` parameter. This is important to remember when working with ROS2 launch files.

2. **Script Installation**: All scripts that are referenced in launch files must be properly installed to the expected location. This is done by adding them to the `install(PROGRAMS ...)` section in the `CMakeLists.txt` file.

3. **Build Process**: Always make sure to rebuild the package and source the workspace setup file after making changes to the launch files or CMakeLists.txt.

## Future Recommendations

1. **Update Documentation**: Update the documentation to reflect the correct usage of the Node constructor in ROS2 launch files.

2. **Add Tests**: Add tests to verify that all required scripts are properly installed and that the launch files are correctly configured.

3. **Standardize Script Installation**: Establish a standard process for adding new scripts to ensure they are properly installed.

4. **Automated Checks**: Implement automated checks to verify that all scripts referenced in launch files are included in the installation list.

## Additional Issue: nevil_cli Executable Not Found

When attempting to run the `nevil_cli` executable using `ros2 run nevil_bringup nevil_cli --help`, we encountered the following error:

```
No executable found
```

### Root Cause

1. There were two implementations of the `nevil_cli` script:
   - `nevil_bringup/nevil_cli.py` (simple implementation)
   - `scripts/nevil_cli.py` (comprehensive implementation with more features)

2. The entry point in `setup.py` was pointing to `nevil_bringup.nevil_cli:main`, but the script was being installed from `scripts/nevil_cli.py` to `lib/nevil_bringup/`.

3. ROS2 was unable to find the executable because the entry point didn't match the installed script.

### Solution

1. **Updated entry point in setup.py**:
   ```python
   # Change from
   'nevil_cli = nevil_bringup.nevil_cli:main',
   
   # To
   'nevil_cli = scripts.nevil_cli:main',
   ```

2. **Created a symbolic link in CMakeLists.txt**:
   ```cmake
   # Create symbolic link for nevil_cli
   install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nevil_cli.py
     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nevil_cli
   )")
   ```

### Steps to Reproduce the Fix

1. Update the entry point in `setup.py` to point to the correct script.
2. Add the symbolic link creation in `CMakeLists.txt`.
3. Rebuild the package:
   ```bash
   colcon build --packages-select nevil_bringup
   ```
4. Source the workspace setup file:
   ```bash
   source install/setup.bash
   ```
5. Test the command:
   ```bash
   ros2 run nevil_bringup nevil_cli --help
   ```

### Lessons Learned

1. **Entry Point Consistency**: Ensure that entry points in `setup.py` match the actual location of the scripts.
2. **Symbolic Links**: Creating symbolic links without the `.py` extension is a common practice in ROS2 packages to make scripts executable with `ros2 run`.
3. **Hybrid Build Systems**: When using both CMake and Python setuptools, ensure that the installation paths and entry points are consistent.