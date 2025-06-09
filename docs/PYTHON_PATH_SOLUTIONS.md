# Python Path Solutions for Nevil-picar v2.0

This document records solutions for common Python path issues encountered in the Nevil-picar v2.0 project.

## Table of Contents
- [Launch File Path Issues](#launch-file-path-issues)
- [Integration Test Path Issues](#integration-test-path-issues)
- [Unit Test Path Issues](#unit-test-path-issues)
- [General Best Practices](#general-best-practices)

## Launch File Path Issues

### Issue: Node Execution Failures in Launch Files

**Error Message:**
```
[speech_recognition_node-1] python3: can't open file '/home/dan/nevil/install/nevil_interfaces_ai/share/nevil_interfaces_ai/nevil_interfaces_ai/speech_recognition_node.py': [Errno 2] No such file or directory
```

**Root Cause:**
The launch file was looking for Python files in the wrong location:
- **Looking in**: `/home/dan/nevil/install/nevil_interfaces_ai/share/nevil_interfaces_ai/nevil_interfaces_ai/`
- **Should be in**: `/home/dan/nevil/install/nevil_interfaces_ai/lib/nevil_interfaces_ai/` or `/home/dan/nevil/install/nevil_interfaces_ai/lib/python3/site-packages/`

**Solution:**

1. **Fix the launch file paths**:
   
   In `src/nevil_interfaces_ai/launch/speech_interface.launch.py`, change:
   ```python
   # Incorrect
   cmd=['python3', os.path.join(src_dir, 'nevil_interfaces_ai', 'speech_recognition_node.py')]
   ```
   
   To:
   ```python
   # Correct
   cmd=['python3', os.path.join(src_dir, 'scripts', 'speech_recognition_node.py')]
   ```

2. **Rebuild with symlink install**:
   ```bash
   cd ~/nevil
   rm -rf build/nevil_interfaces_ai  # Clean build directory first if needed
   colcon build --packages-select nevil_interfaces_ai --symlink-install
   ```

3. **Verify CMakeLists.txt configuration**:
   
   Ensure the CMakeLists.txt has the proper configuration for installing executables:
   ```cmake
   # Install executables
   install(PROGRAMS
     scripts/dialog_manager_node.py
     scripts/speech_recognition_node.py
     scripts/speech_synthesis_node.py
     scripts/text_command_processor.py
     DESTINATION lib/${PROJECT_NAME}
   )
   
   # Install Python modules
   install(DIRECTORY
     ${PROJECT_NAME}/
     DESTINATION lib/python3/site-packages/${PROJECT_NAME}
     PATTERN "__pycache__" EXCLUDE
   )
   ```

4. **Use entry points in setup.py**:
  
  Instead of directly executing Python scripts, use entry points defined in setup.py:
  
  ```python
  # In setup.py
  setup(
      # ...
      entry_points={
          'console_scripts': [
              'dialog_manager_node = nevil_interfaces_ai.dialog_manager_node:main',
              'speech_recognition_node = nevil_interfaces_ai.speech_recognition_node:main',
              'speech_synthesis_node = nevil_interfaces_ai.speech_synthesis_node:main',
          ],
      },
  )
  ```
  
  Then use `ros2 run` to execute the nodes:
  
  ```bash
  ros2 run nevil_interfaces_ai dialog_manager_node
  ros2 run nevil_interfaces_ai speech_recognition_node
  ros2 run nevil_interfaces_ai speech_synthesis_node
  ```

4. **Modify wrapper scripts** to add Python path:
   
   For scripts in the `scripts/` directory that import from the package's Python modules, add the following code at the top of the file:
   
   ```python
   import os
   import sys
   
   # Add the source directory to PYTHONPATH
   src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
   sys.path.insert(0, src_dir)
   ```
   
   This ensures that the script can find the package's Python modules when executed directly.

## Integration Test Path Issues

**Error Message:**
```
FileNotFoundError: [Errno 2] No such file or directory: '/home/dan/nevil/install/nevil_testing/share/nevil_testing/config/test_config.yaml'
```

**Root Cause:**
Integration tests are looking for configuration files in installed package locations rather than source directories.

**Solution:**
Create a wrapper script similar to the unit test wrapper that sets up the correct paths:

1. **Create a modified integration test script** that uses the source directory structure:
   ```python
   # Modified integration test script
   import os
   import sys
   
   # Add the source directory to PYTHONPATH
   src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
   sys.path.insert(0, src_dir)
   
   # Use the config file from the source directory
   config_file = os.path.join(src_dir, 'nevil_testing', 'config', 'test_config.yaml')
   ```

2. **Use relative imports** in test files to ensure proper module resolution.

## Unit Test Path Issues

**Error Message:**
```
ModuleNotFoundError: No module named 'nevil_testing'
```

**Root Cause:**
Python path not properly set up for testing from source.

**Solution:**
1. **Add the source directory to PYTHONPATH**:
   ```python
   import os
   import sys
   
   # Add the source directory to PYTHONPATH
   src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
   sys.path.insert(0, src_dir)
   ```

2. **Create wrapper scripts** for tests to ensure proper imports.

3. **Use relative imports** in test files:
   ```python
   # Instead of
   from nevil_testing.test_utils import TestBase
   
   # Use
   from ...test_utils import TestBase
   ```

## General Best Practices

1. **Use symlink install** when building packages:
   ```bash
   colcon build --symlink-install
   ```

2. **Verify file paths** in launch files and test scripts.

3. **Use the nevil script** to run nodes directly:
   ```bash
   ./nevil run nevil_interfaces_ai dialog_manager_node
   ```

4. **Clean build directories** when encountering symlink issues:
   ```bash
   rm -rf build/[package_name]
   ```

5. **Use absolute paths** when possible to avoid path resolution issues.

## Launch File Command Execution Issues

### Issue: Shell Commands in ExecuteProcess

**Error Message:**
```
FileNotFoundError: [Errno 2] No such file or directory: 'cd'
```

**Root Cause:**
When using `ExecuteProcess` in a launch file with shell commands like `cd`, the command fails because these are shell built-in commands, not executable programs. The `cmd` parameter in `ExecuteProcess` expects each element to be an executable or argument.

**Solutions:**

1. **Use shell=True parameter**:
  
  Modify the `ExecuteProcess` action to use the shell:
  ```python
  # Instead of:
  ExecuteProcess(
      cmd=['cd', src_dir, '&&', 'python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_recognition_node'],
      name='speech_recognition_node',
      output='screen'
  )
  
  # Use:
  ExecuteProcess(
      cmd=['python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_recognition_node'],
      cwd=src_dir,  # Set the working directory
      name='speech_recognition_node',
      output='screen'
  )
  ```

2. **Use PYTHONPATH environment variable**:
  
  Set the PYTHONPATH environment variable instead of changing directories:
  ```python
  import os
  
  # Set environment variables
  env = os.environ.copy()
  env['PYTHONPATH'] = src_dir + ':' + env.get('PYTHONPATH', '')
  
  ExecuteProcess(
      cmd=['python3', '-m', 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_recognition_node'],
      name='speech_recognition_node',
      output='screen',
      env=env
  )
  ```

3. **Use a wrapper script**:
  
  Create a wrapper shell script that sets up the environment and runs the Python module:
  ```bash
  #!/bin/bash
  # run_module.sh
  cd $1
  python3 -m $2
  ```
  
  Then in the launch file:
  ```python
  ExecuteProcess(
      cmd=['/path/to/run_module.sh', src_dir, 'src.nevil_interfaces_ai.nevil_interfaces_ai.speech_recognition_node'],
      name='speech_recognition_node',
      output='screen'
  )
  ```
  
  ## Comprehensive Path Resolution Strategy
  
  After extensive testing and analysis, we've identified several patterns of path issues in the Nevil-picar-v2 project. Here's a comprehensive strategy to address these issues:
  
  ### 1. ROS2 Package Structure Understanding
  
  The ROS2 package structure is fundamentally different from standard Python projects:
  
  ```
  workspace/
  ├── src/                  # Source packages
  │   ├── package1/
  │   │   ├── package1/     # Python module
  │   │   ├── setup.py      # Package setup
  │   │   └── CMakeLists.txt
  │   └── package2/
  ├── build/                # Build artifacts
  │   ├── package1/
  │   └── package2/
  └── install/              # Installed packages
      ├── package1/
      │   ├── lib/
      │   │   └── package1/ # Executables
      │   └── share/
      │       └── package1/ # Launch files, etc.
      └── package2/
  ```
  
  Understanding this structure is crucial for resolving path issues.
  
  ### 2. Standardized Approach for Different Scenarios
  
  #### Development Mode (Running from Source)
  
  When running nodes directly from source:
  
  1. **Use the direct_speech_interface.launch.py approach**:
     ```python
     # Get paths to Python files
     speech_recognition_path = os.path.join(src_dir, 'nevil_interfaces_ai', 'speech_recognition_node.py')
     
     # Execute directly
     ExecuteProcess(
         cmd=['python3', speech_recognition_path],
         name='speech_recognition_node',
         output='screen'
     )
     ```
  
  2. **Set PYTHONPATH environment variable**:
     ```python
     env = os.environ.copy()
     env['PYTHONPATH'] = f"{src_dir}:{env.get('PYTHONPATH', '')}"
     ```
  
  3. **Use relative imports in Python files**:
     ```python
     try:
         from nevil_interfaces_ai.audio_hardware_interface import AudioHardwareInterface
     except ImportError:
         # Try relative import if package import fails
         from .audio_hardware_interface import AudioHardwareInterface
     ```
  
  #### Deployment Mode (Running from Installed Packages)
  
  When running nodes from installed packages:
  
  1. **Use the speech_interface.launch.py approach**:
     ```python
     # Use Node action with entry points
     Node(
         package='nevil_interfaces_ai',
         executable='speech_recognition_node',
         name='speech_recognition_node',
         output='screen'
     )
     ```
  
  2. **Define entry points in setup.py**:
     ```python
     entry_points={
         'console_scripts': [
             'speech_recognition_node = nevil_interfaces_ai.speech_recognition_node:main',
             'speech_synthesis_node = nevil_interfaces_ai.speech_synthesis_node:main',
             'dialog_manager_node = nevil_interfaces_ai.dialog_manager_node:main',
         ],
     }
     ```
  
  3. **Include all necessary files in setup.py**:
     ```python
     data_files=[
         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
         ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
         (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
     ]
     ```
  
  ### 3. Wrapper Scripts for Different Scenarios
  
  We've created several wrapper scripts to handle different scenarios:
  
  1. **nevil_direct.sh**: Runs nodes directly from source without requiring installation
  2. **nevil_installed.sh**: Runs nodes from installed packages
  3. **rebuild_nevil_interfaces_ai.sh**: Rebuilds the package with the updated setup.py file
  
  These scripts provide flexibility for different development and deployment scenarios.
  
  ### 4. Troubleshooting Path Issues
  
  When encountering path issues:
  
  1. **Check import statements**:
     - Use try/except blocks to handle both package and relative imports
     - Print debug information about import paths
  
  2. **Verify PYTHONPATH**:
     - Print the PYTHONPATH environment variable
     - Ensure it includes both source and install directories
  
  3. **Check file existence**:
     - Verify that files exist at the expected paths
     - Print debug information about file paths
  
  4. **Inspect launch file approach**:
     - Ensure the launch file is using the appropriate approach for the scenario
     - Check that environment variables are properly set
  
  5. **Rebuild packages**:
     - Use `colcon build --symlink-install` to rebuild packages
     - Clean build directories if necessary
  
  By following these guidelines, you can effectively resolve path issues in the Nevil-picar-v2 project.
  
  ## Comprehensive Path Resolution Guide
  
  For a more detailed and comprehensive guide to resolving path issues in the Nevil-picar-v2 project, please refer to the [Path Resolution Guide](PATH_RESOLUTION_GUIDE.md). This guide provides:
  
  1. **In-depth analysis of all path issues**
  2. **Detailed solution strategies for each issue**
  3. **Code examples for different scenarios**
  4. **Best practices for path resolution**
  5. **Troubleshooting common issues**
  6. **Development vs. deployment considerations**
  7. **Package configuration guidelines**
  
  The Path Resolution Guide builds upon the solutions presented in this document and provides a more structured and comprehensive approach to resolving path issues in the Nevil-picar-v2 project.