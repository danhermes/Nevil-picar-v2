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