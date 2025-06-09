# Path Resolution Guide for Nevil-picar-v2

This guide provides a comprehensive overview of path resolution issues in the Nevil-picar-v2 project and strategies to resolve them.

## Table of Contents

1. [Understanding the Problem](#understanding-the-problem)
2. [Root Causes](#root-causes)
3. [Solution Strategies](#solution-strategies)
4. [Development vs. Deployment](#development-vs-deployment)
5. [Wrapper Scripts](#wrapper-scripts)
6. [Package Configuration](#package-configuration)
7. [Troubleshooting Common Issues](#troubleshooting-common-issues)
8. [Best Practices](#best-practices)

## Understanding the Problem

The Nevil-picar-v2 project faces several path resolution challenges that can cause errors when building, testing, and running the system. These issues manifest as:

- **Module Not Found Errors**: Python modules not being found during import
- **Launch File Errors**: Executables not found in the expected directories
- **Configuration File Not Found**: Configuration files not being found in the expected locations
- **Dual Directory Structure Confusion**: Confusion between development and deployment directories

## Root Causes

### 1. ROS2 Package Structure Complexity

ROS2 uses a workspace model with `src`, `build`, and `install` directories. This structure can be confusing, especially when:

- Running nodes from source vs. installed packages
- Accessing resources (config files, launch files) from different locations
- Mixing development and deployment environments

### 2. Multiple Python Module Resolution Approaches

The project uses several approaches to Python module resolution:

- **Package Imports**: `from nevil_interfaces_ai import module`
- **Relative Imports**: `from ..module import function`
- **Direct Imports**: `import module`
- **Fallback Imports**: Trying multiple import strategies with try/except

### 3. Inconsistent Launch File Strategies

Launch files use different approaches to starting nodes:

- **Node Action**: Using the `Node` action with package and executable names
- **ExecuteProcess**: Using `ExecuteProcess` with direct paths to Python scripts
- **Mixed Approaches**: Some launch files use a mix of both approaches

### 4. Dual Directory Structure

The project operates in two directory structures:

- **Development Directory**: `/home/dan/Documents/Cursor Projects/Nevil-picar-v2`
- **Deployment Directory**: `/home/dan/nevil`

This dual structure creates confusion when scripts reference absolute paths or when environment variables point to different locations.

### 5. Incomplete Build/Install Process

Issues in the build and install process can lead to:

- Missing files in the installed package
- Incorrect paths in installed scripts
- Incomplete environment setup

### 6. Script Wrapper Complexity

The project uses several wrapper scripts that:

- Set up environment variables
- Source ROS2 and workspace setup files
- Execute commands with specific arguments

These wrappers can introduce additional complexity and path resolution issues.

### 7. Mixed Development and Deployment Approaches

The project mixes:

- Running from source (development)
- Running from installed packages (deployment)
- Testing in different environments

## Solution Strategies

### 1. Consistent Package Structure

Ensure all packages follow a consistent structure:

```
package_name/
├── package_name/
│   ├── __init__.py
│   └── module.py
├── launch/
│   └── package.launch.py
├── scripts/
│   └── executable.py
├── config/
│   └── config.yaml
├── package.xml
└── setup.py
```

### 2. Proper setup.py Configuration

Ensure `setup.py` includes all necessary files:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
],
entry_points={
    'console_scripts': [
        'node_name = package_name.module:main',
    ],
},
```

### 3. Consistent Import Strategy

Use a consistent import strategy across all modules:

```python
# Preferred approach
from package_name.module import function

# Fallback approach (if needed)
try:
    from package_name.module import function
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from module import function
```

### 4. Launch File Best Practices

Use the `Node` action with entry points for launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='node_name',
            output='screen',
        )
    ])
```

### 5. Environment Variable Management

Use environment variables consistently:

```bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Set up workspace environment
source ~/nevil/install/setup.bash

# Set up Python path if needed
export PYTHONPATH=$PYTHONPATH:~/nevil/src/package_name
```

## Development vs. Deployment

### Development Mode

When developing and testing:

1. Run nodes directly from source:
   ```bash
   python3 ~/nevil/src/package_name/scripts/node.py
   ```

2. Use development-specific launch files:
   ```bash
   ros2 launch package_name development.launch.py
   ```

3. Set up the environment for development:
   ```bash
   source ~/nevil/src/development_setup.bash
   ```

### Deployment Mode

When deploying:

1. Build and install packages:
   ```bash
   cd ~/nevil
   colcon build --symlink-install
   ```

2. Run nodes from installed packages:
   ```bash
   source ~/nevil/install/setup.bash
   ros2 run package_name node_name
   ```

3. Use deployment-specific launch files:
   ```bash
   ros2 launch package_name deployment.launch.py
   ```

## Wrapper Scripts

The project includes several wrapper scripts to simplify environment setup and execution:

### nevil_direct.sh

Runs nodes directly from source without requiring installation:

```bash
#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Set up Python path
export PYTHONPATH=$PYTHONPATH:~/nevil/src/package_name

# Execute the command
python3 ~/nevil/src/package_name/scripts/$1.py "${@:2}"
```

### nevil_installed.sh

Runs nodes from installed packages:

```bash
#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Set up workspace environment
source ~/nevil/install/setup.bash

# Execute the command
ros2 run $1 $2 "${@:3}"
```

### rebuild_nevil_interfaces_ai.sh

Rebuilds the package with the updated setup.py file:

```bash
#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/nevil

# Remove build and install directories for the package
rm -rf build/nevil_interfaces_ai install/nevil_interfaces_ai

# Rebuild the package
colcon build --packages-select nevil_interfaces_ai --symlink-install
```

## Package Configuration

### setup.py

Ensure `setup.py` includes all necessary files and entry points:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'nevil_interfaces_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Nevil interfaces AI package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dialog_manager = nevil_interfaces_ai.dialog_manager_node:main',
            'speech_recognition = nevil_interfaces_ai.speech_recognition_node:main',
            'speech_synthesis = nevil_interfaces_ai.speech_synthesis_node:main',
            'text_command_processor = nevil_interfaces_ai.text_command_processor:main',
        ],
    },
)
```

### package.xml

Ensure `package.xml` includes all necessary dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>nevil_interfaces_ai</name>
  <version>0.0.1</version>
  <description>Nevil interfaces AI package</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>nevil_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Troubleshooting Common Issues

### Module Not Found Errors

If you encounter "Module not found" errors:

1. Check your PYTHONPATH:
   ```bash
   echo $PYTHONPATH
   ```

2. Add the package to your PYTHONPATH:
   ```bash
   export PYTHONPATH=$PYTHONPATH:~/nevil/src/package_name
   ```

3. Check if the package is installed:
   ```bash
   ros2 pkg list | grep package_name
   ```

4. Rebuild the package:
   ```bash
   cd ~/nevil
   colcon build --packages-select package_name --symlink-install
   ```

### Launch File Errors

If you encounter launch file errors:

1. Check if the launch file is installed:
   ```bash
   ls ~/nevil/install/package_name/share/package_name/launch
   ```

2. Check if the launch file uses the correct paths:
   ```bash
   cat ~/nevil/src/package_name/launch/package.launch.py
   ```

3. Rebuild the package with symlink install:
   ```bash
   cd ~/nevil
   colcon build --packages-select package_name --symlink-install
   ```

### Configuration File Not Found

If configuration files are not found:

1. Check if the configuration file is installed:
   ```bash
   ls ~/nevil/install/package_name/share/package_name/config
   ```

2. Check if the configuration file is referenced correctly:
   ```python
   import os
   from ament_index_python.packages import get_package_share_directory
   
   config_file = os.path.join(
       get_package_share_directory('package_name'),
       'config',
       'config.yaml'
   )
   ```

3. Rebuild the package with the updated setup.py file:
   ```bash
   cd ~/nevil
   colcon build --packages-select package_name --symlink-install
   ```

### Dual Directory Structure Issues

If you encounter issues related to the dual directory structure:

1. Use relative paths instead of absolute paths:
   ```python
   import os
   
   # Instead of this
   # config_file = '/home/dan/nevil/src/package_name/config/config.yaml'
   
   # Use this
   config_file = os.path.join(
       os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
       'config',
       'config.yaml'
   )
   ```

2. Use environment variables to determine the correct path:
   ```python
   import os
   
   # Check if we're in development or deployment
   if 'NEVIL_DEV' in os.environ:
       base_path = os.environ['NEVIL_DEV']
   else:
       base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
   
   config_file = os.path.join(base_path, 'config', 'config.yaml')
   ```

3. Use the ROS2 package system to find resources:
   ```python
   import os
   from ament_index_python.packages import get_package_share_directory
   
   config_file = os.path.join(
       get_package_share_directory('package_name'),
       'config',
       'config.yaml'
   )
   ```

## Best Practices

### 1. Use colcon build with symlink-install

Always build packages with the `--symlink-install` option:

```bash
colcon build --symlink-install
```

This creates symlinks instead of copying files, which means changes to launch files and other resources are immediately available without rebuilding.

### 2. Include all necessary files in setup.py

Ensure `setup.py` includes all necessary files:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
],
```

### 3. Use entry points in setup.py

Use entry points in `setup.py` for executable scripts:

```python
entry_points={
    'console_scripts': [
        'node_name = package_name.module:main',
    ],
},
```

### 4. Use consistent paths in all scripts

Use consistent paths in all scripts and configuration files:

```python
import os
from ament_index_python.packages import get_package_share_directory

# Get the package share directory
package_share_dir = get_package_share_directory('package_name')

# Get the launch directory
launch_dir = os.path.join(package_share_dir, 'launch')

# Get the configuration directory
config_dir = os.path.join(package_share_dir, 'config')

# Get a specific configuration file
config_file = os.path.join(config_dir, 'config.yaml')
```

### 5. Use the Node action in launch files

Use the `Node` action in launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='node_name',
            output='screen',
        )
    ])
```

### 6. Source setup files in the correct order

Source setup files in the correct order:

```bash
# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace setup
source ~/nevil/install/setup.bash
```

### 7. Use wrapper scripts for complex setups

Use wrapper scripts for complex setups:

```bash
#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Set up workspace environment
source ~/nevil/install/setup.bash

# Set up additional environment variables
export NEVIL_CONFIG_DIR=~/nevil/src/package_name/config

# Execute the command
ros2 launch package_name package.launch.py
```

### 8. Document path resolution strategies

Document path resolution strategies in README files and code comments:

```python
# This function resolves the path to a configuration file
# It tries multiple strategies:
# 1. Using the package share directory (for installed packages)
# 2. Using relative paths (for development)
# 3. Using environment variables (for custom setups)
def resolve_config_path(config_name):
    import os
    from ament_index_python.packages import get_package_share_directory
    
    # Strategy 1: Using the package share directory
    try:
        package_share_dir = get_package_share_directory('package_name')
        config_path = os.path.join(package_share_dir, 'config', config_name)
        if os.path.exists(config_path):
            return config_path
    except Exception:
        pass
    
    # Strategy 2: Using relative paths
    try:
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(base_path, 'config', config_name)
        if os.path.exists(config_path):
            return config_path
    except Exception:
        pass
    
    # Strategy 3: Using environment variables
    try:
        if 'NEVIL_CONFIG_DIR' in os.environ:
            config_path = os.path.join(os.environ['NEVIL_CONFIG_DIR'], config_name)
            if os.path.exists(config_path):
                return config_path
    except Exception:
        pass
    
    # If all strategies fail, raise an exception
    raise FileNotFoundError(f"Could not find configuration file: {config_name}")
```

By following these best practices, you can avoid most path resolution issues in the Nevil-picar-v2 project.