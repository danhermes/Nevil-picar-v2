# Nevil Bringup

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