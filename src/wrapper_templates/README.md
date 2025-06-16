# Wrapper Script Templates

This directory contains template wrapper scripts that are automatically applied after colcon build to ensure proper hardware library paths are configured.

## Purpose

The wrapper scripts in this directory solve the problem of hardware library path configuration being lost during clean rebuilds. These templates contain the correct PYTHONPATH settings for accessing both picar-x and robot-hat libraries.

## Contents

### Realtime Package Wrappers (Hardware-specific)
- `rt_motor_control_node` - Motor control wrapper with robot-hat library path
- `rt_sensor_node` - Sensor node wrapper with robot-hat library path
- `rt_config_manager` - Configuration manager wrapper with robot-hat library path

### AI Interface Package Wrappers
- `ai_interface_node` - AI interface wrapper with message interface paths
- `speech_recognition_node` - Speech recognition wrapper
- `speech_synthesis_node` - Speech synthesis wrapper
- `dialog_manager_node` - Dialog manager wrapper

### Core System Wrappers
- `system_manager_node` - System manager wrapper
- `system_monitor` - System monitoring wrapper
- `battery_monitor` - Battery monitoring wrapper
- `hardware_init` - Hardware initialization wrapper

### Navigation Package Wrappers
- `motion_control_node` - Motion control wrapper

### Simulation Package Wrappers
- `simulation_node` - Simulation wrapper
- `visualization_node` - Visualization wrapper

### Bringup Package Wrappers
- `nevil_cli` - Command-line interface wrapper
- `system_monitor_wrapper` - System monitor wrapper script

### System Files
- `COLCON_IGNORE` - Prevents colcon from treating this as a ROS2 package
- `README.md` - This documentation file

## How It Works

1. **During Build**: The `nevil build` command runs colcon build normally
2. **Post-Build**: After colcon completes, the nevil script automatically copies these templates to `install/nevil_realtime/lib/nevil_realtime/`
3. **Result**: Wrapper scripts always have the correct robot-hat library paths, even after clean rebuilds

## Automatic Application

The wrapper scripts are automatically applied in these scenarios:

### Full Project Build
```bash
./nevil build
```

### Package-Specific Build (when nevil_realtime is included)
```bash
./nevil build --packages-select nevil_realtime
./nevil build --packages-select nevil_interfaces_ai nevil_realtime
```

## Library Paths Configured

Each wrapper script includes these library paths in PYTHONPATH:
- `/home/dan/picar-x` - PiCar-X hardware library
- `/home/dan/robot-hat` - Robot HAT hardware library
- Package-specific Python site-packages

## Manual Application

If needed, you can manually apply the wrapper scripts:
```bash
cp src/wrapper_templates/rt_* install/nevil_realtime/lib/nevil_realtime/
chmod +x install/nevil_realtime/lib/nevil_realtime/rt_*
```

## Maintenance

When wrapper scripts need updates:
1. Make changes to the templates in this directory
2. The changes will be automatically applied on the next build
3. No need to manually update install directory files

## Benefits

- ✅ **Clean Rebuild Safe**: `rm -rf install/` followed by `./nevil build` preserves wrapper fixes
- ✅ **Automatic**: No manual intervention required
- ✅ **Version Controlled**: Templates are part of source code
- ✅ **Consistent**: Everyone gets the same wrapper script configuration
- ✅ **Single Package Builds**: Works with selective package building