# Servo Calibration Architecture Analysis: v1.0 vs v2.0

## Problem Statement
Nevil 2.0's front wheel servo lists 45 degrees to the left despite calibration attempts, while Nevil 1.0 works fine with identical hardware.

## Root Cause: Environmental Architecture Differences

### v1.0 Architecture (Working)
```
Direct Execution Flow:
User Script → picarx.py → fileDB → /opt/picar-x/picar-x.conf → Calibrated Values
```

### v2.0 Architecture (Problematic)
```
ROS2 Execution Flow:
ros2_hardware_start.sh → Config Creation/Reset → ROS2 Launch → navigation_node → picarx.py → fileDB → Default Values
```

## Critical Architectural Issues

### 1. Configuration File Lifecycle Management
**v1.0**: Preserves existing configuration
**v2.0**: `ros2_hardware_start.sh` lines 37-64 create default config, potentially overwriting calibration

### 2. Permission Context Switching
**v1.0**: Consistent user context
**v2.0**: sudo startup → ROS2 user context → potential permission mismatches

### 3. Initialization Sequence
**v1.0**: Direct hardware initialization
**v2.0**: Multi-layer initialization through ROS2 launch system

## Specific Problem Areas

### A. Config File Reset Logic (ros2_hardware_start.sh:44-64)
```bash
if [ -f "$CONFIG_FILE" ]; then
    # Check access but may still recreate
else
    # Always creates default config with picarx_dir_servo=0
fi
```

### B. Permission Inheritance
- Script runs with sudo
- ROS2 nodes may inherit different permissions
- fileDB access may fail silently

### C. Timing Issues
- Config file created during startup
- Hardware initialization happens before calibration load
- Race condition between config creation and hardware access

## Architectural Solutions

### 1. Preserve Existing Calibration
Modify ros2_hardware_start.sh to:
- Backup existing config before any modifications
- Only create default config if none exists
- Preserve calibration values during permission fixes

### 2. Consistent Permission Model
- Ensure ROS2 nodes inherit proper hardware permissions
- Validate fileDB access before hardware initialization
- Implement permission verification in navigation_node

### 3. Initialization Order
- Load existing calibration before hardware reset
- Apply calibration immediately after hardware initialization
- Verify calibration application before proceeding

## Implementation Priority
1. **Immediate**: Fix config preservation in ros2_hardware_start.sh
2. **Short-term**: Add calibration verification to navigation_node
3. **Long-term**: Implement robust permission inheritance model

## Testing Strategy
1. Verify existing calibration preservation
2. Test permission inheritance across sudo/ROS2 boundary
3. Validate initialization sequence timing
4. Confirm hardware state matches configuration