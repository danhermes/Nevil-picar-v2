# Navigation Servo Fix Testing Guide

## Overview
This guide provides instructions for testing the servo angle direction fixes implemented in the navigation system. The fix addresses the critical bug where Nevil was turning LEFT when commanded to turn RIGHT.

## What Was Fixed
- **File**: `src/nevil_navigation/nevil_navigation/picar_actions.py`
- **Issue**: Servo angles were inverted - LEFT commands used -30° (should be +30°), RIGHT commands used +30° (should be -30°)
- **Fix**: Swapped servo angles for left/right turns in both regular and in-place turn methods
- **Debug**: Added logging to help troubleshoot future directional issues

## Testing Prerequisites

### 1. Environment Setup
```bash
# Source the workspace
cd /home/dan/Nevil-picar-v2
source install/setup.bash

# Verify the navigation package was rebuilt
ls -la install/nevil_navigation/
```

### 2. Hardware Requirements
- PiCar-X robot with servo motor
- Sufficient space for robot movement
- Clear line of sight to observe robot behavior

## Testing Procedures

### Test 1: Basic Directional Commands
Test individual directional commands to verify servo angles are correct.

```bash
# Terminal 1: Start the navigation system
ros2 launch nevil_bringup physical_robot.launch.py

# Terminal 2: Test individual commands
# Test RIGHT turn (should turn clockwise/right)
ros2 topic pub --once /nevil/navigation/cmd_vel geometry_msgs/msg/Twist '{angular: {z: -1.0}}'

# Test LEFT turn (should turn counter-clockwise/left)  
ros2 topic pub --once /nevil/navigation/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.0}}'

# Test FORWARD movement
ros2 topic pub --once /nevil/navigation/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'

# Test BACKWARD movement
ros2 topic pub --once /nevil/navigation/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -1.0}}'
```

### Test 2: Navigation Monitor Verification
Use the Navigation Monitor to observe real-time behavior and verify commands are processed correctly.

```bash
# Terminal 1: Start navigation system
ros2 launch nevil_bringup physical_robot.launch.py

# Terminal 2: Start Navigation Monitor
cd /home/dan/Nevil-picar-v2/src/nevil_navigation/scripts
python3 navigation_monitor.py

# Terminal 3: Send test commands and observe in monitor
ros2 topic pub --once /nevil/navigation/cmd_vel geometry_msgs/msg/Twist '{angular: {z: -1.0}}'
```

### Test 3: AI Command Integration
Test the complete pipeline from AI commands to physical movement.

```bash
# Terminal 1: Start full system
ros2 launch nevil_bringup physical_robot.launch.py

# Terminal 2: Start AI interface
ros2 launch nevil_interfaces_ai speech_interface.launch.py

# Terminal 3: Send AI commands
ros2 topic pub --once /nevil/ai/text_command nevil_interfaces_ai_msgs/msg/TextCommand '{command: "turn right"}'
ros2 topic pub --once /nevil/ai/text_command nevil_interfaces_ai_msgs/msg/TextCommand '{command: "turn left"}'
```

## Expected Results

### Correct Behavior (After Fix)
- **"turn right"** or `angular.z: -1.0` → Robot turns clockwise (right)
- **"turn left"** or `angular.z: 1.0` → Robot turns counter-clockwise (left)
- **"move forward"** or `linear.x: 1.0` → Robot moves forward
- **"move backward"** or `linear.x: -1.0` → Robot moves backward

### Debug Logging
Monitor the logs for action mapping confirmation:
```bash
# Watch navigation logs
ros2 topic echo /rosout | grep navigation_node
```

Look for log messages like:
```
[navigation_node]: Action mapped: turn_right -> servo_angle: -30
[navigation_node]: Action mapped: turn_left -> servo_angle: +30
```

## Troubleshooting

### If Directions Are Still Wrong
1. **Check servo calibration**:
   ```bash
   # Test servo directly
   python3 -c "
   from nevil_navigation.picarx import Picarx
   px = Picarx()
   px.set_dir_servo_angle(-30)  # Should turn right
   px.set_dir_servo_angle(30)   # Should turn left
   px.set_dir_servo_angle(0)    # Should center
   "
   ```

2. **Verify package installation**:
   ```bash
   ros2 pkg list | grep nevil_navigation
   ros2 interface show nevil_interfaces/msg/SystemStatus
   ```

3. **Check for hardware issues**:
   - Ensure servo connections are secure
   - Verify power supply to servo motor
   - Test servo manually with PiCar-X utilities

### If Commands Are Not Received
1. **Check topic connections**:
   ```bash
   ros2 topic list | grep cmd_vel
   ros2 topic info /nevil/navigation/cmd_vel
   ```

2. **Verify node status**:
   ```bash
   ros2 node list | grep navigation
   ros2 node info /nevil_navigation_node
   ```

## Success Criteria
- ✅ Robot turns RIGHT when commanded to turn right
- ✅ Robot turns LEFT when commanded to turn left  
- ✅ Forward/backward movements work correctly
- ✅ Navigation Monitor shows correct action mappings
- ✅ AI commands translate to correct physical movements
- ✅ Debug logs confirm proper servo angle assignments

## Post-Testing
After successful testing:
1. Document any remaining issues
2. Update system documentation if needed
3. Consider adding automated tests for directional commands
4. Archive this testing guide for future reference

## Files Modified
- `src/nevil_navigation/nevil_navigation/picar_actions.py` - Servo angle fixes
- `src/nevil_navigation/nevil_navigation/navigation_node.py` - Debug logging
- Documentation files - Navigation Monitor integration

---
**Testing Date**: January 7, 2025  
**Fix Version**: Post-rebuild after servo angle correction  
**Tester**: [Your Name]  
**Status**: [PASS/FAIL/PENDING]