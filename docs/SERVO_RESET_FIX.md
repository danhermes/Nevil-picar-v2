# Servo Reset Fix - Nevil Wheel Skew Issue

## 🔍 Problem Diagnosis

**Issue**: Nevil's wheels were always skewed to the left at 30 degrees after certain actions.

**Root Cause**: Incomplete action sequences in `picar_actions.py` that modified steering servo angles but failed to reset them back to 0° (straight position).

## 🎯 Specific Problems Identified

1. **`turn_left_in_place()`** - Set steering to -30° but never reset to 0°
2. **`turn_right_in_place()`** - Set steering to 30° but never reset to 0° 
3. **`keep_think()`** - Left servos in random positions after animation sequence

## ✅ Solutions Implemented

### 1. Fixed Individual Action Methods

**File**: `src/nevil_navigation/nevil_navigation/picar_actions.py`

- **`turn_left_in_place()`**: Added 0.5s delay and explicit reset to 0°
- **`turn_right_in_place()`**: Added 0.5s delay and explicit reset to 0°
- **`keep_think()`**: Added `car.reset()` call at the end

### 2. Added Safety Mechanisms in NavigationNode

**File**: `src/nevil_navigation/nevil_navigation/navigation_node.py`

Added `picar_actions.initialize_servos()` calls in strategic locations:

1. **After every action execution** (line ~332)
   - Ensures wheels return to straight position after any action
   - Includes error handling to reset servos even if action fails

2. **During emergency stop** (line ~356)
   - Resets servos to safe position during emergency situations

3. **When entering standby mode** (line ~231)
   - Ensures wheels are straight when system goes to standby

## 🏗️ Separation of Concerns (SOC)

✅ **Good Practice**: NavigationNode calls `picar_actions.initialize_servos()`
❌ **Avoided**: Direct calls to picarx from NavigationNode

This maintains proper abstraction layers where:
- NavigationNode handles high-level coordination
- PicarActions handles hardware abstraction
- Picarx handles low-level hardware control

## 🔧 Code Changes Summary

### navigation_node.py
```python
# After action execution
self.picar_actions.initialize_servos()

# During emergency stop
if hasattr(self, 'picar_actions') and self.picar_actions is not None:
    self.picar_actions.initialize_servos()

# When entering standby mode
if hasattr(self, 'picar_actions') and self.picar_actions is not None:
    self.picar_actions.initialize_servos()
```

### picar_actions.py
```python
def turn_left_in_place(self):
    self.car.set_dir_servo_angle(-30)
    sleep(0.5)  # Allow time for the turn
    self.car.set_dir_servo_angle(0)  # Reset to straight

def turn_right_in_place(self):
    self.car.set_dir_servo_angle(30)
    sleep(0.5)  # Allow time for the turn
    self.car.set_dir_servo_angle(0)  # Reset to straight

def keep_think(self):
    # ... animation sequence ...
    self.car.reset()  # Reset all servos at end
```

## 🚀 Expected Results

- ✅ Wheels stay straight (0°) after any action
- ✅ No more persistent 30° left skew
- ✅ Better safety through multiple reset points
- ✅ Proper SOC with `initialize_servos()` abstraction
- ✅ Robust error handling for servo reset failures

## 🧪 Testing

Created verification script: `test/navigation/verify_servo_reset_fix.py`

Run with:
```bash
python test/navigation/verify_servo_reset_fix.py
```

## 📝 Notes

- The `initialize_servos()` method sets all servos to 0° (straight position)
- Multiple safety reset points ensure wheels return to straight even if individual actions fail
- Error handling prevents servo reset failures from crashing the system
- SOC maintained by using PicarActions abstraction layer

## 🔄 Future Improvements

Consider adding:
- Periodic servo position verification
- Servo position feedback monitoring
- Automatic correction if servos drift from expected positions