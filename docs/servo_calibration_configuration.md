# Servo Calibration Configuration

## Configuration File Location

**Primary Configuration**: `/opt/picar-x/picar-x.conf`

### What is `/opt`?
- `/opt` is a standard Linux directory for optional software packages (regular directory, not a symlink)
- Contains third-party software installations like PiCar-X libraries
- Other packages in `/opt/`: `pigpio`, `vilib`, `WidevineCdm`

### Configuration File Details
- **Full Path**: `/opt/picar-x/picar-x.conf`
- **Permissions**: `-rwxrwxrwx` (readable/writable by user `dan`)
- **Size**: ~161 bytes
- **Format**: Key-value pairs with comments
- **Persistence**: Values survive system reboots

## Current Configuration Values

```bash
# robot-hat config and calibration value of robots

picarx_dir_servo = 36.0
picarx_cam_pan_servo = 0.0
picarx_cam_tilt_servo = 0.0
picarx_dir_motor = [1, 1]
```

## Configuration Keys

### Servo Configuration
- `picarx_dir_servo` - Direction servo calibration value (currently: 36.0)
- `picarx_cam_pan_servo` - Camera pan servo calibration value (currently: 0.0)
- `picarx_cam_tilt_servo` - Camera tilt servo calibration value (currently: 0.0)

### Motor Configuration
- `picarx_dir_motor` - Motor direction calibration values (currently: [1, 1])

## How Configuration is Accessed

### Code Access
1. **FileDB System**: `robot_hat.filedb.fileDB` reads this configuration file
2. **Import Path**: `from robot_hat.filedb import fileDB` (note: lowercase 'filedb' module, uppercase 'fileDB' class)
3. **Key Definitions**: `picarx.py` defines the config keys (lines 69-71)
4. **Value Retrieval**: `fileDB.get()` method retrieves values
5. **Calibration Updates**: Servo calibration methods update these values

### FileDB Usage Example
```python
from robot_hat.filedb import fileDB

# Initialize fileDB
db = fileDB('/opt/picar-x/picar-x.conf', 777, 'dan')

# Read servo calibration values
dir_servo_value = db.get('picarx_dir_servo', 0)
cam_pan_value = db.get('picarx_cam_pan_servo', 0)
cam_tilt_value = db.get('picarx_cam_tilt_servo', 0)

# Update servo calibration values
db.set('picarx_dir_servo', 36.0)
```

### Manual Access
```bash
# View current configuration
cat /opt/picar-x/picar-x.conf

# Check specific servo values
cat /opt/picar-x/picar-x.conf | grep servo

# View file properties
ls -la /opt/picar-x/picar-x.conf
```

## Calibration Process

The servo calibration calls in `calibration.py`:
- `px.dir_servo_calibrate(servos_offset[0])` - Updates `picarx_dir_servo`
- `px.cam_pan_servo_calibrate(servos_offset[1])` - Updates `picarx_cam_pan_servo`
- `px.cam_tilt_servo_calibrate(servos_offset[2])` - Updates `picarx_cam_tilt_servo`

### Calibration Flow
1. User provides servo offset values
2. Calibration methods apply offsets to servos
3. New calibration values are saved to `/opt/picar-x/picar-x.conf`
4. Values persist across system reboots

---

# Servo Calibration Logging

## Log File Locations

### Primary Log Files
| File Path | Purpose | Level | Rotation |
|-----------|---------|-------|----------|
| `/home/dan/Nevil-picar-v2/logs/servo_calibration.log` | Main calibration session logs | INFO+ | 10MB, 3 backups |
| `/home/dan/Nevil-picar-v2/logs/servo_calibration_debug.log` | Detailed debug information | DEBUG+ | 10MB, 2 backups |

### Log File Access
```bash
# Navigate to project directory
cd /home/dan/Nevil-picar-v2

# View current calibration session
tail -f logs/servo_calibration.log

# View detailed debug information
tail -f logs/servo_calibration_debug.log

# Check servo configuration values
cat /opt/picar-x/picar-x.conf | grep servo

# View all calibration logs
ls -la logs/servo_calibration*
```

## Log Levels & Content

### INFO Level
- Session start/end events
- Servo calibration operations
- Offset adjustments
- Configuration saves
- User interactions

### DEBUG Level
- Function entry/exit
- Hardware communication details
- Configuration file operations
- Servo movement commands
- Internal state changes

### WARNING Level
- Constraint violations
- Value clamping operations
- Recovery procedures

### ERROR Level
- Hardware communication failures
- Configuration save errors
- Exception handling

## Log Format

### Detailed Format (Files)
```
2025-01-07 01:05:00,123 - servo_calibration - INFO - [cali_helper:154] - Direction servo calibration successful: 0.0 -> 1.2
```

### Simple Format (Console)
```
2025-01-07 01:05:00,123 - INFO - Direction servo calibration successful: 0.0 -> 1.2
```

## Log Rotation Policy

### Automatic Rotation
- **Trigger**: File size exceeds 10MB
- **Retention**: 3 backups for main log, 2 for debug
- **Naming**: `servo_calibration.log.1`, `servo_calibration.log.2`, etc.

## Troubleshooting

### Log Analysis Commands
```bash
# Navigate to project directory
cd /home/dan/Nevil-picar-v2

# Find calibration errors
grep -i error logs/servo_calibration*.log

# Track servo value changes
grep "servo.*successful" logs/servo_calibration.log

# Monitor real-time calibration
tail -f logs/servo_calibration.log | grep -E "(servo|calibration)"

# Check configuration persistence
grep -E "picarx_(dir|cam)" /opt/picar-x/picar-x.conf