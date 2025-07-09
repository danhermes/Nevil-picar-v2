# Servo Calibration Logging Reference

## Log File Locations

### Primary Log Files
| File Path | Purpose | Level | Rotation |
|-----------|---------|-------|----------|
| `/home/dan/Nevil-picar-v2/logs/servo_calibration.log` | Main calibration session logs | INFO+ | 10MB, 3 backups |
| `/home/dan/Nevil-picar-v2/logs/servo_calibration_debug.log` | Detailed debug information | DEBUG+ | 10MB, 2 backups |
| `/opt/picar-x/picar-x.conf` | Persistent servo calibration values | N/A | System managed |

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

## Configuration File Details

### What is `/opt/picar-x/picar-x.conf`?

**Directory Structure:**
- `/opt/` - Standard Linux directory for optional software packages (regular directory, not a symlink)
- `/opt/picar-x/` - PiCar-X specific configuration directory
- `/opt/picar-x/picar-x.conf` - Main configuration file for servo calibration values

**File Properties:**
- **Permissions**: `-rwxrwxrwx` (readable/writable by user `dan`)
- **Size**: ~161 bytes
- **Format**: Key-value pairs with comments
- **Persistence**: Values survive system reboots

**Current Configuration Values:**
```bash
# robot-hat config and calibration value of robots

picarx_dir_servo = 36.0
picarx_cam_pan_servo = 0.0
picarx_cam_tilt_servo = 0.0
picarx_dir_motor = [1, 1]
```

**Other Packages in `/opt/`:**
- `pigpio` - GPIO control library
- `vilib` - Vision library
- `WidevineCdm` - DRM content decryption

## Configuration Keys Logged

### Servo Configuration
- `picarx_dir_servo` - Direction servo calibration value (currently: 36.0)
- `picarx_cam_pan_servo` - Camera pan servo calibration value (currently: 0.0)
- `picarx_cam_tilt_servo` - Camera tilt servo calibration value (currently: 0.0)

### Motor Configuration
- `picarx_dir_motor` - Motor direction calibration values (currently: [1, 1])

## Log Rotation Policy

### Automatic Rotation
- **Trigger**: File size exceeds 10MB
- **Retention**: 3 backups for main log, 2 for debug
- **Naming**: `servo_calibration.log.1`, `servo_calibration.log.2`, etc.

### Manual Cleanup
```bash
# Remove old logs (optional)
rm /tmp/servo_calibration*.log.*

# Archive current session
cp /tmp/servo_calibration.log ~/servo_calibration_$(date +%Y%m%d_%H%M%S).log
```

## Troubleshooting

### Common Log Locations
1. **Current Session**: `/home/dan/Nevil-picar-v2/logs/servo_calibration.log`
2. **Debug Details**: `/home/dan/Nevil-picar-v2/logs/servo_calibration_debug.log`
3. **Configuration**: `/opt/picar-x/picar-x.conf`
4. **System Logs**: `/var/log/syslog` (for system-level issues)

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
```

## Integration with System Monitoring

### Log Aggregation
- Logs can be forwarded to centralized logging systems
- Compatible with rsyslog, journald, or custom log collectors
- JSON format available for structured logging needs

### Alerting
- Monitor for ERROR level messages
- Track calibration drift over time
- Alert on configuration file corruption

## Security Considerations

### File Permissions
- Log files: `644` (owner read/write, others read)
- Config files: `777` with user restrictions
- No sensitive data logged (hardware values only)

### Data Retention
- Temporary logs in `/tmp/` (system cleanup)
- Configuration persisted in `/opt/picar-x/`
- No personal or network information logged