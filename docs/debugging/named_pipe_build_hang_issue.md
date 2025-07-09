# Named Pipe Build Hang Issue - Debugging Guide

## Issue Summary
**Problem**: ROS2 colcon build hanging indefinitely during CMake copy operations
**Root Cause**: Named pipe files (`.lgd-nfy0`) created by development tools
**Impact**: Build times exceeding 6+ minutes instead of normal 15-20 seconds
**Resolution**: Remove named pipe files and clean build directory

## Technical Details

### What Are Named Pipes?
Named pipes (FIFOs) are special files that act as communication channels between processes:
- **File Type**: `prw-rw-r--` (p = pipe, rw = read/write permissions)
- **Behavior**: Block operations when no reader/writer is attached
- **CMake Issue**: `cmake -E copy_directory` cannot handle these files properly

### Symptoms
1. **Build Hanging**: `colcon build` appears to freeze
2. **CMake Processes**: Multiple stuck `cmake -E copy_directory` processes
3. **No Progress**: Build progress stops at package copy stage
4. **High CPU**: CMake processes consuming CPU while stuck

### Identification Commands
```bash
# Find all named pipes in project
find /home/dan/Nevil-picar-v2 -name ".lgd-nfy*" -type p

# Check file type
ls -la /path/to/.lgd-nfy0
# Output: prw-rw-r-- 1 dan dan 0 Jul  9 02:49 .lgd-nfy0

# Find stuck CMake processes
ps aux | grep "cmake.*copy_directory"

# Check what's using the pipes
lsof | grep "\.lgd-nfy"
```

## Root Cause Analysis

### Named Pipe Pattern: `.lgd-nfy0`
- **lgd**: Likely "Log/Debug" 
- **nfy**: Likely "Notify"
- **0**: Instance number

### Suspected Sources
1. **VS Code Language Server**
   - Python language server debugging features
   - Real-time code analysis notifications
   - Debugging session communication

2. **VS Code Extensions**
   - Code monitoring extensions
   - Debugging tools
   - Performance profilers

3. **Development Environment**
   - IDE notification systems
   - Real-time logging tools
   - Code analysis pipelines

### Evidence Supporting VS Code Theory
- VS Code processes running during pipe creation
- Pipes created at 02:49 AM during active development
- Multiple pipes across project directories
- No active processes using pipes (orphaned after VS Code session)

## Locations Found
During the July 9, 2025 incident, named pipes were found in:
```
/home/dan/Nevil-picar-v2/.lgd-nfy0
/home/dan/Nevil-picar-v2/v1.0/.lgd-nfy0
/home/dan/Nevil-picar-v2/v1.0/picarlibs/.lgd-nfy0
/home/dan/Nevil-picar-v2/test/navigation/.lgd-nfy0
/home/dan/Nevil-picar-v2/src/.lgd-nfy0
/home/dan/Nevil-picar-v2/src/nevil_navigation/nevil_navigation/.lgd-nfy0
```

## Resolution Steps

### Immediate Fix
```bash
# 1. Kill stuck CMake processes
pkill -f "cmake.*copy_directory.*nevil_navigation"

# 2. Remove all named pipes
find /home/dan/Nevil-picar-v2 -name ".lgd-nfy*" -type p -delete

# 3. Clean build directory
rm -rf build/nevil_navigation

# 4. Rebuild package
colcon build --packages-select nevil_navigation
```

### Verification
```bash
# Confirm pipes are gone
find /home/dan/Nevil-picar-v2 -name ".lgd-nfy*" 2>/dev/null || echo "All pipes removed"

# Check build success
ls -la install/nevil_navigation/lib/nevil_navigation/
```

## Prevention Strategies

### 1. Build Script Enhancement
Add pipe cleanup to build preparation:
```bash
#!/bin/bash
# Clean named pipes before build
echo "Cleaning named pipes..."
find . -name ".lgd-nfy*" -type p -delete 2>/dev/null || true

# Continue with normal build
colcon build --packages-select nevil_navigation
```

### 2. Git Ignore Rules
Add to `.gitignore`:
```
# Development tool artifacts
.lgd-nfy*
*.pipe
*.fifo
```

### 3. Monitoring Script
```bash
#!/bin/bash
# Monitor for pipe creation
watch -n 5 'find /home/dan/Nevil-picar-v2 -name ".lgd-nfy*" -type p 2>/dev/null'
```

### 4. VS Code Configuration
Consider disabling aggressive debugging features:
- Review Python extension settings
- Disable real-time code analysis if not needed
- Check debugging extension configurations

## Debugging Commands Reference

### Process Investigation
```bash
# Find all processes with 'lgd' or 'nfy' in command/name
ps aux | grep -E "(lgd|nfy)" | grep -v grep

# Check system logs for pipe creation
journalctl --since "02:45" --until "02:55" | grep -i "lgd\|nfy\|pipe"

# Find VS Code processes
ps aux | grep -i "code\|vscode" | head -5
```

### File System Analysis
```bash
# Find all special files
find /home/dan/Nevil-picar-v2 -type p

# Check file creation times
stat /path/to/.lgd-nfy0

# Monitor file system events (if inotify available)
inotifywait -m -r /home/dan/Nevil-picar-v2 --include="\.lgd-nfy.*"
```

## Lessons Learned

1. **Named Pipes Block CMake**: CMake copy operations cannot handle FIFO files
2. **Development Tools Create Artifacts**: IDE tools can create problematic files
3. **Build Environment Hygiene**: Regular cleanup prevents build issues
4. **Process Monitoring**: Multiple stuck processes indicate systematic issue
5. **File Type Awareness**: Understanding special file types helps debugging

## Future Monitoring

### Watch for Recurrence
- Monitor build times for sudden increases
- Check for `.lgd-nfy*` files during development
- Observe VS Code extension behavior
- Track CMake process behavior

### Early Warning Signs
- Build hanging at copy stage
- Multiple CMake processes in process list
- Sudden appearance of named pipe files
- VS Code debugging sessions creating artifacts

## Related Issues
- CMake copy_directory limitations with special files
- VS Code language server debugging artifacts
- ROS2 build system interaction with development tools
- File system special file handling in build systems

---
**Document Created**: July 9, 2025 03:01 AM  
**Issue Resolved**: July 9, 2025 02:57 AM  
**Build Time After Fix**: 15.6 seconds (SUCCESS)