# Wrapper Script Template System

## Overview

The Nevil-picar v2.0 project includes an automatic wrapper script template system that ensures hardware library paths are correctly configured even after clean rebuilds. This system solves the problem of manually fixed wrapper scripts being lost when the `install/` directory is deleted.

## Problem Solved

**Before**: Clean rebuilds (`rm -rf install/ && ./nevil build`) would wipe out manually fixed wrapper scripts, requiring manual re-application of robot-hat library paths.

**After**: The template system automatically applies corrected wrapper scripts after every build, making clean rebuilds completely safe.

## System Architecture

### Template Storage
```
src/wrapper_templates/
├── COLCON_IGNORE              # Prevents colcon from building this as a package
├── README.md                  # Documentation for the template system
├── rt_motor_control_node      # Realtime motor control wrapper
├── rt_sensor_node             # Realtime sensor wrapper
├── rt_config_manager          # Realtime config manager wrapper
├── ai_interface_node          # AI interface wrapper
├── speech_recognition_node    # Speech recognition wrapper
├── speech_synthesis_node      # Speech synthesis wrapper
├── dialog_manager_node        # Dialog manager wrapper
├── system_manager_node        # System manager wrapper
├── system_monitor             # System monitor wrapper
├── battery_monitor            # Battery monitor wrapper
├── hardware_init              # Hardware initialization wrapper
├── motion_control_node        # Navigation motion control wrapper
├── simulation_node            # Simulation wrapper
├── visualization_node         # Visualization wrapper
├── nevil_cli                  # Command-line interface wrapper
└── system_monitor_wrapper     # System monitor wrapper script
```

### Automatic Application
The `nevil` build script automatically applies templates after colcon build completes:

1. **Colcon Build**: Normal ROS2 package building
2. **Template Detection**: Check if `src/wrapper_templates/` exists
3. **Target Verification**: Verify `install/nevil_realtime/` exists
4. **Template Application**: Copy templates to install directory
5. **Permission Setting**: Make scripts executable

## Usage

### Full Project Build
```bash
./nevil build
```
**Result**: All packages built + wrapper scripts automatically applied

### Single Package Build
```bash
./nevil build --packages-select nevil_realtime
```
**Result**: nevil_realtime package built + wrapper scripts automatically applied

### Multiple Package Build
```bash
./nevil build --packages-select nevil_interfaces_ai nevil_realtime
```
**Result**: Selected packages built + wrapper scripts applied if nevil_realtime included

## Template Configuration

Each wrapper script template includes the correct hardware library paths:

```bash
#!/bin/bash
# Wrapper script for [node_name]

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set up the Python path including picar-x and robot_hat libraries
export PYTHONPATH="$SCRIPT_DIR/../python3.11/site-packages:/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"

# Run the actual Python file directly
python_file="$SCRIPT_DIR/[node_name].py"
exec python3 "$python_file" "$@"
```

## Library Paths Configured

- **picar-x**: `/home/dan/picar-x` - PiCar-X hardware control library
- **robot-hat**: `/home/dan/robot-hat` - Robot HAT hardware interface library
- **ROS2 Packages**: `$SCRIPT_DIR/../python3.11/site-packages` - Package-specific Python modules

## Build Script Integration

The `nevil` script includes post-build logic:

```bash
elif [[ "$1" == "build" ]]; then
    shift
    echo "? Building with: colcon build $@"
    colcon build "$@"
    
    # Post-build: Apply wrapper script fixes
    apply_wrapper_fixes() {
        if [[ -d "src/wrapper_templates" ]] && [[ -d "install/nevil_realtime" ]]; then
            echo "? Applying wrapper script fixes..."
            cp src/wrapper_templates/rt_* install/nevil_realtime/lib/nevil_realtime/ 2>/dev/null
            if [[ $? -eq 0 ]]; then
                chmod +x install/nevil_realtime/lib/nevil_realtime/rt_*
                echo "? Wrapper scripts updated with robot-hat library paths"
            fi
        fi
    }
    
    # Apply wrapper fixes for all packages
    apply_wrapper_fixes() {
        if [[ -d "src/wrapper_templates" ]]; then
            echo "? Applying wrapper script fixes..."
            local fixes_applied=0
            
            # Apply fixes for each package
            for package in nevil_bringup nevil_interfaces_ai nevil_core nevil_navigation nevil_realtime nevil_simulation; do
                if [[ -d "install/$package/lib/$package" ]]; then
                    # Copy wrapper scripts for this package
                    for template in src/wrapper_templates/*; do
                        if [[ -f "$template" && -x "$template" && "$template" != *".md" && "$template" != *"COLCON_IGNORE" ]]; then
                            template_name=$(basename "$template")
                            target_dir="install/$package/lib/$package"
                            
                            # Check if this wrapper belongs to this package
                            if [[ -f "$target_dir/$template_name" || -f "$target_dir/${template_name}.py" ]]; then
                                cp "$template" "$target_dir/" 2>/dev/null
                                chmod +x "$target_dir/$template_name" 2>/dev/null
                                fixes_applied=1
                            fi
                        fi
                    done
                fi
            done
            
            if [[ $fixes_applied -eq 1 ]]; then
                echo "? Wrapper scripts updated with correct library paths"
            fi
        fi
    }
    
    # Apply wrapper fixes for all builds
    apply_wrapper_fixes
```

## Benefits

### ✅ Clean Rebuild Safe
- `rm -rf install/` followed by `./nevil build` preserves wrapper fixes
- No manual intervention required after clean rebuilds

### ✅ Automatic Application
- Templates applied automatically on every relevant build
- No need to remember manual steps

### ✅ Version Controlled
- Templates stored in `src/` directory as part of source code
- Changes tracked in git history

### ✅ Consistent Configuration
- Everyone gets the same wrapper script configuration
- Eliminates environment-specific issues

### ✅ Single Package Build Support
- Works with `--packages-select nevil_realtime`
- Selective building doesn't break wrapper application

### ✅ Development Friendly
- Developers can modify templates and changes apply automatically
- No need to manually update install directory files

## Maintenance

### Updating Templates
1. Modify wrapper scripts in `src/wrapper_templates/`
2. Run `./nevil build --packages-select nevil_realtime`
3. Changes automatically applied to install directory

### Adding New Templates
1. Add new wrapper script to `src/wrapper_templates/`
2. Update `nevil` script to include new template in copy operation
3. Templates automatically applied on next build

### Troubleshooting

#### Templates Not Applied
**Check**: Verify `src/wrapper_templates/` directory exists
```bash
ls -la src/wrapper_templates/
```

**Check**: Verify nevil_realtime package was built
```bash
ls -la install/nevil_realtime/lib/nevil_realtime/
```

#### Manual Application
If needed, manually apply templates:
```bash
cp src/wrapper_templates/rt_* install/nevil_realtime/lib/nevil_realtime/
chmod +x install/nevil_realtime/lib/nevil_realtime/rt_*
```

## Testing the System

### Test Clean Rebuild
```bash
# Backup current state
cp -r install/ install_backup/

# Clean rebuild
rm -rf install/
./nevil build

# Verify wrapper scripts have correct paths
grep "robot-hat" install/nevil_realtime/lib/nevil_realtime/rt_motor_control_node
```

### Test Single Package Build
```bash
./nevil build --packages-select nevil_realtime
# Should see: "? Wrapper scripts updated with robot-hat library paths"
```

### Test Library Access
```bash
# Test that libraries are accessible through wrapper
export PYTHONPATH="/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"
python3 -c "import picarx, robot_hat; print('Both libraries accessible')"
```

## Related Documentation

- [Hardware Library Setup Guide](HARDWARE_LIBRARY_SETUP.md) - Complete hardware library configuration
- [Build Status](build/BUILD_STATUS.md) - Current system build status and progress
- [Launch System Guide](launch_system_guide.md) - System launch and troubleshooting procedures

---

**Note**: This system ensures that hardware library paths are never lost during development, making the build process robust and developer-friendly.