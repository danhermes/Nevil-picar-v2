# Hardware Library Setup Guide

## Overview

The Nevil-picar v2.0 system requires two critical hardware libraries for proper operation:

1. **picar-x** - Located at `/home/dan/picar-x`
2. **robot_hat** - Located at `/home/dan/robot-hat`

Both libraries must be accessible to the Python runtime environment for hardware nodes to function correctly.

## Library Locations

### picar-x Library
```bash
# Verify picar-x library exists
ls -la /home/dan/picar-x/picarx
```

### robot_hat Library
```bash
# Verify robot_hat library exists
ls -la /home/dan/robot-hat
```

## Environment Configuration

### Manual PYTHONPATH Setup
```bash
# Add both libraries to Python path
export PYTHONPATH="/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"
```

### Wrapper Script Template System

The system now includes an **automatic wrapper script template system** that ensures correct library paths are always applied, even after clean rebuilds.

#### Template Location:
- `src/wrapper_templates/` - Contains corrected wrapper scripts with robot-hat library paths
- `src/wrapper_templates/COLCON_IGNORE` - Prevents colcon from treating this as a package

#### Automatic Application:
The `nevil build` command automatically applies these templates after colcon build completes:

```bash
# Full project build
./nevil build

# Single package build (when nevil_realtime is included)
./nevil build --packages-select nevil_realtime
```

#### Template Configuration:
All wrapper scripts include both hardware libraries:
```bash
export PYTHONPATH="$SCRIPT_DIR/../python3.11/site-packages:/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"
```

## Verification Steps

### Test Library Access
```bash
# Test picar-x import
python3 -c "import picarx; print('picar-x: OK')"

# Test robot_hat import
python3 -c "import robot_hat; print('robot_hat: OK')"
```

### Test Combined Environment
```bash
# Set environment and test both libraries
export PYTHONPATH="/home/dan/picar-x:/home/dan/robot-hat:$PYTHONPATH"
python3 -c "import picarx, robot_hat; print('Both libraries: OK')"
```

## Troubleshooting

### Issue: "No module named 'robot_hat'"
**Solution**: Verify robot_hat library location and update PYTHONPATH:
```bash
ls -la /home/dan/robot-hat
export PYTHONPATH="/home/dan/robot-hat:$PYTHONPATH"
```

### Issue: "No module named 'picarx'"
**Solution**: Verify picar-x library location and update PYTHONPATH:
```bash
ls -la /home/dan/picar-x/picarx
export PYTHONPATH="/home/dan/picar-x:$PYTHONPATH"
```

### Issue: Hardware nodes failing with import errors
**Solution**: Update wrapper scripts to include both library paths as shown above.

## Implementation Notes

1. **Wrapper Script Priority**: The wrapper scripts automatically handle library path configuration during system launch
2. **Manual Testing**: Use manual PYTHONPATH export for testing individual nodes
3. **System Integration**: Both libraries are required for full hardware functionality
4. **Path Order**: Library paths should be added before existing PYTHONPATH to ensure priority

## Environment Configuration

### AI Interface Environment Variables
The system requires a `.env` file in the project root containing:
```bash
# OpenAI API Key (required for AI language processing)
OPENAI_API_KEY="your-openai-api-key-here"

# Speech Recognition Settings
SPEECH_RECOGNITION_LANGUAGE=en
SPEECH_RECOGNITION_ENERGY_THRESHOLD=300
SPEECH_RECOGNITION_PAUSE_THRESHOLD=0.5
SPEECH_RECOGNITION_DYNAMIC_ENERGY=true

# Speech Synthesis Settings
SPEECH_SYNTHESIS_VOICE=onyx
SPEECH_SYNTHESIS_RATE=200
SPEECH_SYNTHESIS_VOLUME=1.0

# Whisper Settings (offline speech-to-text)
WHISPER_MODEL=small
```

**Location**: `.env` file is located in the project root (`/home/dan/Nevil-picar-v2/.env`)

## Related Documentation

- [Launch System Guide](launch_system_guide.md) - Complete system launch procedures
- [Python Path Solutions](PYTHON_PATH_SOLUTIONS.md) - Detailed Python path troubleshooting
- [Build Status](build/BUILD_STATUS.md) - Current system build status

---

**Note**: This configuration is critical for both hardware nodes (`rt_motor_control_node`, `rt_sensor_node`) to access the physical picar-x robot hardware AND for AI interface nodes to access OpenAI services for language processing.