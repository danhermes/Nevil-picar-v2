# Building and Using Environment Variables in Nevil-picar v2.0

This document explains how to build and use the environment variable support in the Nevil-picar v2.0 system, particularly for the audio hardware interface.

## Table of Contents

1. [Overview](#overview)
2. [Environment Variable Configuration](#environment-variable-configuration)
3. [Building the Package](#building-the-package)
4. [Running the Tests](#running-the-tests)
5. [Why Rebuilding is Necessary](#why-rebuilding-is-necessary)
6. [Development Best Practices](#development-best-practices)

## Overview

The Nevil-picar v2.0 system now supports configuration via environment variables stored in a root-level `.env` file. This provides a more secure and flexible way to configure the system, especially for sensitive information like API keys.

Key features of the environment variable support:

- Configuration of speech recognition parameters
- Configuration of speech synthesis parameters
- Configuration of Whisper model for speech recognition
- Configuration of OpenAI API key for language processing (not needed for Whisper)
- Fallback mechanisms for when environment variables are not set

## Environment Variable Configuration

### The `.env` File

Create a `.env` file in the root directory of the project with the following content:

```
# OpenAI API Keys (for language processing, not needed for Whisper)
OPENAI_API_KEY=your_openai_api_key_here

# Speech Recognition Settings
SPEECH_RECOGNITION_LANGUAGE=en
SPEECH_RECOGNITION_ENERGY_THRESHOLD=300
SPEECH_RECOGNITION_PAUSE_THRESHOLD=0.5
SPEECH_RECOGNITION_DYNAMIC_ENERGY=true

# Speech Synthesis Settings
SPEECH_SYNTHESIS_VOICE=onyx
SPEECH_SYNTHESIS_RATE=200
SPEECH_SYNTHESIS_VOLUME=1.0

# Whisper Settings (offline speech-to-text, no API key needed)
WHISPER_MODEL=small  # Options: tiny, base, small, medium, large
```

### Available Configuration Options

| Environment Variable | Description | Default Value |
|----------------------|-------------|---------------|
| `OPENAI_API_KEY` | OpenAI API key for language processing (not needed for Whisper) | None |
| `SPEECH_RECOGNITION_LANGUAGE` | Language code for speech recognition | "en" |
| `SPEECH_RECOGNITION_ENERGY_THRESHOLD` | Energy threshold for speech detection | 300 |
| `SPEECH_RECOGNITION_PAUSE_THRESHOLD` | Pause threshold for speech detection | 0.5 |
| `SPEECH_RECOGNITION_DYNAMIC_ENERGY` | Whether to use dynamic energy threshold | true |
| `SPEECH_SYNTHESIS_VOICE` | Voice ID for speech synthesis | "onyx" |
| `SPEECH_SYNTHESIS_RATE` | Speech rate in words per minute | 200 |
| `SPEECH_SYNTHESIS_VOLUME` | Speech volume (0.0 to 1.0) | 1.0 |
| `WHISPER_MODEL` | Whisper model to use (offline, no API key needed) | "small" |

### Notes on OpenAI API Key

It's important to note that the OpenAI API key is **not** needed for Whisper speech recognition, which runs offline. The API key is only needed if you're using OpenAI's language processing services for other parts of the system.

## Building the Package

After making changes to the code or configuration, you need to rebuild the package for the changes to take effect. This is because ROS2 installs packages into a specific location during the build process, and the installed files are what get used at runtime.

### Prerequisites

- ROS2 Humble or later
- Python 3.8 or later
- Required Python packages:
  - `python-dotenv`
  - `speech_recognition`
  - `pyaudio`
  - `whisper` (optional, for improved speech recognition)
  - `pyttsx3` (optional, fallback for speech synthesis)
  - `robot_hat` (for hardware access)

### Build Steps

1. Navigate to the workspace root directory:
   ```bash
   cd /path/to/nevil-picar-v2
   ```

2. Build the package:
   ```bash
   colcon build --packages-select nevil_interfaces_ai
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Development Mode

For faster development iterations, you can use the `--symlink-install` option, which creates symlinks to the source files instead of copying them:

```bash
colcon build --packages-select nevil_interfaces_ai --symlink-install
```

This way, changes to Python files will be immediately reflected without needing to rebuild (except for changes to `setup.py` or other build-related files).

## Running the Tests

### Testing Environment Variable Loading

To test that environment variables are being loaded correctly:

```bash
ros2 run nevil_interfaces_ai test_env_loading
```

This will print out the values of all environment variables loaded from the `.env` file.

### Testing Audio Hardware Interface

To test the audio hardware interface:

```bash
ros2 run nevil_interfaces_ai test_audio_hardware
```

This will run a series of tests on the audio hardware interface, including:
- Speech synthesis
- Speech recognition
- Audio recording and playback

### Running the Nodes

To run the speech recognition and synthesis nodes:

```bash
ros2 run nevil_interfaces_ai speech_recognition_node
ros2 run nevil_interfaces_ai speech_synthesis_node
```

## Why Rebuilding is Necessary

In ROS2, Python packages need to be rebuilt after changes for several important reasons:

### ROS2 Package System

ROS2 uses a structured package system where:

1. **Source vs. Install Directories**: Your source code lives in the `src/` directory, but when you build the package, it gets installed to the `install/` directory.

2. **Python Import Path**: When you run a ROS2 node, it looks for Python modules in the installed location, not your source files. This is set up when you source the setup script (`source install/setup.bash`).

3. **Package Discovery**: ROS2 uses a package index to find and load packages at runtime. This index is updated during the build process.

### What Happens During Build

When you run `colcon build`:

1. The Python files are copied from `src/your_package/` to `install/your_package/lib/python3.x/site-packages/your_package/`
2. Entry points defined in `setup.py` are registered as executable scripts
3. Package metadata is updated in the ROS2 package index

### Common Issues

If you're getting import errors or your changes don't seem to be taking effect, it's likely because:

1. You haven't rebuilt the package after making changes
2. You haven't sourced the setup file after rebuilding
3. You're trying to run the Python files directly instead of using `ros2 run`

## Development Best Practices

### Environment Variables

1. **Never hardcode sensitive information** like API keys in your code. Use environment variables instead.
2. **Don't commit your `.env` file** to version control. Add it to `.gitignore`.
3. **Provide a sample `.env.example` file** with placeholder values.
4. **Document all environment variables** in your README or documentation.

### ROS2 Development

1. **Use `--symlink-install`** for faster development iterations.
2. **Source the setup file** after rebuilding.
3. **Use `ros2 run`** to run your nodes, not `python3 your_script.py`.
4. **Use ROS2 parameters** for configuration that needs to be changed at runtime.
5. **Use environment variables** for configuration that is set once and rarely changes.

### Testing

1. **Write unit tests** for your code.
2. **Use ROS2's built-in testing framework** for integration tests.
3. **Test with both real hardware and simulation** to ensure your code works in all environments.
4. **Test with different configuration values** to ensure your code handles all cases.

## Conclusion

By following these guidelines, you can effectively use environment variables in your Nevil-picar v2.0 system and avoid common pitfalls in ROS2 development.