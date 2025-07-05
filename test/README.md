# Test Directory Structure

This directory contains all test files for the Nevil PiCar-X project, organized by category for better maintainability and clarity.

## Directory Structure

### `/test/navigation/`
Navigation and movement-related tests:
- `test_navigation_node_complete.py` - Complete navigation node testing
- `test_nevil_navigation_api.py` - Navigation API testing
- `test_nevil_node_movement.py` - Node movement functionality
- `test_nevil_real_movement.py` - Real hardware movement tests
- `test_final_node_movement.py` - Final movement validation
- `test_picar_actions_real_movement.py` - PiCar action movement tests
- `test_simple_picar_actions.py` - Basic PiCar action tests
- `test_tf2_imports.py` - TF2 transformation imports test

### `/test/hardware/`
Hardware interface and integration tests:
- `test_direct_hardware.py` - Direct hardware interface tests
- `test_hardware_architecture.py` - Hardware architecture validation
- `test_hardware_initialization.py` - Hardware initialization tests
- `test_picar_hardware_logging.py` - PiCar hardware logging tests
- `test_picarx_integration.py` - PiCar-X integration tests
- `test_real_hardware_logging.py` - Real hardware logging validation
- `setup_picarx.py` - PiCar-X setup and configuration

### `/test/ai/`
AI, speech, and command processing tests:
- `test_ai_command_import.py` - AI command import functionality
- `test_whisper_tts.py` - Whisper TTS integration tests
- `run_test_dialog_manager.py` - Dialog manager test runner
- `run_dialog_manager.py` - Dialog manager execution
- `run_speech_interface.py` - Speech interface runner
- `test_env_loading.py` - Environment loading tests
- `test_audio_hardware.py` - Audio hardware functionality
- `test_navigation_actions.py` - Navigation action tests
- `test_action_execution.py` - Action execution tests
- `test_openai_message_flow.py` - OpenAI message flow tests
- `test_speech_coordination.py` - Speech coordination tests
- `test_integration.py` - AI integration tests

### `/test/integration/`
System integration and workflow tests:
- `test_action_workflow.py` - Action workflow integration
- `run_unit_tests_modified.py` - Modified unit test runner
- `generate_test_coverage.py` - Test coverage generation
- `integration_test.py` - Main integration test suite

### `/test/unit/`
Unit tests and isolated component tests:
- `test_force_success_logging.py` - Force success logging tests
- `test_picar_success_logging.py` - PiCar success logging tests
- `test_ros2_env.sh` - ROS2 environment validation script
- `test_launch_command.sh` - Launch command testing script
- `nevil_node_test_results.md` - Node test results documentation
- `run_hardware_test.sh` - Hardware test execution script
- `test_recording.wav` - Test audio recording file
- `test.c` - C language test file
- `test_dialog_manager_node.py` - Dialog manager node tests
- `test_system_mode.py` - System mode tests

### `/test/docs/`
Testing documentation and guides:
- `navigation_servo_fix_testing.md` - Navigation servo fix testing guide

## Running Tests

### Navigation Tests
```bash
cd test/navigation
python3 test_navigation_node_complete.py
```

### Hardware Tests
```bash
cd test/hardware
python3 test_direct_hardware.py
```

### AI Tests
```bash
cd test/ai
python3 test_ai_command_import.py
```

### Integration Tests
```bash
cd test/integration
python3 integration_test.py
```

### Unit Tests
```bash
cd test/unit
./test_ros2_env.sh
```

## Test Organization Benefits

1. **Clear Separation**: Tests are organized by functional area
2. **Easy Navigation**: Developers can quickly find relevant tests
3. **Maintainability**: Related tests are grouped together
4. **Scalability**: New tests can be easily categorized
5. **Documentation**: Each category has clear purpose and scope

## Adding New Tests

When adding new tests, place them in the appropriate category:
- Navigation/movement functionality → `/test/navigation/`
- Hardware interfaces → `/test/hardware/`
- AI/speech processing → `/test/ai/`
- System integration → `/test/integration/`
- Isolated components → `/test/unit/`
- Test documentation → `/test/docs/`

## Test Dependencies

Most tests require the Nevil packages to be built and sourced:
```bash
cd /home/dan/Nevil-picar-v2
colcon build
source install/setup.bash
```

For hardware tests, ensure the PiCar-X is properly connected and configured.