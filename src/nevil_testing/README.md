# Nevil Testing Framework

A comprehensive testing framework for Nevil-picar v2.0 that validates all major components and their integration. The framework follows Test-Driven Development principles and ensures that the system meets the requirements specified in the project documentation.

## Overview

The Nevil Testing Framework provides:

- Unit tests for individual components
- Integration tests for component interactions
- System tests for end-to-end validation
- Performance benchmarking
- Support for both physical robot and simulation testing

## Features

- **Modular Test Structure**: Organized by component and test type
- **Simulation Support**: Test with the digital twin before deploying to hardware
- **Hardware Abstraction**: Same tests can run on both simulated and real hardware
- **Comprehensive Coverage**: Tests all aspects of the Nevil-picar v2.0 system
- **Continuous Integration**: Designed to work with CI/CD pipelines
- **Detailed Reporting**: Clear reporting of test results

## Directory Structure

```
nevil_testing/
├── config/                 # Test configuration files
├── include/                # C++ header files
├── launch/                 # Launch files for running tests
├── nevil_testing/          # Python package
│   ├── __init__.py
│   ├── test_base.py        # Base test class
│   ├── simulation_test_base.py  # Simulation test base class
│   ├── hardware_test_base.py    # Hardware test base class
│   ├── test_utils.py       # Test utilities
│   └── test_runner.py      # Test runner
├── scripts/                # Test scripts
├── src/                    # C++ source files
└── test/                   # Test files
    ├── unit/               # Unit tests
    │   ├── core/           # Core component tests
    │   ├── navigation/     # Navigation component tests
    │   ├── perception/     # Perception component tests
    │   ├── realtime/       # Real-time component tests
    │   ├── interfaces_ai/  # AI interfaces tests
    │   └── simulation/     # Simulation component tests
    ├── integration/        # Integration tests
    └── system/             # System tests
```

## Installation

1. Clone the Nevil-picar v2.0 repository:
   ```bash
   git clone https://github.com/your-username/nevil-picar-v2.git
   cd nevil-picar-v2
   ```

2. Build the packages:
   ```bash
   colcon build --packages-select nevil_testing
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running All Tests

```bash
ros2 launch nevil_testing test_launcher.launch.py
```

### Running Specific Test Types

```bash
# Run unit tests
ros2 launch nevil_testing test_launcher.launch.py test_type:=unit

# Run integration tests
ros2 launch nevil_testing test_launcher.launch.py test_type:=integration

# Run system tests
ros2 launch nevil_testing test_launcher.launch.py test_type:=system
```

### Running Tests with Simulation or Hardware

```bash
# Run with simulation (default)
ros2 launch nevil_testing test_launcher.launch.py use_simulation:=true

# Run with physical hardware
ros2 launch nevil_testing test_launcher.launch.py use_simulation:=false
```

### Running Tests for Specific Packages

```bash
# Test the navigation package
ros2 launch nevil_testing test_launcher.launch.py test_package:=navigation

# Test the perception package
ros2 launch nevil_testing test_launcher.launch.py test_package:=perception
```

### Running a Specific Test File

```bash
ros2 launch nevil_testing test_launcher.launch.py test_file:=test_motion_control_node.py
```

### Verbose Output

```bash
ros2 launch nevil_testing test_launcher.launch.py verbose:=true
```

## Writing Tests

### Unit Tests

Unit tests should be placed in the appropriate subdirectory under `test/unit/`. For example, a test for the motion control node should be placed in `test/unit/navigation/`.

```python
#!/usr/bin/env python3

import unittest
from nevil_testing.test_base import NevilTestBase

class TestMotionControlNode(NevilTestBase):
    def test_cmd_vel_subscription(self):
        # Test that the motion control node subscribes to cmd_vel
        self.assert_topic_published('/cmd_vel', Twist)
    
    def test_set_velocity(self):
        # Test setting velocity
        # ...

if __name__ == '__main__':
    unittest.main()
```

### Integration Tests

Integration tests should be placed in the `test/integration/` directory.

```python
#!/usr/bin/env python3

import unittest
from nevil_testing.simulation_test_base import SimulationTestBase

class TestNavigationSystem(SimulationTestBase):
    def test_navigation_with_obstacle_avoidance(self):
        # Test navigation with obstacle avoidance
        # ...

if __name__ == '__main__':
    unittest.main()
```

### System Tests

System tests should be placed in the `test/system/` directory.

```python
#!/usr/bin/env python3

import unittest
from nevil_testing.simulation_test_base import SimulationTestBase

class TestEndToEndNavigation(SimulationTestBase):
    def test_navigate_to_goal(self):
        # Test navigating to a goal
        # ...

if __name__ == '__main__':
    unittest.main()
```

## Configuration

The test configuration is stored in `config/test_config.yaml`. This file contains settings for:

- General test configuration
- Unit test configuration
- Integration test configuration
- System test configuration
- Performance test configuration
- Simulation configuration
- Hardware configuration

## Contributing

1. Create a new test file in the appropriate directory
2. Implement the test using the provided base classes
3. Add the test to the appropriate test runner
4. Run the test to ensure it passes
5. Submit a pull request

## License

This project is licensed under the [LICENSE NAME] - see the LICENSE file for details.