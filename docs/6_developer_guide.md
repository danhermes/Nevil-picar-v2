# Nevil-picar v2.0: Developer Guide

This guide provides information for developers who want to extend or modify the Nevil-picar v2.0 system. It covers code organization, development workflows, testing procedures, and guidelines for contributing to the project.

## Table of Contents

- [Code Organization](#code-organization)
- [Development Environment Setup](#development-environment-setup)
- [Building and Testing](#building-and-testing)
- [Creating New Nodes](#creating-new-nodes)
- [Extending Existing Functionality](#extending-existing-functionality)
- [Real-Time Development](#real-time-development)
- [Simulation Development](#simulation-development)
- [AI Integration](#ai-integration)
- [Testing Framework](#testing-framework)
- [Contribution Guidelines](#contribution-guidelines)

## Code Organization

The Nevil-picar v2.0 codebase is organized as a ROS2 workspace with multiple packages:

```
nevil-picar-v2/
├── src/
│   ├── nevil_core/             # Core system components
│   ├── nevil_interfaces/       # Message, service, and action definitions
│   ├── nevil_interfaces_ai/    # AI-specific interface definitions
│   ├── nevil_navigation/       # Navigation and motion control
│   ├── nevil_perception/       # Camera vision and sensor processing
│   ├── nevil_realtime/         # Real-time components and utilities
│   ├── nevil_simulation/       # Digital twin simulation
│   └── nevil_testing/          # Testing framework and tests
├── docs/                       # Documentation
└── init_docs/                  # Initial project documentation
```

### Package Structure

Each package follows a standard ROS2 package structure:

```
nevil_package/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── setup.py                    # Python package setup
├── setup.cfg                   # Python package configuration
├── include/                    # C++ header files
│   └── nevil_package/
├── src/                        # C++ source files
├── nevil_package/              # Python modules
├── scripts/                    # Executable Python scripts
├── launch/                     # Launch files
├── config/                     # Configuration files
├── resource/                   # Resource files
└── test/                       # Test files
```

### Key Components

- **System Manager**: Coordinates the overall system behavior and manages node lifecycle
- **Motion Control**: Controls the robot's movement and behaviors
- **Obstacle Avoidance**: Processes sensor data to detect and avoid obstacles
- **Navigation**: Handles path planning and execution
- **Camera Vision**: Processes camera input for object detection and tracking
- **Voice Control**: Manages speech recognition and synthesis
- **AI Processing**: Handles AI processing and decision-making
- **Digital Twin**: Provides a simulated environment for development and testing

## Development Environment Setup

### Prerequisites

- Ubuntu 22.04 or Raspberry Pi OS (64-bit)
- ROS2 Humble
- Python 3.8+
- C++ compiler (GCC 9+)
- Git

### Setting Up the Development Environment

1. Install ROS2 Humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)

2. Install additional dependencies:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

3. Create a workspace:
   ```bash
   mkdir -p ~/nevil_ws/src
   cd ~/nevil_ws/src
   git clone https://github.com/username/nevil-picar-v2.git
   ```

4. Install dependencies:
   ```bash
   cd ~/nevil_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace:
   ```bash
   cd ~/nevil_ws
   colcon build
   source install/setup.bash
   ```

### Development Tools

- **Visual Studio Code**: Recommended IDE with the following extensions:
  - ROS extension
  - Python extension
  - C/C++ extension
  - YAML extension

- **rqt**: ROS2 GUI tools for debugging and visualization

- **rviz2**: ROS2 3D visualization tool

## Building and Testing

### Building the Project

```bash
cd ~/nevil_ws
colcon build
source install/setup.bash
```

To build specific packages:

```bash
colcon build --packages-select nevil_core nevil_navigation
```

To build with debug symbols:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests

To run all tests:

```bash
cd ~/nevil_ws
colcon test
colcon test-result --verbose
```

To run tests for specific packages:

```bash
colcon test --packages-select nevil_core
```

To run a specific test:

```bash
cd ~/nevil_ws
python3 src/nevil_testing/test/unit/core/test_system_manager_node.py
```

### Continuous Integration

The project uses GitHub Actions for continuous integration. Each pull request is automatically tested with the following workflow:

1. Build the project
2. Run unit tests
3. Run integration tests
4. Run linting checks

## Creating New Nodes

### Creating a New Python Node

1. Create a new Python file in the appropriate package:
   ```bash
   cd ~/nevil_ws/src/nevil_package/nevil_package
   touch my_new_node.py
   ```

2. Implement the node using the ROS2 Python API:
   ```python
   import rclpy
   from rclpy.node import Node

   class MyNewNode(Node):
       def __init__(self):
           super().__init__('my_new_node')
           self.timer = self.create_timer(1.0, self.timer_callback)
           self.get_logger().info('MyNewNode initialized')
           
       def timer_callback(self):
           self.get_logger().info('Timer callback')

   def main(args=None):
       rclpy.init(args=args)
       node = MyNewNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Create an entry point in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'my_new_node = nevil_package.my_new_node:main',
       ],
   },
   ```

4. Build and run the node:
   ```bash
   cd ~/nevil_ws
   colcon build --packages-select nevil_package
   source install/setup.bash
   ros2 run nevil_package my_new_node
   ```

### Creating a New C++ Node

1. Create new C++ files in the appropriate package:
   ```bash
   cd ~/nevil_ws/src/nevil_package
   touch include/nevil_package/my_new_node.hpp
   touch src/my_new_node.cpp
   ```

2. Implement the node using the ROS2 C++ API:
   ```cpp
   // include/nevil_package/my_new_node.hpp
   #ifndef NEVIL_PACKAGE__MY_NEW_NODE_HPP_
   #define NEVIL_PACKAGE__MY_NEW_NODE_HPP_

   #include "rclcpp/rclcpp.hpp"

   namespace nevil_package
   {

   class MyNewNode : public rclcpp::Node
   {
   public:
     MyNewNode();

   private:
     void timer_callback();
     rclcpp::TimerBase::SharedPtr timer_;
   };

   }  // namespace nevil_package

   #endif  // NEVIL_PACKAGE__MY_NEW_NODE_HPP_
   ```

   ```cpp
   // src/my_new_node.cpp
   #include "nevil_package/my_new_node.hpp"

   namespace nevil_package
   {

   MyNewNode::MyNewNode()
   : Node("my_new_node")
   {
     timer_ = this->create_wall_timer(
       std::chrono::seconds(1),
       std::bind(&MyNewNode::timer_callback, this));
     RCLCPP_INFO(this->get_logger(), "MyNewNode initialized");
   }

   void MyNewNode::timer_callback()
   {
     RCLCPP_INFO(this->get_logger(), "Timer callback");
   }

   }  // namespace nevil_package

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<nevil_package::MyNewNode>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }
   ```

3. Update `CMakeLists.txt`:
   ```cmake
   add_executable(my_new_node src/my_new_node.cpp)
   ament_target_dependencies(my_new_node rclcpp)
   target_include_directories(my_new_node PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)
   install(TARGETS my_new_node
     DESTINATION lib/${PROJECT_NAME})
   ```

4. Build and run the node:
   ```bash
   cd ~/nevil_ws
   colcon build --packages-select nevil_package
   source install/setup.bash
   ros2 run nevil_package my_new_node
   ```

### Creating a Launch File

1. Create a new launch file in the appropriate package:
   ```bash
   cd ~/nevil_ws/src/nevil_package/launch
   touch my_new_launch.py
   ```

2. Implement the launch file:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='nevil_package',
               executable='my_new_node',
               name='my_new_node',
               output='screen',
               parameters=[{
                   'param1': 42,
                   'param2': 'hello'
               }]
           ),
           # Additional nodes...
       ])
   ```

3. Build and run the launch file:
   ```bash
   cd ~/nevil_ws
   colcon build --packages-select nevil_package
   source install/setup.bash
   ros2 launch nevil_package my_new_launch.py
   ```

## Extending Existing Functionality

### Adding a New Message, Service, or Action

1. Create a new message definition in `nevil_interfaces` or `nevil_interfaces_ai`:
   ```bash
   cd ~/nevil_ws/src/nevil_interfaces/msg
   touch MyNewMessage.msg
   ```

2. Define the message structure:
   ```
   # MyNewMessage.msg
   string data
   int32 value
   bool flag
   ```

3. Update `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/MyNewMessage.msg"
     # Other message, service, and action files...
   )
   ```

4. Build the package:
   ```bash
   cd ~/nevil_ws
   colcon build --packages-select nevil_interfaces
   source install/setup.bash
   ```

5. Use the new message in your code:
   ```python
   from nevil_interfaces.msg import MyNewMessage

   msg = MyNewMessage()
   msg.data = "Hello"
   msg.value = 42
   msg.flag = True
   publisher.publish(msg)
   ```

### Modifying an Existing Node

1. Identify the node you want to modify
2. Understand its functionality by reading the code and documentation
3. Make your changes
4. Test your changes thoroughly
5. Submit a pull request with your changes

### Adding a New Feature

1. Identify where the feature should be implemented
2. Design the feature, considering its interactions with existing components
3. Implement the feature
4. Write tests for the feature
5. Document the feature
6. Submit a pull request with your changes

## Real-Time Development

### Understanding PREEMPT-RT

The PREEMPT-RT patch makes the Linux kernel fully preemptible, which means that high-priority tasks can interrupt lower-priority tasks, even if the lower-priority task is executing kernel code. This is essential for real-time performance in robotics applications.

### Real-Time Best Practices

1. **Avoid Blocking Operations**: Blocking operations can cause priority inversion and affect real-time performance
2. **Use ROS2 Timers**: Instead of `sleep()` or `time.sleep()`, use ROS2 timers for periodic execution
3. **Minimize Memory Allocation**: Dynamic memory allocation can cause unpredictable latency
4. **Use Real-Time Priority Scheduling**: Assign appropriate priorities to critical tasks
5. **Measure and Monitor Latency**: Use tools like `cyclictest` to measure and monitor latency

### Implementing Real-Time Nodes

1. Use the `RTExecutor` class from `nevil_realtime`:
   ```python
   from nevil_realtime.rt_executor import RTExecutor
   from rclpy.node import Node

   class MyRTNode(Node):
       def __init__(self):
           super().__init__('my_rt_node')
           # Node implementation...

   def main(args=None):
       rclpy.init(args=args)
       node = MyRTNode()
       executor = RTExecutor(priority=80)
       executor.add_node(node)
       executor.spin()
       node.destroy_node()
       rclpy.shutdown()
   ```

2. Use the `rt_thread_utils` module for real-time thread management:
   ```cpp
   #include "nevil_realtime/rt_thread_utils.hpp"

   // Set thread priority
   nevil_realtime::set_thread_priority(80);

   // Lock memory to prevent paging
   nevil_realtime::lock_memory();

   // Run real-time task
   nevil_realtime::run_rt_task([]() {
     // Real-time task implementation...
   });
   ```

## Simulation Development

### Digital Twin Architecture

The digital twin simulation provides a virtual representation of the physical robot, allowing for development and testing without requiring physical hardware.

### Extending the Simulation

1. **Adding New Environments**:
   - Create a new environment YAML file in `nevil_simulation/environments/`
   - Define the environment properties, obstacles, and other elements
   - Register the environment in `simulation_config.yaml`

2. **Adding New Sensor Simulations**:
   - Extend the `SimulationHardwareInterface` class in `nevil_simulation`
   - Implement the sensor simulation logic
   - Update the hardware abstraction layer to use the new sensor

3. **Customizing Physics Simulation**:
   - Modify the `PhysicsEngine` class in `nevil_simulation`
   - Adjust physics parameters in `simulation_config.yaml`

### Testing with Simulation

1. Launch the simulation:
   ```bash
   ros2 launch nevil_simulation nevil_simulation.launch.py
   ```

2. Launch the system with simulation:
   ```bash
   ros2 launch nevil_simulation nevil_system_with_simulation.launch.py
   ```

3. Visualize the simulation:
   ```bash
   ros2 run nevil_simulation visualization_node
   ```

## AI Integration

### Online AI Integration

1. Set up the OpenAI API key:
   ```bash
   export OPENAI_API_KEY="your_api_key_here"
   ```

2. Configure the AI settings in `ai_config.yaml`

3. Use the AI API in your code:
   ```python
   from nevil_interfaces_ai.ai_processing_node import OnlineAIAPI

   online_ai = OnlineAIAPI()
   response = online_ai.generate_text("Hello, Nevil!")
   ```

### Local AI Integration

1. Download the local AI model:
   ```bash
   cd ~/nevil_ws/models
   # Download Gemma 2 or TinyLlama model
   ```

2. Configure the AI settings in `ai_config.yaml`:
   ```yaml
   ai:
     offline:
       provider: "gemma"
       model_path: "/path/to/model"
   ```

3. Use the AI API in your code:
   ```python
   from nevil_interfaces_ai.ai_processing_node import OfflineAIAPI

   offline_ai = OfflineAIAPI()
   response = offline_ai.generate_text("Hello, Nevil!")
   ```

### Hybrid AI Approach

The system uses a hybrid approach to AI processing, combining cloud-based and local AI models:

1. **Cloud-Based Processing**: Used for advanced language understanding, high-quality TTS, and complex reasoning
2. **Local Processing**: Used for offline operation, low-latency tasks, and basic cognition
3. **Seamless Switching**: The system automatically switches between cloud and local processing based on network connectivity, task complexity, and response time requirements

## Testing Framework

### Unit Testing

1. Create a new unit test in the appropriate package:
   ```bash
   cd ~/nevil_ws/src/nevil_testing/test/unit/package_name
   touch test_my_component.py
   ```

2. Implement the unit test:
   ```python
   import unittest
   from nevil_testing.test_base import TestBase
   from nevil_package.my_component import MyComponent

   class TestMyComponent(TestBase):
       def setUp(self):
           super().setUp()
           self.component = MyComponent()
           
       def test_functionality(self):
           result = self.component.do_something()
           self.assertEqual(result, expected_result)
           
   if __name__ == '__main__':
       unittest.main()
   ```

3. Run the unit test:
   ```bash
   cd ~/nevil_ws
   python3 src/nevil_testing/test/unit/package_name/test_my_component.py
   ```

### Integration Testing

1. Create a new integration test:
   ```bash
   cd ~/nevil_ws/src/nevil_testing/test/integration
   touch test_my_integration.py
   ```

2. Implement the integration test:
   ```python
   import unittest
   from nevil_testing.test_base import TestBase
   import rclpy
   from rclpy.node import Node

   class TestMyIntegration(TestBase):
       def setUp(self):
           super().setUp()
           rclpy.init()
           self.node = Node('test_node')
           
       def tearDown(self):
           self.node.destroy_node()
           rclpy.shutdown()
           super().tearDown()
           
       def test_integration(self):
           # Test integration between components
           pass
           
   if __name__ == '__main__':
       unittest.main()
   ```

### Simulation Testing

1. Create a new simulation test:
   ```bash
   cd ~/nevil_ws/src/nevil_testing/test/simulation
   touch test_my_simulation.py
   ```

2. Implement the simulation test:
   ```python
   import unittest
   from nevil_testing.simulation_test_base import SimulationTestBase

   class TestMySimulation(SimulationTestBase):
       def setUp(self):
           super().setUp()
           self.start_simulation()
           
       def tearDown(self):
           self.stop_simulation()
           super().tearDown()
           
       def test_simulation(self):
           # Test behavior in simulation
           pass
           
   if __name__ == '__main__':
       unittest.main()
   ```

### Hardware Testing

1. Create a new hardware test:
   ```bash
   cd ~/nevil_ws/src/nevil_testing/test/hardware
   touch test_my_hardware.py
   ```

2. Implement the hardware test:
   ```python
   import unittest
   from nevil_testing.hardware_test_base import HardwareTestBase

   class TestMyHardware(HardwareTestBase):
       def setUp(self):
           super().setUp()
           self.initialize_hardware()
           
       def tearDown(self):
           self.cleanup_hardware()
           super().tearDown()
           
       def test_hardware(self):
           # Test hardware functionality
           pass
           
   if __name__ == '__main__':
       unittest.main()
   ```

## Contribution Guidelines

### Code Style

- **Python**: Follow PEP 8 style guide
- **C++**: Follow the ROS2 C++ style guide
- **Documentation**: Use clear, concise language and proper formatting
- **Commit Messages**: Use descriptive commit messages with the format: `[Package] Brief description`

### Pull Request Process

1. Fork the repository
2. Create a new branch for your feature or bug fix
3. Make your changes
4. Write or update tests for your changes
5. Ensure all tests pass
6. Update documentation as needed
7. Submit a pull request

### Code Review Process

1. All pull requests must be reviewed by at least one maintainer
2. Automated tests must pass
3. Code must follow the style guidelines
4. Documentation must be updated as needed

### Documentation Guidelines

1. Use Markdown for documentation
2. Include examples and diagrams where appropriate
3. Keep documentation up-to-date with code changes
4. Document public APIs, classes, and functions

## Conclusion

This developer guide provides the information needed to extend and modify the Nevil-picar v2.0 system. By following these guidelines, you can contribute to the project and create new functionality while maintaining compatibility with the existing system.

For more detailed information on specific components, refer to the [API Reference](5_api_reference.md) and the source code documentation.