cmake_minimum_required(VERSION 3.8)
project(nevil_testing)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nevil_interfaces REQUIRED)
find_package(nevil_interfaces_ai REQUIRED)
find_package(nevil_core REQUIRED)
find_package(nevil_navigation REQUIRED)
find_package(nevil_perception REQUIRED)
find_package(nevil_realtime REQUIRED)
find_package(nevil_simulation REQUIRED)

# Include directories
include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/run_unit_tests.py
  scripts/run_integration_tests.py
  scripts/run_system_tests.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install test directories
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add C++ unit tests
  find_package(ament_cmake_gtest REQUIRED)
  
  # Core unit tests
  ament_add_gtest(test_system_manager_node
    test/unit/core/test_system_manager_node.cpp
  )
  target_link_libraries(test_system_manager_node
    ${rclcpp_LIBRARIES}
    ${nevil_core_LIBRARIES}
  )
  
  # Navigation unit tests
  ament_add_gtest(test_motion_control_node
    test/unit/navigation/test_motion_control_node.cpp
  )
  target_link_libraries(test_motion_control_node
    ${rclcpp_LIBRARIES}
    ${nevil_navigation_LIBRARIES}
  )
  
  # Perception unit tests
  ament_add_gtest(test_camera_vision_node
    test/unit/perception/test_camera_vision_node.cpp
  )
  target_link_libraries(test_camera_vision_node
    ${rclcpp_LIBRARIES}
    ${nevil_perception_LIBRARIES}
  )
  
  # Realtime unit tests
  ament_add_gtest(test_rt_executor
    test/unit/realtime/test_rt_executor.cpp
  )
  target_link_libraries(test_rt_executor
    ${rclcpp_LIBRARIES}
    ${nevil_realtime_LIBRARIES}
  )
  
  # Add Python tests
  find_package(ament_cmake_pytest REQUIRED)
  
  # Add launch tests
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # Integration tests
  add_launch_test(
    test/integration/test_navigation_system.py
  )
  
  # System tests
  add_launch_test(
    test/system/test_end_to_end_navigation.py
  )
endif()

ament_package()