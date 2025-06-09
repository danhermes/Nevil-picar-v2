cmake_minimum_required(VERSION 3.8)
project(nevil_navigation)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nevil_interfaces REQUIRED)
find_package(nevil_core REQUIRED)
find_package(nevil_realtime REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# TF2 dependencies
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Include directories
include_directories(include)

# Create C++ executable
add_executable(motion_control_node src/motion_control_node.cpp)
ament_target_dependencies(motion_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nevil_interfaces
  nevil_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
)

# Install C++ executables
install(TARGETS
  motion_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/navigation_node.py
  scripts/test_tf2_imports.py
  nevil_navigation/nevil_navigation_api/examples/basic_movement.py
  nevil_navigation/nevil_navigation_api/examples/complex_navigation.py
  nevil_navigation/nevil_navigation_api/examples/sensor_integration.py
  nevil_navigation/nevil_navigation_api/examples/error_handling.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation
install(FILES
  nevil_navigation/nevil_navigation_api/README.md
  DESTINATION share/${PROJECT_NAME}/nevil_navigation_api
)

# Install include directories
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()