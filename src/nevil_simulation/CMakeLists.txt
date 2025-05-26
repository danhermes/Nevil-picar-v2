cmake_minimum_required(VERSION 3.8)
project(nevil_simulation)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nevil_interfaces REQUIRED)
find_package(nevil_core REQUIRED)
find_package(nevil_navigation REQUIRED)
find_package(nevil_perception REQUIRED)
find_package(nevil_realtime REQUIRED)

# Include directories
include_directories(include)

# C++ libraries and executables
add_library(simulation_hardware_interface
  src/simulation_hardware_interface.cpp
)
ament_target_dependencies(simulation_hardware_interface
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nevil_interfaces
  nevil_realtime
)

add_library(physics_engine
  src/physics_engine.cpp
)
ament_target_dependencies(physics_engine
  rclcpp
)

add_executable(simulation_node
  src/simulation_node.cpp
)
ament_target_dependencies(simulation_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  nevil_interfaces
)
target_link_libraries(simulation_node
  simulation_hardware_interface
  physics_engine
)

add_executable(visualization_node
  src/visualization_node.cpp
)
ament_target_dependencies(visualization_node
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
)

# Install C++ executables
install(TARGETS
  simulation_hardware_interface
  physics_engine
  simulation_node
  visualization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install C++ header files
install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/simulation_manager.py
  scripts/environment_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install environments
install(DIRECTORY
  environments
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()