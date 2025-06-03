cmake_minimum_required(VERSION 3.8)
project(nevil_interfaces_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nevil_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AICommand.msg"
  "msg/AIStatus.msg"
  "srv/AIQuery.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nevil_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(PROGRAMS
  scripts/ai_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
