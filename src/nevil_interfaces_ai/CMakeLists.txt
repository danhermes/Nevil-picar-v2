cmake_minimum_required(VERSION 3.8)
project(nevil_interfaces_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nevil_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TextCommand.msg"
  "msg/TextResponse.msg"
  "msg/VoiceCommand.msg"
  "msg/VoiceResponse.msg"
  "msg/DialogState.msg"
  "srv/QueryCapabilities.srv"
  "srv/TranslateCommand.srv"
  "action/ProcessDialog.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs nevil_interfaces
)

# Include directories
include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/text_command_processor.py
  scripts/speech_recognition_node.py
  scripts/speech_synthesis_node.py
  scripts/dialog_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()