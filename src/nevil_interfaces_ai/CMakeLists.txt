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
find_package(nevil_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AICommand.msg"
  "msg/AIStatus.msg"
  "msg/Audio.msg"
  "msg/DialogState.msg"
  "msg/TextCommand.msg"
  "msg/TextResponse.msg"
  "msg/VoiceCommand.msg"
  "msg/VoiceResponse.msg"
  "srv/AIQuery.srv"
  "srv/QueryCapabilities.srv"
  "srv/TranslateCommand.srv"
  "action/ProcessDialog.action"
  DEPENDENCIES std_msgs geometry_msgs nevil_interfaces
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
  scripts/dialog_manager_node.py
  scripts/speech_recognition_node.py
  scripts/speech_synthesis_node.py
  scripts/text_command_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/python3/site-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
