cmake_minimum_required(VERSION 3.8)
project(nevil_interfaces_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nevil_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
#install(PROGRAMS
#  nevil_interfaces_ai/ai_interface_node.py
#  nevil_interfaces_ai/dialog_manager_node.py
#  nevil_interfaces_ai/speech_recognition_node.py
#  nevil_interfaces_ai/speech_synthesis_node.py
#  nevil_interfaces_ai/text_command_processor.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()
