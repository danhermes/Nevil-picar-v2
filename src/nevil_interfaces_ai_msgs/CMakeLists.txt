cmake_minimum_required(VERSION 3.8)
project(nevil_interfaces_ai_msgs)

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
  "action/ProcessDialog.action"
  "srv/AIQuery.srv"
  "srv/QueryCapabilities.srv"
  "srv/TranslateCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs nevil_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
