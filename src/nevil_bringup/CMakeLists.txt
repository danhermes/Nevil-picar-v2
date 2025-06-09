cmake_minimum_required(VERSION 3.8)
project(nevil_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/nevil_cli.py
  scripts/system_monitor.py
  scripts/system_monitor_wrapper
  DESTINATION lib/${PROJECT_NAME}
)

# Create symbolic link for nevil_cli
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nevil_cli.py
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nevil_cli
)")

# Install test files
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()