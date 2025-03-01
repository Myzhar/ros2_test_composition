cmake_minimum_required(VERSION 3.5)
project(ros2_test_composition)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# INSTALL

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()