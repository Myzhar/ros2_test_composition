cmake_minimum_required(VERSION 3.5)
project(ros2_test_composition_components)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# SOURCES
set(TIMER_COMPONENT_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/timer_component.hpp
)

set(TIMER_COMPONENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/timer_component.cpp
)

set(THREAD_COMPONENT_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/thread_component.hpp
)

set(THREAD_COMPONENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/thread_component.cpp
)

# INCLUDES
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# COMPONENTS
add_library(timer_component SHARED
    ${TIMER_COMPONENT_INC}
    ${TIMER_COMPONENT_SRC}
)
target_compile_definitions(timer_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(timer_component
    rclcpp
    rclcpp_components
)
rclcpp_components_register_nodes(timer_component "tc::TimerComponent")
set(node_plugins "${node_plugins}tc::TimerComponent;$<TARGET_FILE:timer_component>\n")

add_library(thread_component SHARED
    ${THREAD_COMPONENT_INC}
    ${THREAD_COMPONENT_SRC}
)
target_compile_definitions(thread_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(thread_component
    rclcpp
    rclcpp_components
)
rclcpp_components_register_nodes(thread_component "tc::ThreadComponent")
set(node_plugins "${node_plugins}tc::ThreadComponent;$<TARGET_FILE:thread_component>\n")

# INSTALL

# Install components
install(TARGETS timer_component thread_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(timer_component thread_component)
ament_export_dependencies(rclcpp rclcpp_components)
ament_package()