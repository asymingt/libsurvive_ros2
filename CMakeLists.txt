# Copyright 2022 Andrew Symington
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Declare the minimum cmake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project(libsurvive_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set compile options
add_compile_options(-Wall -Wextra -Wpedantic)

# Find the core interface library
include(ExternalProject)
ExternalProject_Add(libsurvive
  GIT_REPOSITORY https://github.com/cntools/libsurvive.git
  GIT_TAG v1.01
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Universally add this components includes
include_directories(include)

# Add the component
add_library(libsurvive_ros2_component SHARED
  src/component.cpp)
add_dependencies(libsurvive_ros2_component libsurvive)
target_include_directories(libsurvive_ros2_component PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include/libsurvive
  ${CMAKE_CURRENT_BINARY_DIR}/include/libsurvive/redist)
target_link_directories(libsurvive_ros2_component PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/lib)
target_link_libraries(libsurvive_ros2_component -lsurvive)
target_compile_definitions(libsurvive_ros2_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(libsurvive_ros2_component
  rclcpp
  rclcpp_components
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros)
rclcpp_components_register_nodes(libsurvive_ros2_component "libsurvive_ros2::Component")

# Export the library path because the package installs libraries without exporting them
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Manually compose the component into a node
add_executable(libsurvive_ros2_node
  src/node.cpp)
target_link_libraries(libsurvive_ros2_node
  libsurvive_ros2_component)
ament_target_dependencies(libsurvive_ros2_node rclcpp)

# Install the ROS2 interface and libsurvive.
install(
  TARGETS
    libsurvive_ros2_component
    libsurvive_ros2_node
  DESTINATION
    lib/${PROJECT_NAME})

# Install the example configuration file.
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME})

# this must happen before the invocation of ament_package()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export all include directories and declare the package
ament_export_include_directories(include)
ament_package()

