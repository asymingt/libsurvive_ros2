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

# COMPILATION OPTIONS

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

# DEPENDENCIES

# Obtain a specific version of the libsurvive low-level driver.
include(ExternalProject)
externalproject_add(libsurvive
  GIT_REPOSITORY https://github.com/cntools/libsurvive.git
  GIT_TAG d89876a1af0efcfcddeaaf2a282eec8753a1bac4
  CMAKE_ARGS
    -DCMAKE_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
)

# Obtain a specific version of the GTSAM estimation software.
include(ExternalProject)
externalproject_add(gtsam
  GIT_REPOSITORY https://github.com/borglab/gtsam.git
  GIT_TAG 621ef2e7c560e6fb06b463769d9cf7bc6f626308
  CMAKE_ARGS
    -DCMAKE_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DGTSAM_BUILD_PYTHON=1
)

# Third party libraries
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Interfaces
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Universally add this components includes
include_directories(include)

# IDL GENERATION

# Generate messages used in this project
rosidl_generate_interfaces(libsurvive_ros2
    "msg/Angle.msg"
    "msg/Lighthouse.msg"
    "msg/Tracker.msg"
  DEPENDENCIES
    diagnostic_msgs
    geometry_msgs
    std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  libsurvive_ros2 "rosidl_typesupport_cpp")

# BUILDING DRIVER

# Component
add_library(libsurvive_ros2_driver_component SHARED
  src/driver_component.cpp)
add_dependencies(libsurvive_ros2_driver_component
  libsurvive)
target_include_directories(libsurvive_ros2_driver_component PUBLIC
  ${CMAKE_INSTALL_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/include/libsurvive
  ${CMAKE_INSTALL_PREFIX}/include/libsurvive/redist)
target_link_directories(libsurvive_ros2_driver_component PUBLIC
  ${CMAKE_INSTALL_PREFIX}/lib)
target_link_libraries(libsurvive_ros2_driver_component
  "${cpp_typesupport_target}"
  -lsurvive)
target_compile_definitions(libsurvive_ros2_driver_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(libsurvive_ros2_driver_component
  rclcpp
  rclcpp_components
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros)
rclcpp_components_register_nodes(libsurvive_ros2_driver_component
  "libsurvive_ros2::DriverComponent")

# Node
add_executable(libsurvive_ros2_driver_node
  src/driver_node.cpp)
target_link_libraries(libsurvive_ros2_driver_node
  libsurvive_ros2_driver_component)
ament_target_dependencies(libsurvive_ros2_driver_node rclcpp)

# BUILDING POSER

# Component
add_library(libsurvive_ros2_poser_component SHARED
  src/poser_component.cpp)
add_dependencies(libsurvive_ros2_poser_component
  gtsam)
target_include_directories(libsurvive_ros2_poser_component PUBLIC
  ${CMAKE_INSTALL_PREFIX}/include
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS})
target_link_directories(libsurvive_ros2_poser_component PUBLIC
  ${CMAKE_INSTALL_PREFIX}/lib)
target_link_libraries(libsurvive_ros2_poser_component
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES}
  "${cpp_typesupport_target}"
  Eigen3::Eigen
  -lgtsam)
target_compile_definitions(libsurvive_ros2_poser_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_compile_options(libsurvive_ros2_poser_component
  PUBLIC -Wno-deprecated-copy -Wno-unused-parameter)
ament_target_dependencies(libsurvive_ros2_poser_component
  rclcpp
  rclcpp_components
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros)
rclcpp_components_register_nodes(libsurvive_ros2_poser_component
  "libsurvive_ros2::PoserComponent")

# Node
add_executable(libsurvive_ros2_poser_node
  src/poser_node.cpp)
target_link_libraries(libsurvive_ros2_poser_node
  libsurvive_ros2_poser_component)
ament_target_dependencies(libsurvive_ros2_poser_node rclcpp)

# INSTALLATION

# Install python packages

externalproject_get_property(gtsam binary_dir)
install(
  DIRECTORY
    ${binary_dir}/python/gtsam
  DESTINATION
    ${PYTHON_INSTALL_DIR})

# Install the components

install(
  TARGETS
    libsurvive_ros2_driver_component
    libsurvive_ros2_poser_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install the nodes

install(
  TARGETS
    libsurvive_ros2_driver_node
    libsurvive_ros2_poser_node
  DESTINATION lib/${PROJECT_NAME})

# Install helper programs

install(
  PROGRAMS
    scripts/config_merger
  DESTINATION
    lib/${PROJECT_NAME})

# Install the shared files

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME})

# Install the python package

ament_python_install_package(libsurvive_ros2_py)

# TESTING

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # Lint tests
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_ros_isolated_pytest_test(test_config_tools
      test/test_config_tools.py
    TIMEOUT 120)

endif()

# EXPORTS

# Export all include directories and declare the package
ament_export_include_directories(include)
ament_package()
