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
  GIT_TAG master
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(libsurvive_ros2_component SHARED
  src/libsurvive_ros2_component.cpp)
target_compile_definitions(libsurvive_ros2_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(libsurvive_ros2_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")


# Build ROS2 interface to libsurvive
add_executable(libsurvive_node src/libsurvive_node.cpp)
add_dependencies(libsurvive_node libsurvive)
target_include_directories(libsurvive_node PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include/libsurvive
  ${CMAKE_CURRENT_BINARY_DIR}/include/libsurvive/redist
)
target_link_directories(libsurvive_node PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/lib
)
target_link_libraries(libsurvive_node -lsurvive)
ament_target_dependencies(libsurvive_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install the ROS2 interface and libsurvive
install(TARGETS
          libsurvive_node
        DESTINATION
          lib/${PROJECT_NAME}
)

# Install the example config
install(DIRECTORY
          config
        DESTINATION
          share/${PROJECT_NAME}
)

# this must happen before the invocation of ament_package()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()


