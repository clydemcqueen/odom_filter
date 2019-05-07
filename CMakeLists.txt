cmake_minimum_required(VERSION 3.5)
project(odom_filter)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  message("Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/crystal/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/crystal/lib/libfastrtps.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# Local includes
include_directories(
  include
)

#=============
# Filter node
#=============

add_executable(
  filter_node
  src/filter_node.cpp
  src/filter_context.cpp
  src/kf.cpp
)

ament_target_dependencies(
  filter_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_msgs
)

#=============
# Install
#=============

# Install targets
install(
  TARGETS filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()