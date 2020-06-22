cmake_minimum_required(VERSION 3.5)
project(rmf_fleet_adapter_python)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_task_msgs REQUIRED)

include_directories(
  include
  ${rmf_fleet_adapter_INCLUDE_DIRS}
)

# pybind11 is installed by the pybind_ament package
# You must have the pybind_ament package in your workspace somewhere!
find_package(pybind11 REQUIRED)

pybind11_add_module(rmf_adapter
  src/adapter.cpp
  src/tests.cpp
  src/types/types.cpp
  src/graph/graph.cpp
  src/graph/lane.cpp
  src/geometry/shapes.cpp
  src/planner/planner.cpp
  src/vehicletraits/vehicletraits.cpp
)
target_link_libraries(rmf_adapter PRIVATE
  rmf_fleet_adapter::rmf_fleet_adapter)

target_include_directories(rmf_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
  PRIVATE
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
)

# NO AMENT PACKAGE! That's settled by setup.py
