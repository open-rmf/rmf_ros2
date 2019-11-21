cmake_minimum_required(VERSION 3.5.0)

project(rmf_fleet_adapter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(dep_pkgs
  rclcpp
  rmf_utils
  rmf_fleet_msgs
  rmf_traffic
  rmf_traffic_ros2
  rmf_traffic_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

# -----------------------------------------------------------------------------

add_library(rmf_fleet_adapter
  src/rmf_fleet_adapter/ParseGraph.cpp
)

target_link_libraries(rmf_fleet_adapter
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    yaml-cpp
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(rmf_fleet_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(read_only
  src/read_only/main.cpp
  src/read_only/FleetAdapterNode.cpp
)

target_link_libraries(read_only
  PRIVATE
    rmf_fleet_adapter
)

# -----------------------------------------------------------------------------

add_executable(fake_fleet
  test/FakeFleet.cpp
)
ament_target_dependencies(fake_fleet rclcpp rmf_fleet_msgs)

# -----------------------------------------------------------------------------

install(
  TARGETS 
    rmf_fleet_adapter
    read_only
    fake_fleet
  RUNTIME DESTINATION lib/rmf_fleet_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
