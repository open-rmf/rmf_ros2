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
  rmf_dispenser_msgs
  rmf_door_msgs
  rmf_fleet_msgs
  rmf_lift_msgs
  rmf_task_msgs
  rmf_traffic
  rmf_traffic_ros2
  rmf_traffic_msgs
  std_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

# -----------------------------------------------------------------------------

add_library(rmf_fleet_adapter
  src/rmf_fleet_adapter/ParseGraph.cpp
  src/rmf_fleet_adapter/ParseArgs.cpp
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

add_executable(full_control
  src/full_control/main.cpp
  src/full_control/FleetAdapterNode.cpp
  src/full_control/MoveAction.cpp
  src/full_control/DispenseAction.cpp
  src/full_control/Tasks.cpp
  src/full_control/ScheduleManager.cpp
)

target_link_libraries(full_control
  PRIVATE
    rmf_fleet_adapter
    ${rmf_task_msgs_LIBRARIES}
    ${rmf_dispenser_msgs_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(full_control
  PRIVATE
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_dispenser_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(lift_supervisor
  src/lift_supervisor/main.cpp
  src/lift_supervisor/Node.cpp
)

target_link_libraries(lift_supervisor
  PRIVATE
    rmf_fleet_adapter
    ${rclcpp_LIBARRIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(lift_supervisor
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)


# -----------------------------------------------------------------------------

add_executable(door_supervisor
  src/door_supervisor/main.cpp
  src/door_supervisor/Node.cpp
)

target_link_libraries(door_supervisor
  PRIVATE
    rmf_fleet_adapter
    ${rclcpp_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
)

target_include_directories(door_supervisor
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(fake_fleet
  test/FakeFleet.cpp
)
ament_target_dependencies(fake_fleet rclcpp rmf_fleet_msgs)

# -----------------------------------------------------------------------------

add_executable(test_read_only_adapter
  test/test_read_only_adapter.cpp
)

target_link_libraries(test_read_only_adapter
  PRIVATE
    rmf_fleet_adapter
)

# -----------------------------------------------------------------------------

install(
  TARGETS 
    rmf_fleet_adapter
    read_only
    fake_fleet
    test_read_only_adapter
  RUNTIME DESTINATION lib/rmf_fleet_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
