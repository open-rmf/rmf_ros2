cmake_minimum_required(VERSION 3.5.0)

project(rmf_fleet_adapter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(rmf_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")
                
  rmf_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

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
  std_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

# -----------------------------------------------------------------------------

add_subdirectory(rmf_rxcpp)

# -----------------------------------------------------------------------------

find_package(Boost REQUIRED)
file(GLOB_RECURSE rmf_fleet_adapter_srcs "src/rmf_fleet_adapter/*.cpp")

add_library(rmf_fleet_adapter SHARED
  ${rmf_fleet_adapter_srcs}
)

target_link_libraries(rmf_fleet_adapter
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    yaml-cpp
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  PRIVATE
    rmf_rxcpp
    ${rmf_task_msgs_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    Boost::boost
)

target_include_directories(rmf_fleet_adapter
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

if (BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_catch2(
    rmf_fleet_adapter_test
      test/main.cpp
      test/phases/TransportFixture.cpp
      test/phases/DoorOpenTest.cpp
      test/phases/DoorCloseTest.cpp
      test/phases/RequestLiftTest.cpp
      test/phases/DispenseItemTest.cpp
      test/services/test_FindEmergencyPullover.cpp
      test/services/test_FindPath.cpp
      test/services/test_Negotiate.cpp
  )
  target_include_directories(rmf_fleet_adapter_test
    PRIVATE
      # private includes of rmf_fleet_adapter
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rmf_fleet_adapter>
      ${rmf_task_msgs_INCLUDE_DIRS}
      ${rmf_door_msgs_INCLUDE_DIRS}
      ${rmf_lift_msgs_INCLUDE_DIRS}

      ${std_msgs_INCLUDE_DIRS}
  )
  target_link_libraries(rmf_fleet_adapter_test
    PRIVATE
      # private libraries of rmf_fleet_adapter
      rmf_rxcpp
      ${rmf_task_msgs_LIBRARIES}
      ${rmf_door_msgs_LIBRARIES}
      ${rmf_lift_msgs_LIBRARIES}
      Boost::boost

      rmf_fleet_adapter
      rmf_utils::rmf_utils
      ${std_msgs_LIBRARIES}
  )
endif ()

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

add_executable(robot_state_aggregator
  src/robot_state_aggregator/main.cpp
)

target_link_libraries(robot_state_aggregator
  PRIVATE
    rmf_fleet_adapter
    ${rclcpp_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
)

target_include_directories(robot_state_aggregator
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(test_read_only_adapter
  test/test_read_only_adapter.cpp
)

target_link_libraries(test_read_only_adapter
  PRIVATE
    rmf_fleet_adapter
)

# -----------------------------------------------------------------------------

add_executable(dump_fleet_states
  test/dump_fleet_states
)

target_link_libraries(dump_fleet_states
  PRIVATE
    rmf_fleet_adapter
    ${rclcpp_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
)

target_include_directories(dump_fleet_states
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(task_aggregator
  src/task_aggregator/main.cpp
)

target_link_libraries(task_aggregator
  PRIVATE
    rmf_fleet_adapter
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
)

target_include_directories(task_aggregator
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

install(
  TARGETS 
    rmf_fleet_adapter
    read_only
    full_control
    lift_supervisor
    door_supervisor
    robot_state_aggregator
    test_read_only_adapter
    task_aggregator
  RUNTIME DESTINATION lib/rmf_fleet_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# -----------------------------------------------------------------------------

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
