cmake_minimum_required(VERSION 3.5.0)

project(rmf_fleet_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
if(DEFINED ENV{RMF_ENABLE_FAILOVER})
  if($ENV{RMF_ENABLE_FAILOVER} EQUAL 1)
    find_package(stubborn_buddies REQUIRED)
    find_package(stubborn_buddies_msgs REQUIRED)
  endif()
endif()
include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rmf_utils
  rmf_dispenser_msgs
  rmf_ingestor_msgs
  rmf_door_msgs
  rmf_fleet_msgs
  rmf_lift_msgs
  rmf_task_msgs
  rmf_traffic_ros2
  rmf_task_ros2
  rmf_battery
  rmf_task
  rmf_task_sequence
  std_msgs
  rmf_api_msgs
  rmf_websocket
  rmf_building_map_msgs
  nlohmann_json
  nlohmann_json_schema_validator_vendor
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    include src test rmf_rxcpp/include rmf_rxcpp/src rmf_rxcpp/test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()
# -----------------------------------------------------------------------------

add_subdirectory(rmf_rxcpp)

# -----------------------------------------------------------------------------

file(GLOB_RECURSE rmf_fleet_adapter_srcs "src/rmf_fleet_adapter/*.cpp")

add_library(rmf_fleet_adapter SHARED
  ${rmf_fleet_adapter_srcs}
)

target_link_libraries(rmf_fleet_adapter
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_battery::rmf_battery
    rmf_task::rmf_task
    rmf_task_sequence::rmf_task_sequence
    rmf_task_ros2::rmf_task_ros2
    yaml-cpp
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
  PRIVATE
    rmf_rxcpp
    rmf_websocket::rmf_websocket
    nlohmann_json::nlohmann_json
    rmf_api_msgs::rmf_api_msgs
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${rmf_dispenser_msgs_LIBRARIES}
    ${rmf_ingestor_msgs_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    nlohmann_json_schema_validator
)

target_include_directories(rmf_fleet_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rmf_api_generate_schema_headers/include> # for auto-generated schema headers
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_battery_INCLUDE_DIRS}
  PRIVATE
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${rmf_dispenser_msgs_INCLUDE_DIRS}
    ${rmf_ingestor_msgs_INCLUDE_DIRS}
    ${rmf_api_msgs_INCLUDE_DIRS}
    ${rmf_websocket_INCLUDE_DIR}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${nlohmann_json_schema_validator_INCLUDE_DIRS}
)

if (BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_catch2(
    test_rmf_fleet_adapter
      test/main.cpp
      test/phases/MockAdapterFixture.cpp
      test/phases/test_DoorOpen.cpp
      test/phases/test_DoorClose.cpp
      test/phases/test_RequestLift.cpp
      test/phases/test_DispenseItem.cpp
      test/phases/test_IngestItem.cpp
      test/services/test_FindEmergencyPullover.cpp
      test/services/test_FindPath.cpp
      test/services/test_Negotiate.cpp
      test/tasks/test_Delivery.cpp
      test/tasks/test_Loop.cpp
      test/test_Task.cpp
    TIMEOUT 300
  )
  target_include_directories(test_rmf_fleet_adapter
    PRIVATE
      # private includes of rmf_fleet_adapter
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rmf_fleet_adapter>
      ${rmf_task_msgs_INCLUDE_DIRS}
      ${rmf_door_msgs_INCLUDE_DIRS}
      ${rmf_lift_msgs_INCLUDE_DIRS}
      ${rmf_dispenser_msgs_INCLUDE_DIRS}
      ${rmf_ingestor_msgs_INCLUDE_DIRS}
      ${rmf_api_msgs_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
      ${rmf_building_map_msgs_INCLUDE_DIRS}
      ${rmf_websocket_INCLUDE_DIR}
      ${nlohmann_json_schema_validator_INCLUDE_DIRS}
  )
  target_link_libraries(test_rmf_fleet_adapter
    PRIVATE
      # private libraries of rmf_fleet_adapter
      rmf_rxcpp
      ${rmf_task_msgs_LIBRARIES}
      ${rmf_door_msgs_LIBRARIES}
      ${rmf_lift_msgs_LIBRARIES}
      ${rmf_dispenser_msgs_LIBRARIES}
      ${rmf_ingestor_msgs_LIBRARIES}
      rmf_fleet_adapter
      rmf_utils::rmf_utils
      rmf_api_msgs::rmf_api_msgs
      ${std_msgs_LIBRARIES}
      ${rmf_building_map_msgs_LIBRARIES}
      ${rmf_websocket_INCLUDE_DIR}
      nlohmann_json_schema_validator
  )

  target_compile_definitions(test_rmf_fleet_adapter
    PRIVATE
      "-DTEST_RESOURCES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/resources/\"")

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

add_executable(read_only_blockade
  src/read_only_blockade/main.cpp
  src/read_only_blockade/FleetAdapterNode.cpp
)

target_link_libraries(read_only_blockade
  PRIVATE
    rmf_fleet_adapter
)

# -----------------------------------------------------------------------------

add_executable(full_control src/full_control/main.cpp)

target_link_libraries(full_control
  PRIVATE
    rmf_fleet_adapter
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
)

target_include_directories(full_control
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(mock_traffic_light src/mock_traffic_light/main.cpp)

target_link_libraries(mock_traffic_light
  PRIVATE
    rmf_fleet_adapter
    ${rmf_task_msgs_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
)

target_include_directories(mock_traffic_light
  PRIVATE
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
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

add_executable(experimental_lift_watchdog
  src/experimental_lift_watchdog/main.cpp
)

target_link_libraries(experimental_lift_watchdog
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(experimental_lift_watchdog
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
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

add_library(robot_state_aggregator_main SHARED
  src/robot_state_aggregator/RobotStateAggregator.cpp
)

target_include_directories(robot_state_aggregator_main
  PRIVATE
    include
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${stubborn_buddies_msgs_INCLUDE_DIRS})

target_compile_definitions(robot_state_aggregator_main
  PRIVATE "COMPOSITION_BUILDING_DLL"
)


if(DEFINED ENV{RMF_ENABLE_FAILOVER})
  if($ENV{RMF_ENABLE_FAILOVER} EQUAL 1)
    ament_target_dependencies(robot_state_aggregator_main
      "rclcpp"
      "rclcpp_components"
      "std_msgs"
      "stubborn_buddies_msgs"
      "rmf_fleet_msgs"
    )
  else()
    ament_target_dependencies(robot_state_aggregator_main
      "rclcpp"
      "rclcpp_components"
      "std_msgs"
      "rmf_fleet_msgs"
    )
  endif()
else()
  ament_target_dependencies(robot_state_aggregator_main
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "rmf_fleet_msgs"
  )
endif()

rclcpp_components_register_nodes(robot_state_aggregator_main
  "RobotStateAggregator"
)

set(node_plugins "${node_plugins}RobotStateAggregator;
    $<TARGET_FILE:robot_state_aggregator_main>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(robot_state_aggregator
  src/robot_state_aggregator/main.cpp
)

if(DEFINED ENV{RMF_ENABLE_FAILOVER})
  if($ENV{RMF_ENABLE_FAILOVER} EQUAL 1)
    add_compile_definitions(FAILOVER_MODE)
    set(robot_state_aggregator_main_libs
      robot_state_aggregator_main
      stubborn_buddies::lifecycle_watchdog
      stubborn_buddies::lifecycle_heartbeat
    )
  else()
    set(robot_state_aggregator_main_libs
      robot_state_aggregator_main
    )
  endif()
else()
  set(robot_state_aggregator_main_libs
    robot_state_aggregator_main
  )
endif()


if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(robot_state_aggregator_main_libs
    "-Wl,--no-as-needed"
    ${robot_state_aggregator_main_libs}
    "-Wl,--as-needed")
endif()

target_link_libraries(robot_state_aggregator
  ${robot_state_aggregator_main_libs})

ament_target_dependencies(robot_state_aggregator
  "class_loader"
  "rclcpp"
  "rclcpp_components"
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

add_executable(open_lanes src/open_lanes/main.cpp)

target_link_libraries(open_lanes
  PRIVATE
    ${rmf_fleet_msgs_LIBRARIES}
    rclcpp::rclcpp
    rmf_fleet_adapter
)

target_include_directories(open_lanes
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(close_lanes src/close_lanes/main.cpp)

target_link_libraries(close_lanes
  PRIVATE
    ${rmf_fleet_msgs_LIBRARIES}
    rclcpp::rclcpp
    rmf_fleet_adapter
)

target_include_directories(close_lanes
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_executable(interrupt_robot src/interrupt_robot/main.cpp)

target_link_libraries(interrupt_robot
  PRIVATE
    ${rmf_fleet_msgs_LIBRARIES}
    rclcpp::rclcpp
    rmf_fleet_adapter
)

target_include_directories(interrupt_robot
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

rmf_api_generate_schema_headers(
  PACKAGE rmf_fleet_adapter
  SCHEMAS_DIR ${CMAKE_CURRENT_LIST_DIR}/schemas
)

# -----------------------------------------------------------------------------

ament_export_targets(rmf_fleet_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(${dep_pkgs})

install(
  TARGETS
    rmf_fleet_adapter
    read_only
    read_only_blockade
    mock_traffic_light
    full_control
    lift_supervisor
    experimental_lift_watchdog
    door_supervisor
    robot_state_aggregator
    test_read_only_adapter
    task_aggregator
    open_lanes
    close_lanes
    interrupt_robot
    robot_state_aggregator_main
  EXPORT rmf_fleet_adapter
  RUNTIME DESTINATION lib/rmf_fleet_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# -----------------------------------------------------------------------------

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
