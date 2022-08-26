cmake_minimum_required(VERSION 3.14)

project(rmf_traffic_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic 2 REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rmf_site_map_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZLIB REQUIRED)

# NOTE(MXG): libproj-dev does not currently distribute its cmake config-files
# in Debian, so we can't use find_package and need to rely on pkg-config.
# find_package(PROJ REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)

if (rmf_traffic_FOUND)
  message(STATUS "found rmf_traffic")
  message(STATUS "rmf_traffic_LIBRARIES: ${rmf_traffic_LIBRARIES}")
  message(STATUS "rmf_traffic_INCLUDE_DIRS: ${rmf_traffic_INCLUDE_DIRS}")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 QUIET)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src examples
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )

  file(GLOB_RECURSE unit_test_srcs "test/unit/*.cpp")

  ament_add_catch2(
    test_rmf_traffic_ros2 test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)

  target_compile_definitions(test_rmf_traffic_ros2
    PRIVATE
      "-DTEST_RESOURCES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/resources/\"")

  target_link_libraries(test_rmf_traffic_ros2
      rmf_traffic_ros2
      yaml-cpp
  )

  add_executable(missing_query_schedule_node
    test/mock_schedule_nodes/missing_query_schedule.cpp
  )
  target_include_directories(missing_query_schedule_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(missing_query_schedule_node rmf_traffic_ros2)

  add_executable(missing_query_monitor_node
    test/mock_monitor_nodes/missing_query_monitor.cpp
  )
  target_include_directories(missing_query_monitor_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(missing_query_monitor_node rmf_traffic_ros2)

  add_executable(wrong_query_schedule_node
    test/mock_schedule_nodes/wrong_query.cpp
  )
  target_include_directories(wrong_query_schedule_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(wrong_query_schedule_node rmf_traffic_ros2)

  add_executable(delayed_query_broadcast_monitor_node
    test/mock_monitor_nodes/delayed_query_broadcast_monitor.cpp
  )
  target_include_directories(delayed_query_broadcast_monitor_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(delayed_query_broadcast_monitor_node rmf_traffic_ros2)

  add_executable(missing_participant_schedule_node
    test/mock_schedule_nodes/missing_participant.cpp
  )
  target_include_directories(missing_participant_schedule_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(missing_participant_schedule_node rmf_traffic_ros2)

  add_executable(changed_participant_schedule_node
    test/mock_schedule_nodes/changed_participant_id.cpp
  )
  target_include_directories(changed_participant_schedule_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rmf_traffic_msgs_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      "src"
  )
  target_link_libraries(changed_participant_schedule_node rmf_traffic_ros2)

  add_executable(mock_repetitive_delay_participant
    test/mock_participants/repetitive_delay_participant.cpp)
  target_link_libraries(mock_repetitive_delay_participant
    PUBLIC
      rmf_traffic_ros2)

  install(
    TARGETS
      missing_query_schedule_node
      missing_query_monitor_node
      wrong_query_schedule_node
      delayed_query_broadcast_monitor_node
      missing_participant_schedule_node
      changed_participant_schedule_node
      mock_repetitive_delay_participant
    RUNTIME DESTINATION lib/rmf_traffic_ros2
  )
endif()

file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic_ros2/*.cpp")
add_library(rmf_traffic_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_traffic_ros2
  PUBLIC
    rmf_traffic::rmf_traffic
    nlohmann_json::nlohmann_json
    ${rmf_traffic_msgs_LIBRARIES}
    ${rmf_site_map_msgs_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    yaml-cpp
    ZLIB::ZLIB
    PkgConfig::PROJ
)

target_include_directories(rmf_traffic_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_msgs_INCLUDE_DIRS}
    ${rmf_site_map_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_export_targets(rmf_traffic_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rmf_traffic
  rmf_traffic_msgs
  rmf_fleet_msgs
  rmf_site_map_msgs
  Eigen3
  rclcpp
  yaml-cpp
  nlohmann_json
  ZLIB
)

# TODO(MXG): Change these executables into shared libraries that can act as
# ROS2 node components

#===============================================================================
file(GLOB_RECURSE schedule_srcs "src/rmf_traffic_schedule/*.cpp")
add_executable(rmf_traffic_schedule ${schedule_srcs})

target_link_libraries(rmf_traffic_schedule
  PRIVATE
    rmf_traffic_ros2
)

#===============================================================================
file(GLOB_RECURSE schedule_srcs "src/rmf_traffic_schedule_monitor/*.cpp")
add_executable(rmf_traffic_schedule_monitor ${schedule_srcs})

target_link_libraries(rmf_traffic_schedule_monitor
  PRIVATE
    rmf_traffic_ros2
)

#===============================================================================
file(GLOB_RECURSE blockade_srcs "src/rmf_traffic_blockade/*.cpp")
add_executable(rmf_traffic_blockade ${blockade_srcs})

target_link_libraries(rmf_traffic_blockade
  PRIVATE
    rmf_traffic_ros2
)

#===============================================================================
# Add examples
# TODO(MXG): Consider creating a separate downstream package for these
add_executable(participant_node examples/participant_node.cpp)
target_link_libraries(participant_node PUBLIC rmf_traffic_ros2)

#===============================================================================
add_executable(update_participant src/update_participant/main.cpp)
target_link_libraries(update_participant PRIVATE rmf_traffic_ros2)

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS
    rmf_traffic_ros2
    rmf_traffic_schedule
    rmf_traffic_schedule_monitor
    rmf_traffic_blockade
    update_participant
  EXPORT rmf_traffic_ros2
  RUNTIME DESTINATION lib/rmf_traffic_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package(
  CONFIG_EXTRAS
    cmake/proj_dependency.cmake
)
