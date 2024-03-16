cmake_minimum_required(VERSION 3.8)
project(rmf_obstacle_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

set(dep_pkgs
  geometry_msgs
  rclcpp
  rclcpp_components
  rmf_utils
  rmf_obstacle_msgs
  rmf_fleet_msgs
  rmf_building_map_msgs
  sensor_msgs
  OCTOMAP
  rmf_fleet_adapter
  rmf_traffic
  rmf_traffic_ros2
  tf2_ros
  tf2_geometry_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
file(GLOB_RECURSE core_lib_srcs "src/rmf_obstacle_ros2/*.cpp")
add_library(rmf_obstacle_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_obstacle_ros2
  PUBLIC
    rmf_utils::rmf_utils
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
    ${rmf_obstacle_msgs_TARGETS}
  PRIVATE
    ${OCTOMAP_TARGETS}
)

target_include_directories(rmf_obstacle_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
  PRIVATE
    ${OCTOMAP_INCLUDE_DIRS}
)

target_compile_features(rmf_obstacle_ros2 INTERFACE cxx_std_17)

ament_export_targets(rmf_obstacle_ros2 HAS_LIBRARY_TARGET)

ament_export_dependencies(${dep_pkgs})

ament_export_include_directories(
  include
)

#===============================================================================
file(GLOB_RECURSE lane_blocker_srcs
  "src/lane_blocker/*.cpp"
)
add_library(lane_blocker SHARED ${lane_blocker_srcs})

target_link_libraries(lane_blocker
  PRIVATE
    rclcpp::rclcpp
    ${rclcpp_components_TARGETS}
    ${rmf_obstacle_msgs_TARGETS}
    ${rmf_fleet_msgs_TARGETS}
    ${rmf_building_map_msgs_TARGETS}
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_traffic::rmf_traffic
    ${rmf_traffic_ros2_TARGETS}
    ${tf2_ros_TARGETS}
    ${geometry_msgs_TARGETS}
    tf2_geometry_msgs::tf2_geometry_msgs
)

target_include_directories(lane_blocker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${rmf_fleet_adapter_INCLUDE_DIRS}
    ${rmf_traffic_INCLUDE_DIRS}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_geometry_msgs_include_dirs}
)

target_compile_features(lane_blocker INTERFACE cxx_std_17)

rclcpp_components_register_node(lane_blocker
  PLUGIN "LaneBlocker"
  EXECUTABLE lane_blocker_node)

#===============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_catch2 REQUIRED)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )

  add_subdirectory(src/lane_blocker/test)
endif()

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS
    rmf_obstacle_ros2
    lane_blocker
  EXPORT rmf_obstacle_ros2
  RUNTIME DESTINATION lib/rmf_obstacle_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
