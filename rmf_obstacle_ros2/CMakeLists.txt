cmake_minimum_required(VERSION 3.8)
project(rmf_obstacle_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rmf_utils
  pluginlib
  rmf_obstacle_msgs
  vision_msgs
  rmf_fleet_msgs
  rmf_building_map_msgs
  sensor_msgs
  OCTOMAP
  rmf_fleet_adapter
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  rmf_traffic
  rmf_traffic_ros2
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

if ($ENV{ROS_DISTRO} MATCHES humble)
  set(tf2_geometry_msgs_include_dirs "")
  set(tf2_geometry_msgs_link_libraries tf2_geometry_msgs::tf2_geometry_msgs)
else()
  set(tf2_geometry_msgs_include_dir ${tf2_geometry_msgs_INCLUDE_DIRS})
  set(tf2_geometry_msgs_link_libraries ${tf2_geometry_msgs_LIBRARIES})
endif()

#===============================================================================
file(GLOB_RECURSE core_lib_srcs "src/rmf_obstacle_ros2/*.cpp")
add_library(rmf_obstacle_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_obstacle_ros2
  PUBLIC
    rmf_utils::rmf_utils
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
  PRIVATE
    ${OCTOMAP_LIBRARIES}
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
    ${rclcpp_components_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
    ${vision_msgs_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    rmf_fleet_adapter::rmf_fleet_adapter
    rmf_traffic::rmf_traffic
    ${rmf_traffic_ros2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_geometry_msgs_link_libraries}
)

target_include_directories(lane_blocker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${vision_msgs_INCLUDE_DIRS}
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
