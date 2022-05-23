cmake_minimum_required(VERSION 3.8)
project(rmf_obstacle_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)

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

ament_export_dependencies(
  rclcpp
  sensor_msgs
  rmf_obstacle_msgs
)

#===============================================================================
add_library(obstacle_manager SHARED src/obstacle_manager/ObstacleManager.cpp)

target_link_libraries(obstacle_manager
  PRIVATE
    rmf_obstacle_ros2
    rmf_utils::rmf_utils
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
    ${pluginlib_LIBRARIES}
)

target_include_directories(obstacle_manager
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
)

target_compile_features(obstacle_manager INTERFACE cxx_std_17)

rclcpp_components_register_node(obstacle_manager
  PLUGIN "rmf_obstacle_ros2::ObstacleManager"
  EXECUTABLE obstacle_manager_node)

#===============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )

endif()

#===============================================================================
install(
  DIRECTORY include/rmf_obstacle_ros2
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS
    rmf_obstacle_ros2
    obstacle_manager
  EXPORT rmf_obstacle_ros2
  RUNTIME DESTINATION lib/rmf_obstacle_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
