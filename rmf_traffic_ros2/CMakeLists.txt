cmake_minimum_required(VERSION 3.5)

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
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)

# TODO(MXG): Consider splitting off the components that depend on yaml-cpp
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic_ros2/*.cpp")
add_library(rmf_traffic_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_traffic_ros2
  PUBLIC
    rmf_traffic::rmf_traffic
    ${rmf_traffic_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(rmf_traffic_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_export_interfaces(rmf_traffic_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic rmf_traffic_msgs rclcpp)

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
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_traffic_ros2 rmf_traffic_schedule
  EXPORT rmf_traffic_ros2
  RUNTIME DESTINATION lib/rmf_traffic_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
