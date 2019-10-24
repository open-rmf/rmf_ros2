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

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)


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

ament_package()

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
file(GLOB_RECURSE proto_fleet_adapter_srcs "src/proto_fleet_adapter/*.cpp")
add_executable(proto_fleet_adapter ${proto_fleet_adapter_srcs})

target_link_libraries(proto_fleet_adapter
  PRIVATE
    rmf_traffic_ros2
)
