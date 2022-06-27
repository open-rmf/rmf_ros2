cmake_minimum_required(VERSION 3.5)

project(rmf_task_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rmf_api_msgs REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_websocket REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator_vendor REQUIRED)

file(GLOB_RECURSE core_lib_srcs "src/rmf_task_ros2/*.cpp")
add_library(rmf_task_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_task_ros2
  PUBLIC
    rmf_api_msgs::rmf_api_msgs
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_websocket::rmf_websocket
    ${rmf_task_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
)

target_include_directories(rmf_task_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_websocket_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_export_targets(rmf_task_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic rmf_task_msgs rclcpp nlohmann_json)

#===============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 QUIET)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )

  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    test_rmf_task_ros2 test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)

  target_link_libraries(test_rmf_task_ros2
      rmf_task_ros2
      rmf_traffic::rmf_traffic
      rmf_websocket::rmf_websocket
      rmf_traffic_ros2::rmf_traffic_ros2
      -pthread
  )

  target_include_directories(test_rmf_task_ros2
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
endif()

#===============================================================================

add_executable(rmf_bidder_node
  src/mock_bidder/main.cpp
)
target_link_libraries(rmf_bidder_node PUBLIC rmf_task_ros2 -pthread)

#===============================================================================

add_executable(rmf_task_dispatcher
  src/dispatcher_node/main.cpp
)
target_link_libraries(rmf_task_dispatcher PUBLIC rmf_task_ros2)

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_task_ros2 rmf_task_dispatcher rmf_bidder_node
  EXPORT rmf_task_ros2
  RUNTIME DESTINATION lib/rmf_task_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
