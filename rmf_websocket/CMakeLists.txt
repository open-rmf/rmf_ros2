cmake_minimum_required(VERSION 3.5)

project(rmf_websocket)

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
find_package(rmf_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator_vendor REQUIRED)
find_package(websocketpp REQUIRED)

file(GLOB_RECURSE core_lib_srcs "src/rmf_websocket/*.cpp")
add_library(rmf_websocket SHARED ${core_lib_srcs})

target_link_libraries(rmf_websocket
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${websocketpp_LIBRARIES}
    rmf_utils::rmf_utils
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
)

target_include_directories(rmf_websocket
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
)

ament_export_targets(rmf_websocket HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic rclcpp nlohmann_json)

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_websocket
  EXPORT rmf_websocket
  RUNTIME DESTINATION lib/rmf_websocket
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
