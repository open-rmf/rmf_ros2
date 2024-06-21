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
find_package(nlohmann_json_schema_validator REQUIRED)
find_package(websocketpp REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Threads)


add_library(rmf_websocket SHARED
  src/rmf_websocket/client/ClientWebSocketEndpoint.cpp
  src/rmf_websocket/BroadcastClient.cpp
  src/rmf_websocket/BroadcastServer.cpp
)

target_link_libraries(rmf_websocket
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${websocketpp_LIBRARIES}
    rmf_utils::rmf_utils
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
  PRIVATE
    Boost::system
    Threads::Threads
)

target_include_directories(rmf_websocket
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
)

add_executable(example_client
  examples/client.cpp)

target_link_libraries(example_client
  PUBLIC
  rmf_websocket
    ${websocketpp_LIBRARIES}
  PRIVATE
    Threads::Threads
)

target_include_directories(example_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${WEBSOCKETPP_INCLUDE_DIR}
)


ament_export_targets(rmf_websocket HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic rclcpp nlohmann_json websocketpp)

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

# Disable uncrustify tests by default.
set(TEST_UNCRUSTIFY "Off")
if(BUILD_TESTING)
  if(TEST_UNCRUSTIFY)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_file(uncrustify_config_file
      NAMES "rmf_code_style.cfg"
      PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

    ament_uncrustify(
      include src
      CONFIG_FILE ${uncrustify_config_file}
      LANGUAGE C++
      MAX_LINE_LENGTH 80
    )
  endif()

  # unit test
  find_package(ament_cmake_catch2 REQUIRED)
  ament_add_catch2(test_ring_buffer
    src/rmf_websocket/utils/RingBuffer_TEST.cpp
    TIMEOUT 300)
  target_link_libraries(test_ring_buffer
    PRIVATE
      rmf_utils::rmf_utils
      Boost::system
    )

#integration test
  find_package(OpenSSL REQUIRED)
  ament_add_catch2(test_client
    test/test_client.cpp
    TIMEOUT 300)
  target_link_libraries(test_client
    PUBLIC
    ${websocketpp_LIBRARIES}
      ${rclcpp_LIBRARIES}
      rmf_utils::rmf_utils
      Boost::system
      rmf_websocket
      PRIVATE
      Threads::Threads
      OpenSSL::Crypto
    )
  target_include_directories(test_client
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rclcpp_INCLUDE_DIRS}
      ${WEBSOCKETPP_INCLUDE_DIR}
  )


  ament_add_catch2(test_client_no_server
    test/test_client_no_server.cpp
    TIMEOUT 300)
  target_link_libraries(test_client_no_server
    PUBLIC
      ${websocketpp_LIBRARIES}
      ${rclcpp_LIBRARIES}
      rmf_utils::rmf_utils
      Boost::system
      rmf_websocket
      PRIVATE
      Threads::Threads
      OpenSSL::Crypto
  )
  target_include_directories(test_client_no_server
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rclcpp_INCLUDE_DIRS}
      ${WEBSOCKETPP_INCLUDE_DIR}
  )

  ament_add_catch2(test_client_with_update_cb
    test/test_client_with_update_cb.cpp
    TIMEOUT 300)
  target_link_libraries(test_client_with_update_cb
    PUBLIC
      ${websocketpp_LIBRARIES}
      ${rclcpp_LIBRARIES}
      rmf_utils::rmf_utils
      Boost::system
      rmf_websocket
      PRIVATE
      Threads::Threads
      OpenSSL::Crypto
  )
  target_include_directories(test_client_with_update_cb
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${rclcpp_INCLUDE_DIRS}
      ${WEBSOCKETPP_INCLUDE_DIR}
  )
endif()

ament_package()
