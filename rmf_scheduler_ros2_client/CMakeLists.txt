cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler_ros2_client)

find_package(rclcpp REQUIRED)
find_package(rmf_scheduler_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  ${rmf_scheduler_msgs_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rmf_scheduler_msgs_INCLUDE_DIRS}
)

#===============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)
  find_package(Threads REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/"
  )

  ament_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )

  function(add_custom_test target)
    cmake_parse_arguments(PARSE_ARGV 1 "" "" "" "SOURCES")

    ament_add_catch2(${target}
      ${_SOURCES}
    )
    target_link_libraries(${target} PRIVATE
      rmf_utils::rmf_utils
    )
  endfunction()

  add_custom_test(e2e_TEST SOURCES
    test/e2e_TEST.cpp
  )
  target_link_libraries(e2e_TEST PRIVATE
    ${PROJECT_NAME}
    Boost::filesystem
    Threads::Threads
  )
endif()

#===============================================================================

install(TARGETS ${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
