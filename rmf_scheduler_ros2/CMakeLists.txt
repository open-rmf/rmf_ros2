cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_scheduler_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(sqlite3 sqlite3 REQUIRED IMPORTED_TARGET)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

add_library(lib${PROJECT_NAME} OBJECT
  src/RosPublisher.cpp
  src/SchedulerNode.cpp
  src/SqliteError.cpp
  src/SqliteDataSource.cpp
  src/SystemTimeExecutor.cpp
)
target_link_libraries(lib${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  PkgConfig::sqlite3
  ${rmf_scheduler_msgs_LIBRARIES}
)
target_include_directories(lib${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cronpp-999f7685ab683b58872386c0aa019acf97c6570a
  ${rmf_scheduler_msgs_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  lib${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME} PRIVATE)

#===============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)
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
      lib${PROJECT_NAME}
      rmf_utils::rmf_utils
    )
  endfunction()

  add_custom_test(Scheduler_TEST SOURCES
    src/Scheduler_TEST.cpp
    src/test/VirtualExecutor.cpp
  )

  add_custom_test(SystemTimeExecutor_TEST SOURCES
    src/SystemTimeExecutor_TEST.cpp
  )

  add_custom_test(SqliteDataSource_TEST SOURCES
    src/SqliteDataSource_TEST.cpp
  )
endif()

#===============================================================================

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
