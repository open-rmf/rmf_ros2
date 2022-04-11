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
  src/Scheduler.cpp
  src/SqliteDataSource.cpp
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

# add_executable(${PROJECT_NAME}
#   src/main.cpp)

# target_link_libraries(${PROJECT_NAME} PRIVATE
#   rclcpp::rclcpp)

# target_include_directories(${PROJECT_NAME} PRIVATE)

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

  ament_add_catch2(Scheduler_TEST TIMEOUT 300
    src/Scheduler_TEST.cpp
    src/test/VirtualExecutor.cpp
  )

  target_link_libraries(Scheduler_TEST PRIVATE
    lib${PROJECT_NAME}
    rmf_utils::rmf_utils
  )

  target_include_directories(Scheduler_TEST PRIVATE ${rmf_utils_INCLUDE_DIRS})
endif()

#===============================================================================

# install(TARGETS ${PROJECT_NAME})

ament_package()
