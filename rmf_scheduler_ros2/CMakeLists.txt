cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  rclcpp::rclcpp)

target_include_directories(${PROJECT_NAME} PRIVATE)

#===============================================================================

if(BUILD_TESTING)
  # find_package(ament_cmake_catch2 QUIET)
  # find_package(ament_cmake_uncrustify REQUIRED)
  # find_file(uncrustify_config_file
  #   NAMES "rmf_code_style.cfg"
  #   PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  # ament_uncrustify(
  #   ARGN include src test
  #   CONFIG_FILE ${uncrustify_config_file}
  #   MAX_LINE_LENGTH 80)

  # ament_add_catch2(test_${PROJECT_NAME} TIMEOUT 300)

  # target_link_libraries(test_${PROJECT_NAME} PRIVATE)

  # target_include_directories(test_${PROJECT_NAME} PRIVATE)
endif()

#===============================================================================

install(TARGETS ${PROJECT_NAME})

ament_package()
