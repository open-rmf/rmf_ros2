cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler_ros2_examples)

find_package(rclcpp REQUIRED)
find_package(rmf_scheduler_ros2_client REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

add_executable(create_trigger create_trigger.cpp)
target_link_libraries(create_trigger PRIVATE
  rclcpp::rclcpp
  rmf_scheduler_ros2_client::rmf_scheduler_ros2_client
)

add_executable(create_schedule create_schedule.cpp)
target_link_libraries(create_schedule PRIVATE
  rclcpp::rclcpp
  rmf_scheduler_ros2_client::rmf_scheduler_ros2_client
)
