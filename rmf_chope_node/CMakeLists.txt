cmake_minimum_required(VERSION 3.8)
project(rmf_chope_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_chope_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)

add_executable(queue_manager src/main.cpp)
ament_target_dependencies(queue_manager
  rclcpp rmf_chope_msgs rmf_building_map_msgs)
target_link_libraries(queue_manager rmf_fleet_adapter::rmf_fleet_adapter)
target_include_directories(queue_manager PRIVATE ${rmf_fleet_adapter_INCLUDE_DIRS})

install(TARGETS
  queue_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
