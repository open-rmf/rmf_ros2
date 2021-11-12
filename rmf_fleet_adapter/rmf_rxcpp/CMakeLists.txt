cmake_minimum_required(VERSION 3.5.0)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

add_library(rmf_rxcpp INTERFACE)

target_include_directories(rmf_rxcpp
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/RxCpp-4.1.0/Rx/v2/src
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(rmf_rxcpp
  INTERFACE
    Threads::Threads
    ${rclcpp_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(rmf_utils REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file NAMES
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_add_catch2(
    test_rmf_rxcpp
      test/main.cpp
      test/test_RxJobs.cpp
      test/test_Transport.cpp
  )
  target_include_directories(test_rmf_rxcpp
    PRIVATE
      ${std_msgs_INCLUDE_DIRS}
  )
  target_link_libraries(test_rmf_rxcpp
    PRIVATE
      rmf_rxcpp
      rmf_utils::rmf_utils
      ${std_msgs_LIBRARIES}
  )

  find_package(rmf_traffic REQUIRED)

  add_executable(rmf_rxcpp_example_plan_path examples/PlanPath.cpp)

  target_link_libraries(rmf_rxcpp_example_plan_path
    PUBLIC
      rmf_traffic::rmf_traffic
      rmf_rxcpp
  )

endif()
