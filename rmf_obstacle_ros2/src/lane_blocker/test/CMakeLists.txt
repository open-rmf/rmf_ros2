file(GLOB_RECURSE unit_test_srcs "*.cpp" "../IntersectionChecker.cpp")

ament_add_catch2(
  test_intersection_checker ${unit_test_srcs}
  TIMEOUT 300
)

target_link_libraries(test_intersection_checker
  PRIVATE
    ${rmf_obstacle_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    rmf_utils::rmf_utils
)

target_include_directories(test_intersection_checker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_utils_INCLUDE_DIRS}
)

install(
  TARGETS test_intersection_checker
  EXPORT test_intersection_checker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
