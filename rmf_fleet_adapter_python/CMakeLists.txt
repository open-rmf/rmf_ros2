cmake_minimum_required(VERSION 3.5)
project(rmf_fleet_adapter_python)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(ament_cmake REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_task REQUIRED)
find_package(rmf_battery REQUIRED)
find_package(rmf_task_msgs REQUIRED)

if (NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "Could not find Python interpreter executable. Is Python3 installed and found correctly?")
else()
    message(STATUS "Python Interpreter Executable: ${Python3_EXECUTABLE}")
endif()

# Get the NumPy version string via Python interpreter
execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.__version__)"
                OUTPUT_VARIABLE NUMPY_VERSION_STRING
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_VARIABLE PYTHON_STDERR_OUTPUT # Capture stderr for debugging
                RESULT_VARIABLE PYTHON_CMD_RESULT)
message(STATUS "NumPy Version String from Python: ${NUMPY_VERSION_STRING}")
message(STATUS "Python Command Result: ${PYTHON_CMD_RESULT}")
message(STATUS "Python Command Error: ${PYTHON_CMD_ERROR}")
message(STATUS "Python Command Result: ${PYTHON_CMD_RESULT}")
message(STATUS "Python STDERR Output: ${PYTHON_STDERR_OUTPUT}")

# Define the macro for C++ compilation
add_compile_definitions(BUILT_AGAINST_NUMPY_VERSION="${NUMPY_VERSION_STRING}")

include_directories(
  include
  ${rmf_fleet_adapter_INCLUDE_DIRS}
)

find_package(pybind11 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pybind11_json REQUIRED)

pybind11_add_module(rmf_adapter SHARED
  src/adapter.cpp
  src/tests.cpp
  src/types/types.cpp
  src/graph/graph.cpp
  src/graph/lane.cpp
  src/geometry/shapes.cpp
  src/planner/planner.cpp
  src/nodes/nodes.cpp
  src/vehicletraits/vehicletraits.cpp
  src/battery/battery.cpp
  src/schedule/schedule.cpp
)

target_link_libraries(rmf_adapter PRIVATE
  rmf_fleet_adapter::rmf_fleet_adapter
  nlohmann_json::nlohmann_json
  pybind11_json)

target_include_directories(rmf_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_INCLUDE_DIRS}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_battery_INCLUDE_DIRS}
    ${rmf_task_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
  PRIVATE
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
)

# Locate ament_package template files.
set(PYTHON_INSTALL_DIR "lib/python${PYTHON_MAJOR_MINOR}/site-packages")

# Install library for actual use
install(TARGETS rmf_adapter
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/test_adapter.py
  scripts/test_utils.py
  scripts/test_loop.py
  scripts/test_delivery.py
  scripts/rmf_msg_observer.py
  scripts/traffic_light.py
  scripts/schedule_blockade_nodes.py
  DESTINATION lib/${PROJECT_NAME}
)

# Unit Tests

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_rmf_fleet_adapter_python_tests
    tests/unit/test_geometry.py
    tests/unit/test_graph.py
    tests/unit/test_RobotCommandHandle.py
    tests/unit/test_types.py
    tests/unit/test_vehicletraits.py
  )

  foreach(_test_path ${_rmf_fleet_adapter_python_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      TIMEOUT 120
      WERROR ON
    )
  endforeach()
endif()

ament_package()
