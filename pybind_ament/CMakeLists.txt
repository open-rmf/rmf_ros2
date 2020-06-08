cmake_minimum_required(VERSION 3.5)
project(pybind_ament)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include(ExternalProject)

ExternalProject_Add(pybind11_src
  URL "https://github.com/pybind/pybind11/archive/v2.5.0.zip"
  UPDATE_COMMAND ""
  CMAKE_ARGS -DPYBIND11_PYTHON_VERSION=${PYTHON_VERSION_STRING}
             -DPYBIND11_TEST:BOOL=OFF -DPYBIND11_INSTALL:BOOL=ON
             -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_BINARY_DIR}
)

# Install /include Headers
install(
  DIRECTORY ${PROJECT_BINARY_DIR}/include/pybind11/
  DESTINATION include/pybind11
)

# Install .cmake files
install(
  FILES
  ${PROJECT_BINARY_DIR}/share/cmake/pybind11/FindPythonLibsNew.cmake
  ${PROJECT_BINARY_DIR}/share/cmake/pybind11/pybind11Config.cmake
  ${PROJECT_BINARY_DIR}/share/cmake/pybind11/pybind11ConfigVersion.cmake
  ${PROJECT_BINARY_DIR}/share/cmake/pybind11/pybind11Targets.cmake
  ${PROJECT_BINARY_DIR}/share/cmake/pybind11/pybind11Tools.cmake
  DESTINATION share/pybind11/cmake
)

ament_export_include_directories(include)
ament_package()
