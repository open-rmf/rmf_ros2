set(PYBIND_PREFIX "${CMAKE_INSTALL_PREFIX}/../pybind_ament/share/pybind11/cmake")

include(${PYBIND_PREFIX}/FindPythonLibsNew.cmake)
include(${PYBIND_PREFIX}/pybind11Config.cmake)
include(${PYBIND_PREFIX}/pybind11ConfigVersion.cmake)
include(${PYBIND_PREFIX}/pybind11Targets.cmake)
include(${PYBIND_PREFIX}/pybind11Tools.cmake)
