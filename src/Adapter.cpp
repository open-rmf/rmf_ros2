#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <memory>

#include "rmf_mock_adapter/adapter.hpp"
#include "rmf_mock_adapter_python/PyRobotCommandHandle.hpp"

namespace py = pybind11;
namespace adpt = rmf_mock_adapter;

void bind_types(py::module &);
void bind_tests(py::module &);
void bind_lane(py::module &);

PYBIND11_MODULE(rmf_adapter, m) {
    bind_types(m);
    bind_tests(m);
    bind_lane(m);

    py::class_<adpt::RobotCommandHandle, PyRobotCommandHandle,
               std::shared_ptr<adpt::RobotCommandHandle> >
      (m, "RobotCommandHandle")
        .def(py::init<>())
        .def("follow_new_path", &adpt::RobotCommandHandle::follow_new_path)
        .def("dock", &adpt::RobotCommandHandle::dock);
}
