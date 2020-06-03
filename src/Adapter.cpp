#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <memory>

#include "rmf_mock_adapter/adapter.hpp"
// #include "rmf_mock_adapter_python/_PySharedRobotCommandHandle.hpp"
// #include "rmf_mock_adapter_python/_PyRobotCommandHandle.hpp"
#include "rmf_mock_adapter_python/PyRobotCommandHandle.hpp"

namespace py = pybind11;
namespace adpt = rmf_mock_adapter;

void bind_types(py::module &);
void bind_tests(py::module &);

PYBIND11_MODULE(rmf_adapter, m) {
    bind_types(m);
    bind_tests(m);

    // Abstract Base Class: RobotCommandHandle
    // py::class_<adpt::RobotCommandHandle>(m, "RobotCommandHandle")
    //     .def("follow_new_path", &adpt::RobotCommandHandle::follow_new_path)
    //     .def("dock", &adpt::RobotCommandHandle::dock);

    // py::class_<adpt::RobotCommandHandle, _PyRobotCommandHandle,
    //            std::shared_ptr<_PyRobotCommandHandle> >
    //   (m, "RobotCommandHandle")  //, py::dynamic_attr())
    //     .def(py::init<>())
    //     .def("follow_new_path", &adpt::RobotCommandHandle::follow_new_path)
    //     .def("dock", &adpt::RobotCommandHandle::dock);

    // py::class_<SharedRobotCommandHandle, _PySharedRobotCommandHandle,
    //            std::shared_ptr<SharedRobotCommandHandle> >
    //   (m, "PyRobotCommandHandle")  //, py::dynamic_attr())
    //     .def(py::init<>())
    //     .def("follow_new_path", &SharedRobotCommandHandle::follow_new_path)
    //     .def("dock", &SharedRobotCommandHandle::dock)
    //     .def("get_ptr", &SharedRobotCommandHandle::get_ptr);


    py::class_<adpt::RobotCommandHandle, PyRobotCommandHandle,
               std::shared_ptr<adpt::RobotCommandHandle> >
      (m, "RobotCommandHandle")  //, py::dynamic_attr())
        .def(py::init<>())
        .def("follow_new_path", &adpt::RobotCommandHandle::follow_new_path)
        .def("dock", &adpt::RobotCommandHandle::dock);
        // .def("get_ptr", &PyRobotCommandHandle::get_ptr);

    // py::class_<RobotCommandHandleInstance,
    //            PyRobotCommandHandleInstance,
    //            std::shared_ptr<RobotCommandHandleInstance> >
    //   (m, "RobotCommandHandleInstance", py::dynamic_attr())
    //     .def(py::init<>())
    //     // Methods
    //     .def("follow_new_path", &RobotCommandHandleInstance::follow_new_path)
    //     .def("dock", &RobotCommandHandleInstance::dock)
    //     .def("step", &RobotCommandHandleInstance::step)
    //     .def("interruped", &RobotCommandHandleInstance::interruped)
    //     // .def("get_ptr", &RobotCommandHandleInstance::get_ptr)
    //     // Attriutes
    //     .def_readwrite("rng", &RobotCommandHandleInstance::rng)
    //     .def_readwrite("updater", &RobotCommandHandleInstance::updater)
    //     .def_readwrite("_current_path_index",
    //                    &RobotCommandHandleInstance::_current_path_index)
    //     .def_readwrite("_waypoints", &RobotCommandHandleInstance::_waypoints)
    //     .def_readwrite("_path_finished_callback",
    //                    &RobotCommandHandleInstance::_path_finished_callback);
}
