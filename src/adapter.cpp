#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>

#include "rmf_mock_adapter/adapter.hpp"
#include "rmf_mock_adapter_python/PyRobotCommandHandle.hpp"

namespace py = pybind11;
namespace adpt = rmf_mock_adapter;

void bind_types(py::module &);
void bind_tests(py::module &);
void bind_graph(py::module &);
void bind_shapes(py::module &);
void bind_vehicletraits(py::module &);
void bind_plan(py::module &);

PYBIND11_MODULE(rmf_adapter, m) {
    bind_types(m);
    bind_tests(m);
    bind_graph(m);
    bind_shapes(m);
    bind_vehicletraits(m);
    bind_plan(m);

    // ROBOTCOMMAND HANDLE =====================================================
    py::class_<adpt::RobotCommandHandle, PyRobotCommandHandle,
               std::shared_ptr<adpt::RobotCommandHandle> > \
      (m, "RobotCommandHandle", py::dynamic_attr())
        .def(py::init<>())
        .def("follow_new_path", &adpt::RobotCommandHandle::follow_new_path)
        .def("dock", &adpt::RobotCommandHandle::dock);

    // ROBOTUPDATE HANDLE ======================================================
    py::class_<adpt::RobotUpdateHandle,
               std::shared_ptr<adpt::RobotUpdateHandle> > \
      (m, "RobotUpdateHandle")
        // .def(py::init<>())  // Only to be constructed via FleetUpdateHandle!
        .def("add_delay", &adpt::RobotUpdateHandle::add_delay,
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("interrupted", &adpt::RobotUpdateHandle::interrupted,
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_position",
             py::overload_cast<std::size_t, double> \
               (&adpt::RobotUpdateHandle::update_position),
             py::arg("waypoint"),
             py::arg("orientation"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_position",
             py::overload_cast<const Eigen::Vector3d&,
                               const std::vector<std::size_t>& > \
               (&adpt::RobotUpdateHandle::update_position),
             py::arg("position"),
             py::arg("lanes"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def("update_position",
             py::overload_cast<const std::string&,
                               const Eigen::Vector3d&> \
               (&adpt::RobotUpdateHandle::update_position),
             py::arg("map_name"),
             py::arg("position"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>());

    // FLEETUPDATE HANDLE ======================================================
    py::class_<adpt::FleetUpdateHandle>(m, "FleetUpdateHandle")
        // NOTE(CH3): Might have to publicise this constructor if required
        // Otherwise, it's constructable via adpt::TestScenario
        // .def(py::init<>())  // Private constructor!
        .def("add_robot",
             py::overload_cast<std::shared_ptr<adpt::RobotCommandHandle>,
                               const std::string&,
                               const rmf_traffic::Profile&,
                               std::size_t,
                               double> \
               (&adpt::FleetUpdateHandle::add_robot),
             py::arg("command_handle"),
             py::arg("robot_name"),
             py::arg("robot_profile"),
             py::arg("initial_waypoint"),
             py::arg("orientation"))
        .def("add_robot",
             py::overload_cast<std::shared_ptr<adpt::RobotCommandHandle>,
                               const std::string&,
                               const rmf_traffic::Profile&,
                               const Eigen::Vector3d&,
                               const std::vector<std::size_t>& > \
               (&adpt::FleetUpdateHandle::add_robot),
             py::arg("command_handle"),
             py::arg("robot_name"),
             py::arg("robot_profile"),
             py::arg("initial_position"),
             py::arg("initial_lanes"));

    // TESTSCENARIO ============================================================
    py::class_<adpt::TestScenario /*,
               std::unique_ptr<adpt::TestScenario, py::nodelete>*/ > \
               (m, "TestScenario")
        .def(py::init<>())
        .def("add_fleet", &adpt::TestScenario::add_fleet,
             py::arg("fleet_name"),
             py::arg("nav_graph"),
             py::arg("vehicle_traits"))
        .def("test", &adpt::TestScenario::test, py::arg("conditions"),
             py::call_guard<py::scoped_ostream_redirect,
                            py::scoped_estream_redirect>())
        .def_property_readonly("finished", &adpt::TestScenario::finished);

    py::class_<adpt::TestScenario::Condition>(m, "Condition")
        .def(py::init<std::shared_ptr<adpt::RobotUpdateHandle>,
                      rmf_traffic::agv::Plan::Goal>())
        .def_readwrite("robot", &adpt::TestScenario::Condition::robot)
        .def_readwrite("goal", &adpt::TestScenario::Condition::goal);
}
