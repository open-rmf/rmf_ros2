#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <memory>
#include <random>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include "rmf_fleet_adapter_python/graph/PyExecutor.hpp"
#include "rmf_fleet_adapter_python/graph/PyEvent.hpp"
#include <rmf_utils/clone_ptr.hpp>

namespace py = pybind11;

using Duration = rmf_traffic::Duration;
using Graph = rmf_traffic::agv::Graph;
using Lane = rmf_traffic::agv::Graph::Lane;

void bind_lane(py::module& m)
{
  auto m_lane = m.def_submodule("lane");

  // LANE ==================================================================
  py::class_<Lane>(m_lane, "Lane")
  // TODO(CH3): Find a way to instantiate the templated constructor
  // Constructor must be explicitly instantiated
  // Somehow this still results in an incomplete type...
  // .def_static(py::init(&Lane::Implementation::make<std::size_t,
  //Lane::Node,
  //Lane::Node,
  //bool,
  //std::size_t>))
  //Bind getters and setters to Python instance attributes/properties
  .def_property_readonly("entry",
    py::overload_cast<>(&Lane::entry),
    py::return_value_policy::reference_internal)
  .def_property_readonly("exit",
    py::overload_cast<>(&Lane::exit),
    py::return_value_policy::reference_internal)
  .def_property_readonly("index", &Lane::index)
  .def_property_readonly("properties",
    py::overload_cast<>(&Lane::properties),
    py::return_value_policy::reference_internal);

  // LANE PROPERTIES ===========================================================
  py::class_<Lane::Properties>(m_lane, "Properties")
  .def(py::init<>())
  .def_property("speed_limit",
    py::overload_cast<>(&Lane::Properties::speed_limit, py::const_),
    py::overload_cast<std::optional<double>>(&Lane::Properties::speed_limit));

  // DOORS =====================================================================
  py::class_<Lane::Door>(m_lane, "Door")
  .def(py::init<std::string, Duration>(),
    py::arg("name"),
    py::arg("duration"))
  // Bind getters and setters to Python instance attributes/properties
  .def_property("name",
    py::overload_cast<>(&Lane::Door::name, py::const_),
    py::overload_cast<std::string>(&Lane::Door::name))
  .def_property("duration",
    py::overload_cast<>(&Lane::Door::duration, py::const_),
    py::overload_cast<Duration>(&Lane::Door::duration));

  py::class_<Lane::DoorOpen, Lane::Door>(m_lane, "DoorOpen")
  .def(py::init<std::string, Duration>(),
    py::arg("name"),
    py::arg("duration"));

  py::class_<Lane::DoorClose, Lane::Door>(m_lane, "DoorClose")
  .def(py::init<std::string, Duration>(),
    py::arg("name"),
    py::arg("duration"));

  // LIFTS =====================================================================
  // Doors
  py::class_<Lane::LiftSession>(m_lane, "LiftSession")
  .def(py::init<std::string, std::string, Duration>(),
    py::arg("lift_name"),
    py::arg("floor_name"),
    py::arg("duration"))
  .def_property(
    "lift_name",
    py::overload_cast<>(&Lane::LiftSession::lift_name, py::const_),
    py::overload_cast<std::string>(&Lane::LiftSession::lift_name))
  .def_property(
    "floor_name",
    py::overload_cast<>(&Lane::LiftSession::floor_name, py::const_),
    py::overload_cast<std::string>(&Lane::LiftSession::floor_name))
  .def_property(
    "duration",
    py::overload_cast<>(&Lane::LiftSession::duration, py::const_),
    py::overload_cast<Duration>(&Lane::LiftSession::duration));

  py::class_<Lane::LiftSessionBegin, Lane::LiftSession>(m_lane,
    "LiftSessionBegin")
  .def(py::init<std::string, std::string, Duration>(),
    py::arg("lift_name"),
    py::arg("floor_name"),
    py::arg("duration"));

  py::class_<Lane::LiftMove, Lane::LiftSession>(m_lane, "LiftMove")
  .def(py::init<std::string, std::string, Duration>(),
    py::arg("lift_name"),
    py::arg("floor_name"),
    py::arg("duration"));

  py::class_<Lane::LiftDoorOpen, Lane::LiftSession>(m_lane, "LiftDoorOpen")
  .def(py::init<std::string, std::string, Duration>(),
    py::arg("lift_name"),
    py::arg("floor_name"),
    py::arg("duration"));

  py::class_<Lane::LiftSessionEnd, Lane::LiftSession>(m_lane, "LiftSessionEnd")
  .def(py::init<std::string, std::string, Duration>(),
    py::arg("lift_name"),
    py::arg("floor_name"),
    py::arg("duration"));


  // DOCK ======================================================================
  py::class_<Lane::Dock>(m_lane, "Dock")
  .def(py::init<std::string, Duration>(),
    py::arg("dock_name"),
    py::arg("duration"))
  .def_property("dock_name",
    py::overload_cast<>(&Lane::Dock::dock_name, py::const_),
    py::overload_cast<std::string>(&Lane::Dock::dock_name))
  .def_property("duration",
    py::overload_cast<>(&Lane::Dock::duration, py::const_),
    py::overload_cast<Duration>(&Lane::Dock::duration));

  // WAIT ======================================================================
  py::class_<Lane::Wait>(m_lane, "Wait")
  .def(py::init<Duration>(),
    py::arg("duration"))
  .def_property("duration",
    py::overload_cast<>(&Lane::Wait::duration, py::const_),
    py::overload_cast<Duration>(&Lane::Wait::duration));

  // EXECUTOR ==================================================================
  py::class_<Lane::Executor, PyExecutor>(m_lane, "Executor", py::dynamic_attr())
  .def(py::init<>())
  .def("door_open_execute",
    py::overload_cast<const Lane::DoorOpen&>(&Lane::Executor::execute))
  .def("door_close_execute",
    py::overload_cast<const Lane::DoorClose&>(&Lane::Executor::execute))
  .def("lift_session_begin_execute",
    py::overload_cast<const Lane::LiftSessionBegin&>(
      &Lane::Executor::execute))
  .def("lift_door_open_execute",
    py::overload_cast<const Lane::LiftDoorOpen&>(
      &Lane::Executor::execute))
  .def("lift_door_close_execute",
    py::overload_cast<const Lane::LiftSessionEnd&>(
      &Lane::Executor::execute))
  .def("lift_move_execute",
    py::overload_cast<const Lane::LiftMove&>(&Lane::Executor::execute))
  .def("dock_execute",
    py::overload_cast<const Lane::Dock&>(&Lane::Executor::execute))
  .def("wait_execute",
    py::overload_cast<const Lane::Wait&>(&Lane::Executor::execute));

  // EVENT =====================================================================
  py::class_<Lane::Event, PyEvent>(m_lane, "Event", py::dynamic_attr())
  .def(py::init<>())
  .def("duration", &Lane::Event::duration)
  .def("execute", &Lane::Event::execute<Lane::Executor&>)
  .def("clone", &Lane::Event::clone)
  .def_static("door_open_make",
    py::overload_cast<Lane::DoorOpen>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("door_close_make",
    py::overload_cast<Lane::DoorClose>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("lift_session_begin_make",
    py::overload_cast<Lane::LiftSessionBegin>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("lift_session_end_make",
    py::overload_cast<Lane::LiftSessionEnd>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("lift_move_make",
    py::overload_cast<Lane::LiftMove>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("dock_make",
    py::overload_cast<Lane::Dock>(&Lane::Event::make),
    py::return_value_policy::reference_internal)
  .def_static("wait_make",
    py::overload_cast<Lane::Wait>(&Lane::Event::make),
    py::return_value_policy::reference_internal);


  py::class_<rmf_utils::clone_ptr<Lane::Event>>(m_lane, "EventPtr")
  .def(py::init<>());

  // NODE ======================================================================
  py::class_<Lane::Node>(m_lane, "Node", py::dynamic_attr())
  .def(py::init<std::size_t,
    rmf_utils::clone_ptr<Lane::Event>,
    rmf_utils::clone_ptr<Graph::OrientationConstraint>>(),
    py::arg("waypoint_index"),
    py::arg("event") = (rmf_utils::clone_ptr<Lane::Event>)nullptr,
    py::arg("orientation") =
    (rmf_utils::clone_ptr<Graph::OrientationConstraint>)nullptr
  )
  .def(py::init<std::size_t,
    rmf_utils::clone_ptr<Graph::OrientationConstraint>>(),
    py::arg("waypoint_index"),
    py::arg("orientation") =
    (rmf_utils::clone_ptr<Graph::OrientationConstraint>)nullptr
  )
  .def_property_readonly("waypoint_index", &Lane::Node::waypoint_index)
  // Return policy for rawpointer properties defaults to reference_internal
  .def_property_readonly("event",
    py::overload_cast<>(&Lane::Node::event, py::const_))
  .def_property_readonly(
    "orientation_constraint", &Lane::Node::orientation_constraint);
}
