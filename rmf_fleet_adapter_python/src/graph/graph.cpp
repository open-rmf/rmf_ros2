#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>
#include <random>

#include <rmf_utils/clone_ptr.hpp>
#include "rmf_fleet_adapter/agv/Adapter.hpp"
#include "rmf_fleet_adapter/agv/parse_graph.hpp"

#include "rmf_fleet_adapter_python/graph/PyOrientationConstraint.hpp"

namespace py = pybind11;

void bind_lane(py::module&);

using Duration = rmf_traffic::Duration;
using Graph = rmf_traffic::agv::Graph;
using Lane = rmf_traffic::agv::Graph::Lane;

using OrientationConstraint = Graph::OrientationConstraint;

void bind_graph(py::module& m)
{
  auto m_graph = m.def_submodule("graph");

  // WAYPOINT ==================================================================
  py::class_<Graph::Waypoint>(m_graph, "Waypoint")
  // TODO(CH3): Find a way to instantiate the templated constructor
  // Constructor must be explicitly instantiated
  // Somehow this still results in an incomplete type...
  // .def_static(py::init(&Graph::Waypoint::Implementation::make<std::size_t,
  //                                                      std::string,
  //                                                      Eigen::Vector2d,
  //                                                      bool>))
  // Bind getters and setters to Python instance attributes/properties
  .def_property("map_name",
    &Graph::Waypoint::get_map_name,
    &Graph::Waypoint::set_map_name)
  .def_property("location",
    &Graph::Waypoint::get_location,
    &Graph::Waypoint::set_location)
  .def_property("holding_point",
    &Graph::Waypoint::is_holding_point,
    &Graph::Waypoint::set_holding_point)
  .def("set_holding_point", &Graph::Waypoint::set_holding_point)
  .def_property("passthrough_point",
    &Graph::Waypoint::is_passthrough_point,
    &Graph::Waypoint::set_passthrough_point)
  .def("set_passthrough_point", &Graph::Waypoint::set_passthrough_point)
  .def_property("parking_spot",
    &Graph::Waypoint::is_parking_spot,
    &Graph::Waypoint::set_parking_spot)
  .def("set_parking_spot", &Graph::Waypoint::set_parking_spot)
  .def_property("charger",
    &Graph::Waypoint::is_charger,
    &Graph::Waypoint::set_charger)
  .def("set_charger", &Graph::Waypoint::set_charger)
  .def_property_readonly("index", &Graph::Waypoint::index)
  .def_property_readonly("waypoint_name", &Graph::Waypoint::name);

  // ORIENTATION_CONSTRAINT ====================================================
  py::class_<OrientationConstraint,
    PyOrientationConstraint>(m_graph,
    "OrientationConstraint",
    py::dynamic_attr())
  .def(py::init<>())
  .def_static("make",
    py::overload_cast<std::vector<double>>(
      &OrientationConstraint::make),
    "Create an acceptable orientation constraint pointer.")
  .def_static("make",
    py::overload_cast<OrientationConstraint::Direction,
    const Eigen::Vector2d&>(
      &OrientationConstraint::make),
    "Create a direction constraint pointer.")
  .def("apply",
    &OrientationConstraint::apply)
  .def("clone",
    &OrientationConstraint::clone);

  // clone_ptr
  py::class_<rmf_utils::clone_ptr<OrientationConstraint>>(
    m_graph, "OrientationConstraintPtr")
  .def(py::init<>());
  // .def(py::init<OrientationConstraint::Direction,
  //               std::vector<double> >());
  // TODO(CH3): Potentially allow getting and setting of members
  // as well as pointed to member methods

  // Internal Direction enum class
  py::enum_<OrientationConstraint::Direction>(
    m_graph, "Direction")
  .value("Forward", OrientationConstraint::Direction::Forward)
  .value("Backward", OrientationConstraint::Direction::Backward);
  // No export_value as the enum is an enum class (scoped enum)

  // BIND LANE =================================================================
  // This MUST be specified after! Order matters!
  // It depends on types listed prior to this invocation
  bind_lane(m_graph);

  // GRAPH =====================================================================
  py::class_<Graph>(m_graph, "Graph")
  .def(py::init<>())

  // Waypoints
  .def("add_waypoint", &Graph::add_waypoint,
    py::arg("map_name"),
    py::arg("location"),
    py::return_value_policy::reference_internal)
  .def("get_waypoint",
    py::overload_cast<std::size_t>(&Graph::get_waypoint),
    py::return_value_policy::reference_internal)
  .def("get_waypoint", py::overload_cast<std::size_t>(
      &Graph::get_waypoint, py::const_),
    py::return_value_policy::reference_internal)
  .def("find_waypoint", py::overload_cast<const std::string&>(
      &Graph::find_waypoint),
    py::return_value_policy::reference_internal)
  .def("find_waypoint", py::overload_cast<const std::string&>(
      &Graph::find_waypoint, py::const_),
    py::return_value_policy::reference_internal)
  .def_property_readonly("num_waypoints", &Graph::num_waypoints)

  // Keys
  .def("add_key", &Graph::add_key)
  .def("remove_key", &Graph::remove_key)
  .def("set_key", &Graph::set_key)
  .def_property_readonly("keys", &Graph::keys)

  // Lanes
  .def("add_lane",
    &Graph::add_lane,
    py::arg("entry"),
    py::arg("exit"),
    py::arg("properties") = Lane::Properties())
  .def("add_dock_lane",
    [&](Graph& self,
    const std::size_t w0,
    const std::size_t w1,
    std::string dock_name,
    int seconds,
    Lane::Properties properties)
    {
      self.add_lane(
        {
          w0,
          Lane::Event::make(
            Lane::Dock(dock_name,
            std::chrono::seconds(seconds)))
        },
        w1, properties);
      self.add_lane(w1, w0, properties);
    },
    py::arg("w0"),
    py::arg("w1"),
    py::arg("dock_name"),
    py::arg("dock_seconds") = 10,
    py::arg("properties") = Lane::Properties())
  .def("add_bidir_lane",
    [&](Graph& self,
    const std::size_t w0,
    const std::size_t w1)
    {
      self.add_lane(w0, w1);
      self.add_lane(w1, w0);
    })
  .def("get_lane", py::overload_cast<std::size_t>(&Graph::get_lane))
  .def("get_lane", py::overload_cast<std::size_t>(
      &Graph::get_lane, py::const_))
  .def(
    "lane_from",
    py::overload_cast<std::size_t, std::size_t>(&Graph::lane_from),
    py::return_value_policy::reference_internal)
  .def(
    "lane_from",
    py::overload_cast<std::size_t, std::size_t>(
      &Graph::lane_from, py::const_),
    py::return_value_policy::reference_internal)
  .def_property_readonly("num_lanes", &Graph::num_lanes)
  .def("lanes_from_waypoint",
    py::overload_cast<std::size_t>(&Graph::lanes_from, py::const_),
    py::arg("wp_index"));

  // PARSE GRAPH ==============================================================
  // Helper function to parse a graph from a yaml file
  m_graph.def("parse_graph", &rmf_fleet_adapter::agv::parse_graph);
}
