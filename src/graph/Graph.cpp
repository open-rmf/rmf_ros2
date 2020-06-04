#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>
#include <random>

#include <rmf_mock_adapter/adapter.hpp>
#include <rmf_utils/clone_ptr.hpp>

#include "rmf_mock_adapter_python/graph/PyOrientationConstraint.hpp"
#include "rmf_mock_adapter_python/graph/PyVelocityConstraint.hpp"

namespace py = pybind11;

void bind_lane(py::module &);

using Duration = rmf_traffic::Duration;
using Graph = rmf_traffic::agv::Graph;
using Lane = rmf_traffic::agv::Graph::Lane;

using OrientationConstraint = Graph::OrientationConstraint;
using VelocityConstraint = Graph::VelocityConstraint;

void bind_graph(py::module &m) {
  auto m_graph = m.def_submodule("Graph");

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
      .def_property_readonly("index", &Graph::Waypoint::index);

  // ORIENTATION_CONSTRAINT ====================================================
  py::class_<OrientationConstraint,
             PyOrientationConstraint>(m_graph,
                                      "OrientationConstraint",
                                      py::dynamic_attr())
      .def(py::init<>())
      .def_static("make",
                  py::overload_cast<std::vector<double> > \
                      (&OrientationConstraint::make),
                      "Create an acceptable orientation constraint pointer.")
      .def_static("make",
                  py::overload_cast<OrientationConstraint::Direction,
                                    const Eigen::Vector2d&> \
                      (&OrientationConstraint::make),
                      "Create a direction constraint pointer.")
      .def("apply",
           &OrientationConstraint::apply)
      .def("clone",
           &OrientationConstraint::clone);

  // clone_ptr
  py::class_<rmf_utils::clone_ptr<OrientationConstraint> > \
    (m_graph, "OrientationConstraintPtr")
      .def(py::init<>());
      // .def(py::init<OrientationConstraint::Direction,
      //               std::vector<double> >());
      // TODO(CH3): Potentially allow getting and setting of members
      // as well as pointed to member methods

  // Internal Direction enum class
  py::enum_<OrientationConstraint::Direction> \
    (m_graph, "Direction")
      .value("Forward", OrientationConstraint::Direction::Forward)
      .value("Backward", OrientationConstraint::Direction::Backward);
      // No export_value as the enum is an enum class (scoped enum)

  // VELOCITY CONSTRAINT =======================================================
  py::class_<VelocityConstraint,
             PyVelocityConstraint,
             std::unique_ptr<VelocityConstraint> >(m_graph,
                                                   "VelocityConstraint",
                                                   py::dynamic_attr())
      .def(py::init<>())
      .def("apply",
           &VelocityConstraint::apply);
      // TODO(CH3): Get this to work eventually when needed
      // .def("clone",
      //      &PyVelocityConstraint::clone_wrapper);

  // clone_ptr
  py::class_<rmf_utils::clone_ptr<VelocityConstraint> > \
    (m_graph, "VelocityConstraintPtr")
      .def(py::init<>());
      // .def(py::init<OrientationConstraint::Direction,
                    // std::vector<double> >());
      // TODO(CH3): Potentially allow getting and setting of members
      // as well as pointed to member methods

  // BIND LANE =================================================================
  // This MUST be specified after! Order matters!
  // It depends on types listed prior to this invocation
  bind_lane(m_graph);

  // GRAPH =====================================================================
  py::class_<Graph>(m_graph, "Graph")
      .def(py::init<>())
      .def("add_waypoint", &Graph::add_waypoint,
           py::arg("map_name"),
           py::arg("location"),
           py::arg("is_holding_point") = false)
      .def("get_waypoint", py::overload_cast<std::size_t>(&Graph::get_waypoint))
      .def("get_waypoint", py::overload_cast<std::size_t>(
          &Graph::get_waypoint, py::const_))
      .def("num_waypoints", &Graph::num_waypoints)
      .def_property_readonly("num_waypoints", &Graph::num_waypoints)
      .def("add_lane",
           &Graph::add_lane,
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("add_bidir_lane",
           [&](Graph &self,
               const std::size_t w0,
               const std::size_t w1){
                   self.add_lane(w0, w1);
                   self.add_lane(w1, w0);},
           py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
      .def("get_lane", py::overload_cast<std::size_t>(&Graph::get_lane))
      .def("get_lane", py::overload_cast<std::size_t>(
          &Graph::get_lane, py::const_))
      .def("num_lanes", &Graph::num_lanes)
      .def_property_readonly("num_lanes", &Graph::num_lanes);
}
