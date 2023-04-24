#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "rmf_traffic_ros2/Time.hpp"
#include <rmf_traffic/agv/Planner.hpp>

namespace py = pybind11;

using Plan = rmf_traffic::agv::Plan;
using Planner = rmf_traffic::agv::Planner;

using Start = Planner::Start;
using StartSet = Planner::StartSet;
using Goal = Planner::Goal;
using Options = Planner::Options;
using Configuration = Planner::Configuration;
using Result = Planner::Result;
using Graph = rmf_traffic::agv::Graph;
using VehicleTraits = rmf_traffic::agv::VehicleTraits;
using Interpolate = rmf_traffic::agv::Interpolate;

using TimePoint = std::chrono::time_point<std::chrono::system_clock,
    std::chrono::nanoseconds>;

// NOTE(CH3):
// Factory method for Start() to allow passing in of system_clock::time_points,
// as Start objects are constructed using steady_clock::time_points
//
// We use system_clock::time_points in the bindings to simplify the Pythonside
// types (so it casts to datetime.datetime instead of datetime.timedelta)
Start make_start(TimePoint initial_time,
  std::size_t initial_waypoint,
  double initial_orientation,
  rmf_utils::optional<Eigen::Vector2d> location,
  rmf_utils::optional<std::size_t> initial_lane)
{
  using TimePointSteadyClock =
    std::chrono::time_point<std::chrono::steady_clock,
      std::chrono::nanoseconds>;

  return Start(TimePointSteadyClock(initial_time.time_since_epoch()),
      initial_waypoint,
      initial_orientation,
      location,
      initial_lane);
}

Planner make_planner(Configuration config)
{
  const auto default_options = Options{nullptr};
  return Planner(config, default_options);
}

void bind_plan(py::module& m)
{
  auto m_plan = m.def_submodule("plan");

  // PLANNER ===================================================================
  py::class_<Start>(m_plan, "Start")
  .def(py::init(&make_start),
    py::arg("initial_time"),
    py::arg("initial_waypoint"),
    py::arg("initial_orientation"),
    py::arg("location") = rmf_utils::optional<Eigen::Vector2d>(
      rmf_utils::nullopt),
    py::arg("initial_lane") = rmf_utils::optional<std::size_t>(
      rmf_utils::nullopt))
  .def_property("time",
    py::overload_cast<>(&Start::time, py::const_),
    py::overload_cast<rmf_traffic::Time>(&Start::time))
  .def_property("waypoint",
    py::overload_cast<>(&Start::waypoint, py::const_),
    py::overload_cast<std::size_t>(&Start::waypoint))
  .def_property("orientation",
    py::overload_cast<>(&Start::orientation, py::const_),
    py::overload_cast<double>(&Start::orientation))
  .def_property("location",
    py::overload_cast<>(&Start::location, py::const_),
    py::overload_cast<rmf_utils::optional<Eigen::Vector2d>>(
      &Start::location))
  .def_property("lane",
    py::overload_cast<>(&Start::lane, py::const_),
    py::overload_cast<rmf_utils::optional<std::size_t>>(
      &Start::lane));

  // Rebound signature for compute_plan_starts
  m_plan.def("compute_plan_starts",
    [&](const rmf_traffic::agv::Graph& graph,
    const std::string& map_name,
    const Eigen::Vector3d pose,
    TimePoint start_time,
    const double max_merge_waypoint_distance,
    const double max_merge_lane_distance,
    const double min_lane_length)
    {
      using TimePointSteadyClock =
      std::chrono::time_point<std::chrono::steady_clock,
      std::chrono::nanoseconds>;

      return rmf_traffic::agv::compute_plan_starts(
        graph,
        map_name,
        pose,
        TimePointSteadyClock(start_time.time_since_epoch()),
        max_merge_waypoint_distance,
        max_merge_lane_distance,
        min_lane_length
      );
    },
    py::arg("navigation_graph"),
    py::arg("map_name"),
    py::arg("position"),
    py::arg("start_time"),
    py::arg("max_merge_waypoint_distance") = 0.1,
    py::arg("max_merge_lane_distance") = 1.0,
    py::arg("min_lane_length") = 1e-8);

  // PLAN ======================================================================
  py::class_<Plan>(m_plan, "Plan")
  // Private constructor
  .def_property_readonly("itinerary",
    &Plan::get_itinerary)
  .def_property_readonly("waypoints",
    &Plan::get_waypoints)
  .def_property_readonly("start",
    &Plan::get_start);

  // WAYPOINT ==================================================================
  py::class_<Plan::Waypoint>(m_plan, "Waypoint")
  // Private constructor
  .def_property_readonly("position",
    &Plan::Waypoint::position)
  .def_property_readonly("time", [](Plan::Waypoint& self)
    {
      // The time count is in nanoseconds!
      return TimePoint(self.time().time_since_epoch());
    })
  .def_property_readonly("graph_index",
    &Plan::Waypoint::graph_index)
  .def_property_readonly("approach_lanes",
    &Plan::Waypoint::approach_lanes)
  .def_property_readonly("itinerary_index",
    &Plan::Waypoint::itinerary_index)
  .def_property_readonly("trajectory_index",
    &Plan::Waypoint::trajectory_index)
  .def_property_readonly("event",
    &Plan::Waypoint::event);

  // GOAL ======================================================================
  py::class_<Plan::Goal>(m_plan, "Goal")
  .def(py::init<std::size_t>())
  .def(py::init<std::size_t, double>())
  .def_property("waypoint",
    py::overload_cast<>(&Plan::Goal::waypoint, py::const_),
    py::overload_cast<std::size_t>(&Plan::Goal::waypoint))
  .def_property("orientation",
    py::overload_cast<>(&Plan::Goal::orientation, py::const_),
    py::overload_cast<double>(&Plan::Goal::orientation))
  .def("any_orientation", &Plan::Goal::any_orientation);

  // Configuration =============================================================
  py::class_<Configuration>(m_plan, "Configuration")
  .def(py::init<Graph, VehicleTraits>(),
    py::arg("graph"),
    py::arg("traits"))
  .def_property_readonly("graph", &Configuration::graph)
  .def_property_readonly("traits", &Configuration::vehicle_traits);

  // Options =============================================================
  // TODO

  // Planner ===================================================================
  py::class_<Planner>(m_plan, "Planner")
  .def(py::init([&](Configuration& config)
    {
      return new Planner(config, Planner::Options{nullptr});
    }),
    py::arg("config"))
  .def("get_plan_waypoints",
    [&](Planner& self,
    Start start,
    Goal goal)
    {
      std::vector<Plan::Waypoint> waypoints;
      const auto result = self.plan(start, goal);
      if (result.success())
      {
        waypoints = result->get_waypoints();
      }

      return waypoints;

    },
    py::arg("start"), py::arg("goal"),
    py::return_value_policy::reference_internal);

}
