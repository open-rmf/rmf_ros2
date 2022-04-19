#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Route.hpp>

namespace py = pybind11;
namespace schedule = rmf_traffic::schedule;

using SystemTimePoint = std::chrono::time_point<std::chrono::system_clock,
    std::chrono::nanoseconds>;

// TODO(YL): create a module to manage Time and Duration input and output
rmf_traffic::Time to_rmf_time(SystemTimePoint time)
{
  return rmf_traffic::Time(time.time_since_epoch());
}

//==============================================================================
void bind_schedule(py::module& m)
{
  auto m_schedule = m.def_submodule("schedule");

  /// PARTICIPANT ==============================================================
  py::class_<schedule::Participant,
    std::shared_ptr<schedule::Participant>>(
    m_schedule, "Participant")
  .def_property_readonly("id", &schedule::Participant::id)
  .def_property_readonly("version", &schedule::Participant::id)
  .def_property_readonly(
    "current_plan_id",
    &schedule::Participant::current_plan_id)
  .def_property("delay",
    py::overload_cast<>(
      &schedule::Participant::delay, py::const_),
    py::overload_cast<
      rmf_traffic::Duration>(&schedule::Participant::delay))
  .def("clear", &schedule::Participant::clear)
  .def("get_itinerary", &schedule::Participant::itinerary)
  .def("set_itinerary",
    [&](schedule::Participant& self,
      const std::vector<rmf_traffic::Route>& itinerary)
    {
      self.set(self.assign_plan_id(), itinerary);
    },
    py::arg("itinerary"));

  /// ROUTE ====================================================================
  /// TODO(YL) The current organization of modules differ slighty with the
  /// rmf_core directory structure. e.g. class Route should go directly
  /// rmf_traffic. Will need to adfdress all these in near future, a fix
  /// will be to create a generic python bindings for rmf_core itself.
  py::class_<rmf_traffic::Route,
    std::shared_ptr<rmf_traffic::Route>>(m_schedule, "Route")
  .def(py::init<std::string&, rmf_traffic::Trajectory>(),
    py::arg("map"),
    py::arg("trajectory"))
  .def_property("map",
    py::overload_cast<>(
      &rmf_traffic::Route::map, py::const_),
    py::overload_cast<
      std::string>(&rmf_traffic::Route::map))
  .def_property("trajectory",
    py::overload_cast<>(
      &rmf_traffic::Route::trajectory, py::const_),
    py::overload_cast<
      rmf_traffic::Trajectory>(&rmf_traffic::Route::trajectory));

  // MAKE TRAJECTORY FUNCTION ==================================================
  m_schedule.def(
    "make_trajectory",
    [](const rmf_traffic::agv::VehicleTraits& traits,
    SystemTimePoint start_time,
    const std::vector<Eigen::Vector3d>& input_positions)
    {
      return rmf_traffic::agv::Interpolate::positions(
        traits, to_rmf_time(start_time), input_positions);
    },
    py::arg("traits"),
    py::arg("start_time"),
    py::arg("input_positions"));

  // TRAJECTORY ================================================================
  py::class_<rmf_traffic::Trajectory,
    std::shared_ptr<rmf_traffic::Trajectory>>(m_schedule, "Trajectory")
  .def(py::init<>())
  .def("size", &rmf_traffic::Trajectory::size)
  .def("insert",
    [&](rmf_traffic::Trajectory& self,
    double seconds,
    Eigen::Vector3d position,
    Eigen::Vector3d velocity)
    {
      self.insert(
        rmf_traffic::Time(rmf_traffic::time::from_seconds(seconds)),
        position,
        velocity);
    },
    py::arg("time"),
    py::arg("position"),
    py::arg("velocity"))
  .def("position", [](const rmf_traffic::Trajectory& self, std::size_t index)
    {
      return self.at(index).position();
    },
    py::arg("index"))
  .def("velocity", [](const rmf_traffic::Trajectory& self, std::size_t index)
    {
      return self.at(index).velocity();
    },
    py::arg("index"))
  .def("time", [](const rmf_traffic::Trajectory& self, std::size_t index)
    {
      return rmf_traffic::time::to_seconds(
        self.at(index).time().time_since_epoch());
    },
    py::arg("index"));
}
