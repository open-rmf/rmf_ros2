#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/Profile.hpp>

namespace py = pybind11;
using VehicleTraits = rmf_traffic::agv::VehicleTraits;
using Profile = rmf_traffic::Profile;

using ConstFinalConvexShapePtr =
  rmf_traffic::geometry::ConstFinalConvexShapePtr;

void bind_vehicletraits(py::module& m)
{
  auto m_vehicletraits = m.def_submodule("vehicletraits");

  // LIMITS ====================================================================
  py::class_<VehicleTraits::Limits>(m_vehicletraits, "Limits")
  .def(py::init<double, double>(),
    py::arg("velocity") = 0.0,
    py::arg("acceleration") = 0.0)
  .def_property("nominal_velocity",
    &VehicleTraits::Limits::get_nominal_velocity,
    &VehicleTraits::Limits::set_nominal_velocity)
  .def_property("nominal_acceleration",
    &VehicleTraits::Limits::get_nominal_acceleration,
    &VehicleTraits::Limits::set_nominal_acceleration)
  .def_property_readonly("valid", &VehicleTraits::Limits::valid);

  // STEERING ==================================================================
  py::enum_<VehicleTraits::Steering>(m_vehicletraits, "Steering")
  .value("Differential", VehicleTraits::Steering::Differential)
  .value("Holonomic", VehicleTraits::Steering::Holonomic);

  // DIFFERENTIAL ==============================================================
  py::class_<VehicleTraits::Differential>(m_vehicletraits, "Differential")
  .def(py::init<Eigen::Vector2d, bool>(),
    py::arg("forward") = Eigen::Vector2d::UnitX(),
    py::arg("reversible") = true)
  .def_property("forward",
    &VehicleTraits::Differential::get_forward,
    &VehicleTraits::Differential::set_forward)
  .def_property("reversible",
    &VehicleTraits::Differential::is_reversible,
    &VehicleTraits::Differential::set_reversible)
  .def_property_readonly("valid", &VehicleTraits::Differential::valid);

  // HOLONOMIC =================================================================
  py::class_<VehicleTraits::Holonomic>(m_vehicletraits, "Holonomic")
  .def(py::init<>());

  // PROFILE ===================================================================
  py::class_<Profile>(m_vehicletraits, "Profile")
  .def(py::init<ConstFinalConvexShapePtr,
    ConstFinalConvexShapePtr>(),
    py::arg("footprint"),
    py::arg("vicinity") = (
      ConstFinalConvexShapePtr) nullptr)
  .def_property("footprint",
    py::overload_cast<>(&Profile::footprint, py::const_),
    py::overload_cast<ConstFinalConvexShapePtr>(
      &Profile::footprint))
  .def_property("vicinity",
    py::overload_cast<>(&Profile::vicinity, py::const_),
    py::overload_cast<ConstFinalConvexShapePtr>(
      &Profile::vicinity));

  // VEHICLETRAITS =============================================================
  py::class_<VehicleTraits>(m_vehicletraits, "VehicleTraits")
  .def(py::init<VehicleTraits::Limits,
    VehicleTraits::Limits,
    Profile,
    VehicleTraits::Differential
    >(),
    py::arg("linear"),
    py::arg("angular"),
    py::arg("profile"),
    py::arg("steering") = VehicleTraits::Differential())
  .def_property("linear",
    py::overload_cast<>(&VehicleTraits::linear, py::const_),
    py::overload_cast<>(&VehicleTraits::linear))
  .def_property("rotational",
    py::overload_cast<>(&VehicleTraits::rotational, py::const_),
    py::overload_cast<>(&VehicleTraits::rotational))
  .def_property("profile",
    py::overload_cast<>(&VehicleTraits::profile, py::const_),
    py::overload_cast<>(&VehicleTraits::profile))
  .def_property_readonly("steering", &VehicleTraits::get_steering)
  .def_property("differential",
    py::overload_cast<>(&VehicleTraits::get_differential),
    &VehicleTraits::set_differential)
  .def_property_readonly("const_differential",
    py::overload_cast<>(&VehicleTraits::get_differential,
    py::const_))
  .def_property("holonomic",
    py::overload_cast<>(&VehicleTraits::get_holonomic),
    &VehicleTraits::set_holonomic)
  .def_property_readonly("const_holonomic",
    py::overload_cast<>(&VehicleTraits::get_holonomic,
    py::const_))
  .def_property_readonly("valid", &VehicleTraits::valid);
}
