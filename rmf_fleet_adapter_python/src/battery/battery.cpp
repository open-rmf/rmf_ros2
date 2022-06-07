#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

namespace py = pybind11;
namespace agv = rmf_battery::agv;

void bind_battery(py::module& m)
{
  auto m_battery = m.def_submodule("battery");

  //============================================================================
  py::class_<agv::BatterySystem>(m_battery, "BatterySystem")
  .def_static("make", &agv::BatterySystem::make,
    py::arg("nominal_voltage"),
    py::arg("capacity"),
    py::arg("charging_current"))
  .def_property_readonly("get_nominal_voltage",
    &agv::BatterySystem::nominal_voltage)
  .def_property_readonly("get_capacity",
    &agv::BatterySystem::capacity)
  .def_property_readonly("get_charging_current",
    &agv::BatterySystem::charging_current);

  //============================================================================
  py::class_<agv::MechanicalSystem>(m_battery, "MechanicalSystem")
  .def_static("make", &agv::MechanicalSystem::make,
    py::arg("mass"),
    py::arg("moment_of_inertia"),
    py::arg("friction_coefficient"))
  .def_property_readonly("get_mass",
    &agv::MechanicalSystem::mass)
  .def_property_readonly("get_moment_of_inertia",
    &agv::MechanicalSystem::moment_of_inertia)
  .def_property_readonly("get_friction_coefficient",
    &agv::MechanicalSystem::friction_coefficient);

  py::class_<agv::SimpleMotionPowerSink>(m_battery, "SimpleMotionPowerSink")
  .def(py::init<agv::BatterySystem&, agv::MechanicalSystem&>(),
    py::arg("battery_system"),
    py::arg("mechanical_system"))
  .def_property_readonly("get_battery_system",
    &agv::SimpleMotionPowerSink::battery_system)
  .def_property_readonly("get_mechanical_system",
    &agv::SimpleMotionPowerSink::mechanical_system);

  //============================================================================
  py::class_<agv::PowerSystem>(m_battery, "PowerSystem")
  .def_static("make", &agv::PowerSystem::make,
    py::arg("nominal_power"))
  .def_property_readonly("get_nominal_power",
    &agv::PowerSystem::nominal_power);

  py::class_<agv::SimpleDevicePowerSink>(m_battery, "SimpleDevicePowerSink")
  .def(py::init<agv::BatterySystem&, agv::PowerSystem&>(),
    py::arg("battery_system"),
    py::arg("power_system"))
  .def_property_readonly("get_battery_system",
    &agv::SimpleDevicePowerSink::battery_system)
  .def_property_readonly("get_power_system",
    &agv::SimpleDevicePowerSink::power_system)
  .def("compute_change_in_charge",
    &agv::SimpleDevicePowerSink::compute_change_in_charge,
    py::arg("run_time"));
}
