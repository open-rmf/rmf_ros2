#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>

#include <memory>
#include <random>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/clone_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>

namespace py = pybind11;

rmf_task_msgs::msg::Delivery make_delivery_msg(
  std::string task_id,
  std::string pickup_place_name,
  std::string pickup_dispenser,
  std::string dropoff_place_name,
  std::string dropoff_dispenser)
{
  rmf_task_msgs::msg::Delivery request;
  request.task_id = task_id;

  request.pickup_place_name = pickup_place_name;
  request.pickup_dispenser = pickup_dispenser;

  request.dropoff_place_name = dropoff_place_name;
  request.dropoff_dispenser = dropoff_dispenser;

  return request;
}

void bind_types(py::module &m) {
  auto m_type = m.def_submodule("type");

  // Wrapper for the std::mt19937 class
  py::class_<std::mt19937>(m_type, "mt19937")
    .def(py::init<int>())
    .def(py::init<unsigned int>())
    .def_static("min", &std::mt19937::min)
    .def_static("max", &std::mt19937::max)
    // .def("seed", &std::mt19937::seed) // Doesn't exist for some reason
    .def("discard", &std::mt19937::discard)
    .def("__call__", &std::mt19937::operator());

  py::class_<rmf_utils::optional<std::size_t> >(m_type, "OptionalULong")
      .def_property_readonly("has_value",
                             &rmf_utils::optional<std::size_t>::has_value)
      .def_property_readonly("value", py::overload_cast<>(
          &rmf_utils::optional<std::size_t>::value));

  py::class_<rmf_utils::optional<double> >(m_type, "OptionalDouble")
      .def_property_readonly("has_value",
                             &rmf_utils::optional<double>::has_value)
      .def_property_readonly("value", py::overload_cast<>(
          &rmf_utils::optional<double>::value));

  py::class_<rmf_utils::optional<Eigen::Vector2d> >(m_type, "OptionalVector2D")
      .def_property_readonly("has_value",
                             &rmf_utils::optional<Eigen::Vector2d>::has_value)
      .def_property_readonly("value", py::overload_cast<>(
          &rmf_utils::optional<Eigen::Vector2d>::value));

  py::class_<rmf_utils::nullopt_t>(m_type, "NullOptional");

  py::class_<rmf_task_msgs::msg::Delivery>(m_type, "CPPDeliveryMsg")
      .def(py::init(&make_delivery_msg),
           py::arg("task_id") = "",
           py::arg("pickup_place_name") = "",
           py::arg("pickup_dispenser") = "",
           py::arg("dropoff_place_name") = "",
           py::arg("dropoff_dispenser") = "");
}
