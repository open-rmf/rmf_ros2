#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>

#include <memory>
#include <random>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/clone_ptr.hpp>
#include <rmf_utils/optional.hpp>

namespace py = pybind11;

// PYBIND11_DECLARE_HOLDER_TYPE(T, rmf_utils::unique_impl_ptr<T>)
// PYBIND11_DECLARE_HOLDER_TYPE(T, rmf_utils::clone_ptr<T>)
// PYBIND11_DECLARE_HOLDER_TYPE(T, std::weak_ptr<T>)

// PYBIND11_DECLARE_HOLDER_TYPE(T, rmf_utils::clone_ptr<T>)

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
      .def_property_readonly("value", py::overload_cast<> \
          (&rmf_utils::optional<std::size_t>::value));

  py::class_<rmf_utils::optional<double> >(m_type, "OptionalDouble")
      .def_property_readonly("has_value",
                             &rmf_utils::optional<double>::has_value)
      .def_property_readonly("value", py::overload_cast<> \
          (&rmf_utils::optional<double>::value));

  py::class_<rmf_utils::optional<Eigen::Vector2d> >(m_type, "OptionalVector2D")
      .def_property_readonly("has_value",
                             &rmf_utils::optional<Eigen::Vector2d>::has_value)
      .def_property_readonly("value", py::overload_cast<> \
          (&rmf_utils::optional<Eigen::Vector2d>::value));
}
