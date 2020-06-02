#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <memory>
#include <random>

#include <rmf_utils/impl_ptr.hpp>

namespace py = pybind11;

PYBIND11_DECLARE_HOLDER_TYPE(T, rmf_utils::unique_impl_ptr<T>)
PYBIND11_DECLARE_HOLDER_TYPE(T, std::weak_ptr<T>)

void bind_types(py::module &m) {
  // Wrapper for the std::mt19937 class
  py::class_<std::mt19937>(m, "mt19937")
    .def(py::init<int>())
    .def(py::init<unsigned int>())
    .def_static("min", &std::mt19937::min)
    .def_static("max", &std::mt19937::max)
    // .def("seed", &std::mt19937::seed) // Doesn't exist for some reason
    .def("discard", &std::mt19937::discard)
    .def("__call__", &std::mt19937::operator());
}
