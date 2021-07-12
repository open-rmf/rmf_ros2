#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rmf_traffic/geometry/Circle.hpp>
#include "rmf_fleet_adapter_python/geometry/PyConvexShape.hpp"
#include "rmf_fleet_adapter_python/geometry/PyShape.hpp"

namespace py = pybind11;
namespace geometry = rmf_traffic::geometry;

void bind_shapes(py::module& m)
{
  auto m_geometry = m.def_submodule("geometry");

  // SHAPES ====================================================================
  py::class_<geometry::FinalShape, PyFinalShape,
    std::shared_ptr<geometry::FinalShape>>(
    m_geometry, "FinalShape")
  .def(py::init<>())
  .def_property_readonly("source",
    &geometry::FinalShape::source,
    py::return_value_policy::reference_internal)
  .def_property_readonly("characteristic_length",
    &geometry::FinalShape::get_characteristic_length);

  // CONVEX SHAPES =============================================================
  py::class_<geometry::FinalConvexShape,
    geometry::FinalShape,
    PyFinalConvexShape,
    std::shared_ptr<geometry::FinalConvexShape>>(
    m_geometry, "FinalConvexShape")
  .def(py::init<>())
  .def_property_readonly("source",
    &geometry::FinalShape::source,
    py::return_value_policy::reference_internal)
  .def_property_readonly("characteristic_length",
    &geometry::FinalShape::get_characteristic_length);

  // CIRCLE ====================================================================
  py::class_<geometry::Circle>(m_geometry, "Circle")
  .def(py::init<double>(), py::arg("radius"))
  .def(py::init<const geometry::Circle&>(), py::arg("other"))
  .def_property("radius",
    &geometry::Circle::get_radius,
    &geometry::Circle::set_radius,
    "set or get radius")
  .def("finalize", &geometry::Circle::finalize)
  .def("finalize_convex", &geometry::Circle::finalize_convex);

  // SHAPE FACTORIES ===========================================================
  m_geometry.def("make_final_convex_circle",
    [](double radius)
    {
      return std::make_shared<geometry::FinalConvexShape>(
        geometry::Circle(std::forward<double>(radius)).finalize_convex());
    },
    py::arg("radius"));
}
