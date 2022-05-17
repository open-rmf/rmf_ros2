#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <memory>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_utils/clone_ptr.hpp>

namespace py = pybind11;
namespace agv = rmf_fleet_adapter::agv;

void bind_tests(py::module& m)
{
  // Simply use something like f.test_lambda(lambda: print("wow")) in Python
  m.def("test_lambda",
    [](std::function<void()> f) { f(); },
    "Call a C++ lambda function created in Python.");

  // Test shared_ptr passing
  m.def("test_shared_ptr",
    [](std::shared_ptr<agv::RobotCommandHandle> handle,
    std::string print_str = "DUMMY_DOCK_NAME",
    std::function<void()> docking_finished_callback = []() {})
    {
      handle->dock(print_str, docking_finished_callback);
    },
    py::arg("handle"),
    py::arg("print_str") = "DUMMY_DOCK_NAME",
    py::arg("docking_finished_callback") = (std::function<void()>)[] (){},
    "Test a shared_ptr<agv::RobotCommandHandle> "
    "by testing its dock() method"
  );

  // Test clone_ptr passing
  // TODO(CH3): clone_ptrs do not work at the moment
  // Something about clone_ptr indirection is breaking
  m.def("test_clone_ptr",
    [](rmf_utils::clone_ptr<
      rmf_traffic::agv::Graph::OrientationConstraint> constraint_ptr,
    Eigen::Vector3d& position,
    const Eigen::Vector2d& course_vector)
    {
      return constraint_ptr->apply(position, course_vector);
    },
    "Test a clone_ptr<rmf_traffic::agv::Graph::OrientationConstraint> "
    "by testing its get() method",
    py::return_value_policy::reference_internal
  );
}
