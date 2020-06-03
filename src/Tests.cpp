#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/functional.h>
#include <memory>

#include <rmf_mock_adapter/adapter.hpp>

namespace py = pybind11;

void bind_tests(py::module &m) {
  // Simply use something like f.test_lambda(lambda: print("wow")) in Python
  m.def("test_lambda",
        [](std::function<void()> f){ f(); },
        "Call a C++ lambda function created in Python.");

  // Test shared_ptr passing
  m.def("test_shared_ptr",
        [](std::shared_ptr<rmf_mock_adapter::RobotCommandHandle> handle,
           std::string print_str = "DUMMY_DOCK_NAME",
           std::function<void()> docking_finished_callback = [&](){})
        {
          handle->dock(print_str, docking_finished_callback);
        },
        py::arg("handle"),
        py::arg("print_str") = "DUMMY_DOCK_NAME",
        py::arg("docking_finished_callback") = (std::function<void()>) [](){},
        "Test a shared_ptr<rmf_mock_adapter::RobotCommandHandle "
        "by testing its dock() method",
        py::call_guard<py::scoped_ostream_redirect,
                       py::scoped_estream_redirect>()
      );
}
