#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/iostream.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_traffic_ros2/blockade/Node.hpp>
#include <rmf_traffic_ros2/schedule/Node.hpp>

namespace py = pybind11;

// TODO (YL) Might place this at a different repo as this is not a part of adapter
void bind_nodes(py::module &m) {
  auto m_nodes = m.def_submodule("nodes");

  // Make blockade_Node
  m_nodes.def("make_blockade", &rmf_traffic_ros2::blockade::make_node,
              py::arg("options"));

  // Make Schedule Node
  m_nodes.def("make_schedule", &rmf_traffic_ros2::schedule::make_node,
              py::arg("options"));
}
