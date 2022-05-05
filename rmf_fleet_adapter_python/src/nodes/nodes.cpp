/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/complex.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_traffic_ros2/blockade/Node.hpp>
#include <rmf_traffic_ros2/schedule/Node.hpp>

namespace py = pybind11;

/// Create a ros2 node of different major node components in RMF
/// TODO (YL): Might move these nodes static functions to a different repo as
/// these are not a part of adapter
//==============================================================================
void bind_nodes(py::module& m)
{
  auto m_nodes = m.def_submodule("nodes");

  // Make blockade_Node
  m_nodes.def("make_blockade",
    py::overload_cast<const rclcpp::NodeOptions&>(
      &rmf_traffic_ros2::blockade::make_node),
    py::arg("options"),
    "make rmf ros2 blockade node");

  // Make Schedule Node
  m_nodes.def("make_schedule", &rmf_traffic_ros2::schedule::make_node,
    py::arg("options"));
}
