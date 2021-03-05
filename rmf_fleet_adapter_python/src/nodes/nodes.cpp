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
#include <pybind11/iostream.h>
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
#include <rmf_task_ros2/Dispatcher.hpp>

namespace py = pybind11;
using Dispatcher = rmf_task_ros2::Dispatcher;
using TaskStatus = rmf_task_ros2::TaskStatus;

/// Create a ros2 node of different major node components in RMF
/// TODO (YL): Might move these nodes static functions to a different repo as
/// these are not a part of adapter
//==============================================================================
void bind_nodes(py::module &m)
{
  auto m_nodes = m.def_submodule("nodes");

  // Make blockade_Node
  m_nodes.def("make_blockade", &rmf_traffic_ros2::blockade::make_node,
              py::arg("options"),
              "make rmf ros2 blockade node");

  // Make Schedule Node
  m_nodes.def("make_schedule", &rmf_traffic_ros2::schedule::make_node,
              py::arg("options"));

  /// Dispatcher Node Class
  /// \brief Create a simple dispatcher node api mainly for testing
  py::class_<Dispatcher, std::shared_ptr<Dispatcher>>(
      m_nodes, "DispatcherNode")
      .def_static("make_node", &Dispatcher::make_node,
                  // py::arg("dispatcher_node_name"), TODO Remove in next version
                  py::call_guard<py::scoped_ostream_redirect,
                                 py::scoped_estream_redirect>())
      // .def("submit_task",
      //      py::overload_cast<const Dispatcher::TaskDescription&>(
      //       &Dispatcher::submit_task),
      //      py::arg("task_description"))
      .def("cancel_task", &Dispatcher::cancel_task,
           py::arg("task_id"))
      .def("spin", &Dispatcher::spin,
           "This is a blocking spin")
      .def("node", &Dispatcher::node)
      .def("get_active_task_ids", [](Dispatcher &self) {
        std::vector<std::string> task_ids;
        for (auto task : self.active_tasks())
        {
          task_ids.push_back(task.first);
        }
        return task_ids;
      })
      .def("get_terminated_task_ids", [](Dispatcher &self) {
        std::vector<std::string> task_ids;
        for (auto task : self.terminated_tasks())
        {
          task_ids.push_back(task.first);
        }
        return task_ids;
      })
      .def("get_task_state", &Dispatcher::get_task_state, py::arg("task_id"));

  py::enum_<TaskStatus::State>(
      m_nodes, "TaskState")
      .value("Queued", TaskStatus::State::Queued)
      .value("Executing", TaskStatus::State::Executing)
      .value("Completed", TaskStatus::State::Completed)
      .value("Failed", TaskStatus::State::Failed)
      .value("Canceled", TaskStatus::State::Canceled)
      .value("Pending", TaskStatus::State::Pending);
}
