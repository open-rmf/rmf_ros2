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

#ifndef _PYSHAREDROBOTCOMMANDHANDLE_H
#define _PYSHAREDROBOTCOMMANDHANDLE_H

#include <pybind11/pybind11.h>
#include "SharedRobotCommandHandle.hpp"

// Inheritance Tree: RobotCommandHandle -> SharedRobotCommandHandle

// Trampoline class to support Python overrides of the default
// implementations of PyRobotCommandHandle methods
class _PySharedRobotCommandHandle : public SharedRobotCommandHandle
{
  // Inherit constructor
  using SharedRobotCommandHandle::SharedRobotCommandHandle;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    std::function<void()> path_finished_callback) override
  {
    PYBIND11_OVERLOAD_PURE(
      void,
      SharedRobotCommandHandle,
      follow_new_path,
      waypoints,
      path_finished_callback
    );
  }

  void dock(
    const std::string& dock_name,
    std::function<void()> docking_finished_callback) override
  {
    PYBIND11_OVERLOAD_PURE(
      void,
      SharedRobotCommandHandle,
      dock,
      dock_name,
      docking_finished_callback
    );
  }

  // std::shared_ptr<SharedRobotCommandHandle> get_ptr()
  // {
  //   PYBIND11_OVERLOAD_PURE(
  //     std::shared_ptr<SharedRobotCommandHandle>,
  //     SharedRobotCommandHandle,  // Trailing comma required
  //   );
  // }
};

#endif // _PYSHAREDROBOTCOMMANDHANDLE_H
