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

#ifndef _PYPYROBOTCOMMANDHANDLE_H
#define _PYPYROBOTCOMMANDHANDLE_H

#include <pybind11/pybind11.h>
#include "PyRobotCommandHandle.hpp"

// Trampoline class to support Python overrides of the default
// implementations of PyRobotCommandHandle methods
class _PyRobotCommandHandle :
  public PyRobotCommandHandle
{
  // Inherit constructor
  using PyRobotCommandHandle::PyRobotCommandHandle;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    std::function<void()> path_finished_callback) override
  {
    PYBIND11_OVERLOAD(
      void,
      PyRobotCommandHandle,
      follow_new_path,
      waypoints,
      path_finished_callback
    );
  }

  void dock(
    const std::string& dock_name,
    std::function<void()> docking_finished_callback) override
  {
    PYBIND11_OVERLOAD(
      void,
      PyRobotCommandHandle,
      dock,
      dock_name,
      docking_finished_callback
    );
  }

  std::shared_ptr<PyRobotCommandHandle> get_ptr()
  {
    return shared_from_this();
  }
};

#endif // _PYPYROBOTCOMMANDHANDLE_H
