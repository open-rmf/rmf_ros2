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

#ifndef PYROBOTCOMMANDHANDLE_HPP
#define PYROBOTCOMMANDHANDLE_HPP

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

// Trampoline RobotCommandHandle wrapper class
// to allow method overrides from Python
class PyRobotCommandHandle :
  public rmf_fleet_adapter::agv::RobotCommandHandle
{
public:
  // Constructor
  using rmf_fleet_adapter::agv::RobotCommandHandle::RobotCommandHandle;

  using ArrivalEstimator =
    std::function<void(std::size_t path_index,
      rmf_traffic::Duration remaining_time)>;

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    std::function<void()> path_finished_callback) override
  {
    PYBIND11_OVERLOAD_PURE(
      void,
      rmf_fleet_adapter::agv::RobotCommandHandle,
      follow_new_path,
      waypoints,
      next_arrival_estimator,
      path_finished_callback
    );
  }

  void stop() override
  {
    PYBIND11_OVERLOAD_PURE(
      void,
      rmf_fleet_adapter::agv::RobotCommandHandle,
      stop,  // Trailing comma required
    );
  }

  void dock(
    const std::string& dock_name,
    std::function<void()> docking_finished_callback) override
  {
    PYBIND11_OVERLOAD_PURE(
      void,
      rmf_fleet_adapter::agv::RobotCommandHandle,
      dock,
      dock_name,
      docking_finished_callback
    );
  }
};

#endif // PYROBOTCOMMANDHANDLE_HPP
