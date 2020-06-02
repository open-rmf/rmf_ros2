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

#ifndef PYROBOTCOMMANDHANDLEINSTANCE_H
#define PYROBOTCOMMANDHANDLEINSTANCE_H

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include <rmf_mock_adapter/adapter.hpp>

// Intermediate RobotCommandHandle wrapper to implement shared_pointer getter
class PyRobotCommandHandle :
  public rmf_mock_adapter::RobotCommandHandle,
  public std::enable_shared_from_this<PyRobotCommandHandle>
{
public:
  // Constructor
  PyRobotCommandHandle();

  // Methods
  virtual void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      std::function<void()> path_finished_callback);
  virtual void dock(const std::string& dock_name,
                    std::function<void()> docking_finished_callback);
  virtual void step();
  virtual bool interruped();

  std::shared_ptr<PyRobotCommandHandle> get_ptr()
  {
      return shared_from_this();
  }
};

#endif // PYROBOTCOMMANDHANDLEINSTANCE_H
