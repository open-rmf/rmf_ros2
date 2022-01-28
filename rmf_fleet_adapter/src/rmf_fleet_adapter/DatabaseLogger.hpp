/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__DATABASELOGGER_HPP
#define SRC__RMF_FLEET_ADAPTER__DATABASELOGGER_HPP

#include <rmf_task/TaskPlanner.hpp>

#include "TaskManager.hpp"

#include <sqlite3.h>

#include <memory>

namespace rmf_fleet_adapter {
//==============================================================================
// A wrapper around a sqlite3 database.
// TODO(YV): The Logger can inherit from an abstract class so that users can
// use whatever DBS they want.
class DatabaseLogger
{
public:
  using Assignments = rmf_task::TaskPlanner::Assignments;
  // Bundle up the restored state of the fleet adapter
  // TODO(YV): Ensure index of managers correspond to index of Assignments
  struct Restored
  {
    std::vector<TaskManager> managers;
    std::unordered_map<std::string, Assignments> bid_notice_assignments
  };

  static std::shared_ptr<DatabaseLogger> make(
    const std::string& filename);



  ~DatabaseLogger();

private:
  DatabaseLogger();
  std::shared_ptr<sqlite3> db;

};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__DATABASELOGGER_HPP
