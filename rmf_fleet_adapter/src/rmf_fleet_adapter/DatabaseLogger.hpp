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
#include "agv/internal_FleetUpdateHandle.hpp"

#include <sqlite3.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <mutex>

namespace rmf_fleet_adapter {
//==============================================================================
// A wrapper around a sqlite3 database.
// TODO(YV): The Logger can inherit from an abstract class so that users can
// use whatever DBS they want.
class DatabaseLogger
{
public:
  using Assignment = rmf_task::TaskPlanner::Assignment;
  using Assignments = rmf_task::TaskPlanner::Assignments;
  using BidNoticeAssignments =
    std::unordered_map<std::string, Assignments>;
  // Bundle up the restored state of the fleet adapter
  // TODO(YV): Ensure index of managers correspond to index of Assignments
  // We should return some sort of map<robot_name, manager> which
  // FleetUpdateHandle::add_robot() can lookup for restored task managers.
  // But if a new robot is being added, what do we do with the
  // previous bid_notice_assignments? The index order will be incorrect.
  // We would need to modify the implementation of dispatch_command_cb to not
  // return an error when the number of robots do not match the number in
  // Assignments.
  struct Restored
  {
    std::vector<TaskManager> managers;
    BidNoticeAssignments bid_notice_assignments;
  };

  static std::shared_ptr<DatabaseLogger> make(
    const std::string& file_path);

  // Returns nullopt if file_path did not exist previously
  std::optional<Restored> restore() const;

  // TODO(YV): Consider having internal_FleetUpdateHandle and TaskManager
  // receive these functions as callbacks to trigger on updates
  // Backup bid_notice_assignments
  void backup(const BidNoticeAssignments& assignments);

  // Backup active task state along with task queues
  void backup(const TaskManager& mgr);

  // Do not allow copying or moving
  // DatabaseLogger(const DatabaseLogger&) = delete;
  // DatabaseLogger& operator=(const DatabaseLogger&) = delete;
  // DatabaseLogger(DatabaseLogger&&) = delete;
  // DatabaseLogger& operator=(DatabaseLogger&&) = delete;

  ~DatabaseLogger();

private:
  DatabaseLogger();

  // Helper functions to serialize/deserialize assignments
  // TODO(YV): Consider formalizing the schema in rmf_fleet_adapter/schemas
  nlohmann::json _convert(const Assignment& assignment);
  Assignment _convert(const nlohmann::json& msg);

  std::string _file_path;
  sqlite3* _db;
  std::mutex _mutex;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__DATABASELOGGER_HPP
