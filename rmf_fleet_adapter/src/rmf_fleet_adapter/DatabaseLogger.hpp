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
  // TODO(YV): FleetUpdateHandle::add_robot() can lookup for restored task
  // managers. But if a new robot is being added, what do we do with the
  // previous bid_notice_assignments? The index order will be incorrect.
  // We would need to modify the implementation of dispatch_command_cb to not
  // return an error when the number of robots do not match the number in
  // Assignments.
  struct Restored
  {
    std::unordered_map<std::string, std::shared_ptr<TaskManager>> managers;
    BidNoticeAssignments bid_notice_assignments;
  };

  // TODO(YV): Should we accept a bool to delete/drop all tables?
  static std::shared_ptr<DatabaseLogger> make(
    const std::string& file_path);

  // Returns nullopt if file_path did not exist previously.
  // Does this function require a weak_ptr<FleetUpdateHandle> to generating
  // requests/deserialization/validation?
  std::optional<Restored> restore() const;

  void backup_bid_notice_assignments(const BidNoticeAssignments& assignments);

  void backup_task_queues(const TaskManager& mgr);

  // Key used: robot
  void backup_active_task(
    const std::string& robot,
    const nlohmann::json& task_state);

  // Key used: (task_id)
  void backup_task_logs(
    const std::string& task_id,
    const nlohmann::json& task_logs);

  ~DatabaseLogger();

private:
  DatabaseLogger();

  // Helper functions to serialize/deserialize assignments
  // TODO(YV): Consider formalizing the schema in rmf_fleet_adapter/schemas
  nlohmann::json convert(const rmf_task::State& state) const;

  nlohmann::json convert(
    const Assignment& assignment,
    const std::unordered_map<std::string, nlohmann::json> request_jsons) const;

  Assignment convert(const nlohmann::json& msg) const;

  // TODO(YV): Explore using an open-source library to build sql statements

  std::string _file_path;
  sqlite3* _db;
  std::mutex _mutex;
  std::optional<Restored> _restored = std::nullopt;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__DATABASELOGGER_HPP
