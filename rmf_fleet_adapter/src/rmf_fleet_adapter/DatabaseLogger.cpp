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

#include "DatabaseLogger.hpp"

#include <filesystem>
#include <fstream>

// TODO(YV) Remove
#include <iostream>

namespace rmf_fleet_adapter {
//==============================================================================
auto DatabaseLogger::make(const std::string& file_path) ->
std::shared_ptr<DatabaseLogger>
{
  std::shared_ptr<DatabaseLogger> logger(new DatabaseLogger());
  logger->_file_path = file_path;

  if (!std::filesystem::exists(file_path))
  {
    std::filesystem::create_directories(
      std::filesystem::absolute(file_path).parent_path());
  }

  // Variables that will be reassigned
  int error;
  std::string sql;
  char* error_msg;


  error = sqlite3_open(file_path.c_str(), &logger->_db);
  if (error)
    return nullptr;

  // Create table for active task states
  sql = "CREATE TABLE IF NOT EXISTS ACTIVE_TASK("
        "ROBOT TEXT PRIMARY KEY NOT NULL, "
        "STATE TEXT NOT NULL);";
  error = sqlite3_exec(logger->_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error from creating ACTIVE_TASK table: " << error_msg << std::endl;

  // Create table for task logs
  sql = "CREATE TABLE IF NOT EXISTS TASK_LOGS("
        "ID TEXT PRIMARY KEY NOT NULL, "
        "LOGS TEXT NOT NULL);";
  error = sqlite3_exec(logger->_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error from creating TASK_LOGS table: " << error_msg << std::endl;

  // TODO(YV): Tables for task queues, task logs and bid notice assignments

  // TODO(YV): Also store history of task & state logs along with latest state.
  // Proposed key (robot, task, phase, event, seq)


  // TODO(YV): If db is not empty, generate Restore. For now we will set it to
  // nullopt.

  std::cout << "Created database" << std::endl;

  return logger;
}

//==============================================================================
DatabaseLogger::DatabaseLogger()
{
  // Do nothing
}

//==============================================================================
auto DatabaseLogger::restore() const -> std::optional<Restored>
{
  return std::nullopt;
}

//==============================================================================
DatabaseLogger::~DatabaseLogger()
{
  // Safely destruct the sqlite3 object
  sqlite3_close(_db);
}

//==============================================================================
void DatabaseLogger::backup_bid_notice_assignments(
  const BidNoticeAssignments& assignments)
{
  // TODO
}

//==============================================================================
void DatabaseLogger::backup_task_queues(TaskManager& mgr)
{
  // TODO
}

//==============================================================================
void DatabaseLogger::backup_active_task(
  const std::string& robot,
  const nlohmann::json& task_state)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const std::string state = task_state.dump();

  const std::string sql = "REPLACE INTO ACTIVE_TASK (ROBOT,STATE) "
    "VALUES('" + robot + "', '" + state + "');";

  char* error_msg;
  auto error = sqlite3_exec(_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error updating active task: " << error_msg << std::endl;
}

//==============================================================================
void DatabaseLogger::backup_task_logs(
    const std::string& robot,
    const nlohmann::json& task_logs)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto it = task_logs.find("task_id");
  if (it == task_logs.end())
    return;
  const auto& task_id = it.value();
  if (!task_id.is_string())
    return;

  const std::string id = "(" + robot + "," + task_id.get<std::string>() + ")";
  const std::string logs = task_logs.dump();

  const std::string sql = "REPLACE INTO TASK_LOGS (ID,LOGS) "
    "VALUES('" + id + "', '" + logs + "');";

  char* error_msg;
  auto error = sqlite3_exec(_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error updating active task: " << error_msg << std::endl;
}

//==============================================================================
nlohmann::json DatabaseLogger::convert(const Assignment& assignment)
{
  nlohmann::json msg;
  return msg;
}

//==============================================================================
auto DatabaseLogger::convert(const nlohmann::json& msg) -> Assignment
{
  rmf_task::State state;
  Assignment assignment{nullptr, state, std::chrono::steady_clock::now()};
  return assignment;
}

} // namespace rmf_fleet_adapter
