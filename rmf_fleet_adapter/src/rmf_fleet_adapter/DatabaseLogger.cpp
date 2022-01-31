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

  auto table_exists =
    [logger](const std::string& name) -> bool
    {
      char* error_msg;
      const std::string query = "SELECT name FROM sqlite_master "
      "WHERE type='table' AND name='" + name +"';";
      auto error = sqlite3_exec(logger->_db, query.c_str(), NULL, 0, &error_msg);
      // TOOD(YV): Distinguish between error code from bad statement and table
      // not existing.
      return !error;
    };

  // Create tables if they do not already exist
  // Active Task Table
  if (!table_exists("ACTIVE_TASK"))
  {
    sql = "CREATE TABLE ACTIVE_TASK("
          "ROBOT TEXT PRIMARY KEY NOT NULL, "
          "STATE TEXT NOT NULL);";
    error = sqlite3_exec(logger->_db, sql.c_str(), NULL, 0, &error_msg);
    assert(!error);
  }

  // TODO(YV): Tables for task queues, task logs and bid notice assignments

  // TODO(YV): Also store history of task & state logs along with latest state.
  // Proposed key (robot, task, phase, event, seq)


  // TODO(YV): If db is not empty, generate Restore. For now we will set it to
  // nullopt.

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

  const std::string sql = "IF EXISTS(SELECT * FROM ACTIVE_TASK WHERE ROBOT=" +
  robot +")" + "UPDATE ACTIVE_TASK SET STATE=" + state + "WHERE ROBOT=" + robot +
  "ELSE INSERT INTO ACTIVE_TASK(" + robot +") values(" + state + ");";
  char* error_msg;

  auto print_table =
    [](void* data, int num_col, char** col_values, char** col_names)
    {
      int i;
      fprintf(stderr, "%s: ", (const char*)data);

      for (i = 0; i < num_col; i++) {
          printf("%s = %s\n", col_names[i], col_values[i] ? col_values[i] : "NULL");
      }

      printf("\n");
      return 0;
    };
  auto error = sqlite3_exec(_db, sql.c_str(), print_table, 0, &error_msg);

}

//==============================================================================
void DatabaseLogger::backup_task_logs(
    const std::string& robot,
    const nlohmann::json& task_logs)
{
  // TODO
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
