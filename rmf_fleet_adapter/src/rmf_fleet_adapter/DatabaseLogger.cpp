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
#include "agv/internal_FleetUpdateHandle.hpp"

#include <filesystem>
#include <fstream>

// TODO(YV) Remove
#include <iostream>

//==============================================================================
// int sqlite3_exec(
//   sqlite3*,                                  /* An open database */
//   const char *sql,                           /* SQL to be evaluated */
//   int (*callback)(void*,int,char**,char**),  /* Callback function */
//   void *,                                    /* 1st argument to callback */
//   char **errmsg                              /* Error msg written here */
// );
//==============================================================================

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

  // Create table for task logs
  // DISPATCH and DIRECT will be jsons containing deserialized assignment jsons
  sql = "CREATE TABLE IF NOT EXISTS TASK_QUEUES("
        "ROBOT TEXT PRIMARY KEY NOT NULL, "
        "DISPATCH TEXT, "
        "DIRECT TEXT);";
  error = sqlite3_exec(logger->_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error from creating TASK_QUEUES table: " << error_msg << std::endl;


  // TODO(YV): Table for bid notice assignments

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
  return _restored;
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
void DatabaseLogger::backup_task_queues(const TaskManager& mgr)
{
  // We can access private members of TaskManager as DatabaseLogger is a friend

  auto fleet_handle = mgr._fleet_handle.lock();
  if (!fleet_handle)
  {
    std::cout << "Unable to lock fleet handle inside backup_task_queues"
              << std::endl;
    return;
  }
  auto& impl =
    agv::FleetUpdateHandle::Implementation::get(*fleet_handle);
  const std::string& robot = mgr._context->name();
  const auto& request_jsons = impl.task_request_jsons;
  std::vector<std::string> errors;

  nlohmann::json queue;
  std::vector<nlohmann::json> assignments;
  for (const auto& assignment : mgr._queue)
  {
    assignments.push_back(convert(assignment, request_jsons));
  }
  queue = assignments;
  // Write to db
  {
  std::lock_guard<std::mutex> lock(_mutex);
  std::string data = queue.dump();
  std::string sql = "REPLACE INTO TASK_QUEUES (ROBOT,DISPATCH) "
    "VALUES('" + robot + "', '" + data + "');";
  char* error_msg;
  auto error = sqlite3_exec(_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error updating dispatch queue: " << error_msg << std::endl;
  }

  // Direct assignments
  queue = {};
  assignments.clear();
  for (const auto& assignment : mgr._direct_queue)
  {
    assignments.push_back(convert(assignment.assignment, request_jsons));
  }
  queue = assignments;
  // Write to db
  {
  std::lock_guard<std::mutex> lock(_mutex);
  std::string data = queue.dump();
  std::string sql = "REPLACE INTO TASK_QUEUES (ROBOT,DIRECT) "
    "VALUES('" + robot + "', '" + data + "');";
  char* error_msg;
  auto error = sqlite3_exec(_db, sql.c_str(), NULL, NULL, &error_msg);
  if (error)
    std::cout << "Error updating dispatch queue: " << error_msg << std::endl;
  }

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

namespace {
//==============================================================================
std::chrono::milliseconds to_millis(rmf_traffic::Duration duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
}

} // namespace anonymous

//==============================================================================
nlohmann::json DatabaseLogger::convert(const rmf_task::State& state) const
{
  // State schmea
  // {
  //   waypoint: integer,
  //   orientation: double,[optional]
  //   time: integer
  //   charger: integer
  //   battery_soc
  // }

  nlohmann::json msg;

  // To be safe we check if the optionals have value but they definitely should
  auto waypoint_opt = state.waypoint();
  if (waypoint_opt.has_value())
    msg["waypoint"] = waypoint_opt.value();
  auto orientation_opt = state.orientation();
  if (orientation_opt.has_value())
    msg["orientation"] = orientation_opt.value();
  auto time_opt = state.time();
  if (time_opt.has_value())
    msg["time"] = to_millis(time_opt.value().time_since_epoch()).count();
  auto charger_opt = state.dedicated_charging_waypoint();
  if (charger_opt.has_value())
    msg["charger"] = charger_opt.value();
  auto soc_opt = state.battery_soc();
  if (soc_opt.has_value())
    msg["battery_soc"] = soc_opt.value();

  return msg;
}

//==============================================================================
nlohmann::json DatabaseLogger::convert(
  const Assignment& assignment,
  const std::unordered_map<std::string, nlohmann::json> request_jsons) const
{

  // Assignment schema
  // {
  //   request: ref rmf_api_msgs::schemas::task_request,
  //   state: ref State schema,
  //   unix_millis_deployment_time: integer,
  // }

  nlohmann::json msg;
  const auto request = assignment.request();
  const auto& state = assignment.finish_state();
  const auto deployment_time = assignment.deployment_time();

  msg["state"] = convert(state);
  msg["unix_millis_deployment_time"] =
    to_millis(deployment_time.time_since_epoch()).count();

  const auto it = request_jsons.find(request->booking()->id());
  if (it != request_jsons.end())
  {
    msg["request"] = it->second;
  }
  else
  {
    // TODO(YV): Deserialize this automatic task into schemas::task_request
  }

  return msg;
}

//==============================================================================
auto DatabaseLogger::convert(const nlohmann::json& msg) const -> Assignment
{
  // msg follows the Assignment schema defined above

  // TODO(YV): Fix
  rmf_task::State state;
  // We can use FleetUpdateHandle::Implementation::convert(json_requst) to get
  // ConstRequestPtr
  Assignment assignment{nullptr, state, std::chrono::steady_clock::now()};
  return assignment;
}

} // namespace rmf_fleet_adapter
