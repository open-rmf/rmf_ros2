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

  auto error = sqlite3_open(file_path, *logger->_db)
  if (error)
    return nullptr;

  // Currently we only store the state of the fleet adapter
  // for restoration.
  // Table TaskManagers: Robot name as key. Columns for active state json & queues



  // TODO(YV): Also store history of task & state logs along with latest state.
  // Proposed key (robot, task, phase, event, seq)

  return logger;
}

//==============================================================================
DatabaseLogger::DatabaseLogger()
{
  // Do nothing
}

//==============================================================================
DatabaseLogger::DatabaseLogger()
{
  // Safely destruct the sqlite3 object
  sqlite3_close(_db);
}
//==============================================================================
void DatabaseLogger::backup(const BidNoticeAssignments& assignments)
{

}

//==============================================================================
void DatabaseLogger::backup(const TaskManager& mgr)
{

}

//==============================================================================
nlohmann::json DatabaseLogger::_convert(const Assignment& assignment)
{
  nlohmann::json msg;
  return msg;
}

//==============================================================================
auto DatabaseLogger::_convert(const nlohmann::json& msg) -> Assignment
{
  rmf_task::State state;
  Assignment assignment{nullptr, state, rmf_traffic::Time(0)};
  return assignment;
}

} // namespace rmf_fleet_adapter
