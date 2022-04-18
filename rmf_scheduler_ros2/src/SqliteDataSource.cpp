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

#include "SqliteDataSource.hpp"

#include <sqlite3.h>

#include <chrono>
#include <stdexcept>

namespace rmf::scheduler {

//====

SqliteDataSource::DatabaseError::DatabaseError(_SqlitePtr db)
: DatabaseError(sqlite3_errcode(db.get()), sqlite3_errmsg(db.get()))
{
}

SqliteDataSource::DatabaseError::DatabaseError(int code, const char* errmsg)
: code(code), errmsg(errmsg)
{
}

const char* SqliteDataSource::DatabaseError::what() const noexcept
{
  return this->errmsg.c_str();
}

//====

SqliteDataSource::_Transaction::_Transaction(SqliteDataSource* store)
: _store(store), _db(store->_db)
{
}

void SqliteDataSource::_Transaction::create_schedule(
  const rmf_scheduler_msgs::msg::Schedule& schedule,
  const rmf_scheduler_msgs::msg::ScheduleState& state)
{
  std::string sql =
    R"(
INSERT OR REPLACE INTO Schedule (
  name,
  created_at,
  schedule,
  start_at,
  finish_at,
  "group",
  payload_type,
  payload_data,
  last_modified,
  last_ran,
  next_run,
  status
) VALUES (
  ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
)
  )";
  sqlite3_stmt* stmt;
  this->_store->_prepare_stmt(&stmt, sql,
    schedule.name,
    schedule.created_at,
    schedule.schedule,
    schedule.start_at,
    schedule.finish_at,
    schedule.group,
    schedule.payload.type,
    schedule.payload.data,
    state.last_modified,
    state.last_ran,
    state.next_run,
    state.status
  );

  if (sqlite3_step(stmt) != SQLITE_DONE)
  {
    throw DatabaseError(this->_db);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

void SqliteDataSource::_Transaction::save_schedule_state(
  const rmf_scheduler_msgs::msg::ScheduleState& state)
{
  std::string sql =
    R"(
UPDATE Schedule SET
  last_modified = ?,
  last_ran = ?,
  next_run = ?,
  status = ?
WHERE name = ?
  )";
  sqlite3_stmt* stmt;
  this->_store->_prepare_stmt(&stmt, sql,
    state.last_modified,
    state.last_ran,
    state.next_run,
    state.status,
    state.name
  );

  if (sqlite3_step(stmt) != SQLITE_DONE)
  {
    throw DatabaseError(this->_db);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

void SqliteDataSource::_Transaction::create_trigger(
  const rmf_scheduler_msgs::msg::Trigger& trigger,
  const rmf_scheduler_msgs::msg::TriggerState& state)
{
  std::string sql =
    R"(
INSERT OR REPLACE INTO Trigger (
  name, created_at, last_modified, at, "group", payload_type,
  payload_data, last_ran, status
) VALUES (
  ?, ?, ?, ?, ?, ?, ?, ?, ?
)
)";
  sqlite3_stmt* stmt;
  this->_store->_prepare_stmt(&stmt, sql, trigger.name, trigger.created_at,
    state.last_modified,
    trigger.at, trigger.group, trigger.payload.type,
    trigger.payload.data, state.last_ran,
    state.status);

  if (sqlite3_step(stmt) != SQLITE_DONE)
  {
    throw DatabaseError(this->_db);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

void SqliteDataSource::_Transaction::save_trigger_state(
  const rmf_scheduler_msgs::msg::TriggerState& state)
{
  std::string sql =
    R"(
UPDATE Trigger SET
  last_modified = ?,
  last_ran = ?,
  status = ?
WHERE name = ?
  )";
  sqlite3_stmt* stmt;
  this->_store->_prepare_stmt(&stmt, sql, state.last_modified, state.last_ran,
    state.status,
    state.name);

  if (sqlite3_step(stmt) != SQLITE_DONE)
  {
    throw DatabaseError(this->_db);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

//===

SqliteDataSource::SqliteDataSource(const std::string& file)
{
  {
    sqlite3* tmp;
    sqlite3_open(file.c_str(), &tmp);
    this->_db.reset(tmp, [](sqlite3* db) { sqlite3_close(db); });
  }
  char* errmsg;

  int result = sqlite3_exec(
    this->_db.get(), R"(
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS Trigger (
  name TEXT PRIMARY KEY,
  created_at INTEGER NOT NULL,
  last_modified INTEGER NOT NULL,
  at INTEGER NOT NULL,
  "group" TEXT NOT NULL,
  payload_type INTEGER NOT NULL,
  payload_data BLOB NOT NULL,
  last_ran INTEGER NOT NULL,
  status INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS Trigger_index ON Trigger (
  created_at, last_modified, status
);

CREATE TABLE IF NOT EXISTS Schedule (
  name TEXT PRIMARY KEY,
  created_at INTEGER NOT NULL,
  schedule TEXT NOT NULL,
  start_at INTEGER NOT NULL,
  finish_at INTEGER NOT NULL,
  "group" TEXT NOT NULL,
  payload_type INTEGER NOT NULL,
  payload_data BLOB NOT NULL,
  last_modified INTEGER NOT NULL,
  last_ran INTEGER NOT NULL,
  next_run INTEGER NOT NULL,
  status INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS Schedule_index ON Schedule (
  created_at, last_modified, status
);
)", nullptr, nullptr,
    &errmsg);
  if (result != SQLITE_OK)
  {
    throw DatabaseError(result, errmsg);
  }
}

SqliteDataSource::_Transaction SqliteDataSource::begin_transaction()
{
  char* errmsg;
  int result = sqlite3_exec(
    this->_db.get(), "BEGIN TRANSACTION", nullptr, nullptr,
    &errmsg);
  if (result != SQLITE_OK)
  {
    throw DatabaseError(result, errmsg);
  }

  return _Transaction(this);
}

void SqliteDataSource::commit_transaction()
{
  char* errmsg;
  int result = sqlite3_exec(
    this->_db.get(), "COMMIT TRANSACTION", nullptr, nullptr,
    &errmsg);
  if (result != SQLITE_OK)
  {
    throw DatabaseError(result, errmsg);
  }
}

std::optional<rmf_scheduler_msgs::msg::Schedule>
SqliteDataSource::fetch_schedule(const std::string& name)
{
  auto schedules = this->_fetch_schedules("WHERE name = ?", name);
  if (schedules.empty())
  {
    return std::nullopt;
  }
  return schedules.front();
}

std::vector<rmf_scheduler_msgs::msg::Schedule>
SqliteDataSource::fetch_active_schedules()
{
  return this->_fetch_schedules("WHERE status = ? OR status = ?",
      rmf_scheduler_msgs::msg::ScheduleState::STARTED,
      rmf_scheduler_msgs::msg::ScheduleState::CREATED);
}

std::vector<std::string>
SqliteDataSource::fetch_schedules_in_group(const std::string& group)
{
  std::string sql = R"(SELECT name FROM Schedule WHERE "group" = ?)";
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, group);

  std::vector<std::string> schedules;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    if (result != SQLITE_ROW)
    {
      throw DatabaseError(this->_db);
    }

    schedules.emplace_back(
      reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
  }

  return schedules;
}

std::optional<rmf_scheduler_msgs::msg::ScheduleState>
SqliteDataSource::fetch_schedule_state(
  const std::string& name)
{
  auto states = this->_fetch_schedule_states("WHERE name = ?", name);
  if (states.empty())
  {
    return std::nullopt;
  }
  return states.front();
}

std::vector<rmf_scheduler_msgs::msg::Schedule>
SqliteDataSource::fetch_schedules_created_after(
  int64_t created_after)
{
  return this->_fetch_schedules(
    "WHERE created_at > ? ORDER BY created_at ASC", created_after);
}

std::vector<rmf_scheduler_msgs::msg::ScheduleState>
SqliteDataSource::fetch_schedule_states_modified_after(
  int64_t modified_after)
{
  return this->_fetch_schedule_states(
    "WHERE last_modified > ? ORDER BY last_modified ASC", modified_after);
}

std::optional<rmf_scheduler_msgs::msg::Trigger> SqliteDataSource::fetch_trigger(
  const std::string& name)
{
  auto triggers = this->_fetch_triggers("WHERE name = ?", name);
  if (triggers.empty())
  {
    return std::nullopt;
  }
  return triggers.front();
}

std::vector<rmf_scheduler_msgs::msg::Trigger>
SqliteDataSource::fetch_triggers_created_after(
  int64_t created_after)
{
  return this->_fetch_triggers("WHERE created_at > ? ORDER BY created_at ASC",
      created_after);
}

std::vector<rmf_scheduler_msgs::msg::TriggerState>
SqliteDataSource::fetch_trigger_states_modified_after(
  int64_t modified_after)
{
  return this->_fetch_trigger_states(
    "WHERE last_modified > ? ORDER BY last_modified ASC", modified_after);
}

std::vector<rmf_scheduler_msgs::msg::Trigger>
SqliteDataSource::fetch_active_triggers()
{
  return this->_fetch_triggers("WHERE status = ?",
      rmf_scheduler_msgs::msg::TriggerState::STARTED);
}

std::vector<std::string>
SqliteDataSource::fetch_triggers_in_group(const std::string& group)
{
  std::string sql = R"(SELECT name FROM Trigger WHERE "group" = ?)";
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, group);

  std::vector<std::string> triggers;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    if (result != SQLITE_ROW)
    {
      throw DatabaseError(this->_db);
    }

    triggers.emplace_back(
      reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
  }

  return triggers;
}

std::optional<rmf_scheduler_msgs::msg::TriggerState>
SqliteDataSource::fetch_trigger_state(const std::string& name)
{
  auto states = this->_fetch_trigger_states("WHERE name = ?", name);
  if (states.empty())
  {
    return std::nullopt;
  }
  return states.front();
}

template<typename T, std::enable_if_t<std::is_integral_v<T>, bool>>
void SqliteDataSource::_bind_arg(sqlite3_stmt* stmt, int i, T arg)
{
  if constexpr (sizeof(T) <= 4)
  {
    if (sqlite3_bind_int(stmt, i, arg) != SQLITE_OK)
    {
      throw DatabaseError(this->_db);
    }
  }
  else
  {
    if (sqlite3_bind_int64(stmt, i, arg) != SQLITE_OK)
    {
      throw DatabaseError(this->_db);
    }
  }
}

template<size_t I>
void SqliteDataSource::_bind_arg(sqlite3_stmt* stmt, int i,
  const rosidl_runtime_cpp::BoundedVector<uint8_t, I>& arg)
{
  if (sqlite3_bind_blob(stmt, i, arg.template data<const uint8_t>(), arg.size(),
    SQLITE_STATIC) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

void SqliteDataSource::_bind_arg(sqlite3_stmt* stmt, int i,
  const std::string& arg)
{
  if (sqlite3_bind_text(stmt, i, arg.c_str(), arg.size(),
    SQLITE_STATIC) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }
}

template<typename... Args>
void SqliteDataSource::_prepare_stmt(sqlite3_stmt** stmt,
  std::string_view sql, Args&& ... args)
{
  if (sqlite3_prepare_v2(this->_db.get(), sql.data(), sql.size() + 1, stmt,
    nullptr) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  int i = 1;
  (this->_bind_arg(*stmt, i++, std::forward<Args>(args)), ...);
}

template<typename... Args>
std::vector<rmf_scheduler_msgs::msg::Trigger>
SqliteDataSource::_fetch_triggers(const std::string& where, Args&& ... args)
{
  std::string sql =
    R"(SELECT name, created_at, at, "group", payload_type, payload_data FROM Trigger )"
    + where;
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, std::forward<Args>(args)...);

  std::vector<rmf_scheduler_msgs::msg::Trigger> triggers;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    if (result != SQLITE_ROW)
    {
      throw DatabaseError(this->_db);
    }

    auto& trigger = triggers.emplace_back();
    trigger.name = (const char*) sqlite3_column_text(stmt, 0);
    trigger.created_at = sqlite3_column_int64(stmt, 1);
    trigger.at = sqlite3_column_int64(stmt, 2);
    trigger.group = (const char*) sqlite3_column_text(stmt, 3);
    trigger.payload.type = sqlite3_column_int(stmt, 4);
    auto blob = (uint8_t*) sqlite3_column_blob(stmt, 5);
    auto payload_size = sqlite3_column_bytes(stmt, 5);
    using PayloadData = decltype(trigger.payload.data);
    trigger.payload.data = PayloadData{blob, blob + payload_size};
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return triggers;
}

template<typename... Args>
std::vector<rmf_scheduler_msgs::msg::TriggerState>
SqliteDataSource::_fetch_trigger_states(const std::string& where,
  Args&& ... args)
{
  std::string sql =
    "SELECT name, last_modified, last_ran, status FROM Trigger " + where;
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, std::forward<Args>(args)...);

  std::vector<rmf_scheduler_msgs::msg::TriggerState> states;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    auto& state = states.emplace_back();
    state.name = (const char*) sqlite3_column_text(stmt, 0);
    state.last_modified = sqlite3_column_int64(stmt, 1);
    state.last_ran = sqlite3_column_int64(stmt, 2);
    state.status = sqlite3_column_int(stmt, 3);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return states;
}

template<typename... Args>
std::vector<rmf_scheduler_msgs::msg::Schedule>
SqliteDataSource::_fetch_schedules(const std::string& where, Args&& ... args)
{
  std::string sql =
    R"(SELECT name, created_at, schedule, start_at, finish_at, "group",
payload_type, payload_data FROM Schedule )"
    + where;
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, std::forward<Args>(args)...);

  std::vector<rmf_scheduler_msgs::msg::Schedule> schedules;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    if (result != SQLITE_ROW)
    {
      throw DatabaseError(this->_db);
    }

    auto& schedule = schedules.emplace_back();
    schedule.name = (const char*) sqlite3_column_text(stmt, 0);
    schedule.created_at = sqlite3_column_int64(stmt, 1);
    schedule.schedule = (const char*) sqlite3_column_text(stmt, 2);
    schedule.start_at = sqlite3_column_int64(stmt, 3);
    schedule.finish_at = sqlite3_column_int64(stmt, 4);
    schedule.group = (const char*) sqlite3_column_text(stmt, 5);
    schedule.payload.type = sqlite3_column_int(stmt, 6);
    auto blob = (uint8_t*) sqlite3_column_blob(stmt, 7);
    auto payload_size = sqlite3_column_bytes(stmt, 7);
    using PayloadData = decltype(schedule.payload.data);
    schedule.payload.data = PayloadData{blob, blob + payload_size};
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return schedules;
}

template<typename... Args>
std::vector<rmf_scheduler_msgs::msg::ScheduleState>
SqliteDataSource::_fetch_schedule_states(const std::string& where,
  Args&& ... args)
{
  std::string sql =
    "SELECT name, last_modified, last_ran, next_run, status FROM Schedule " +
    where;
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, std::forward<Args>(args)...);

  std::vector<rmf_scheduler_msgs::msg::ScheduleState> states;
  for (int result = sqlite3_step(stmt); result != SQLITE_DONE;
    result = sqlite3_step(stmt))
  {
    auto& state = states.emplace_back();
    state.name = (const char*) sqlite3_column_text(stmt, 0);
    state.last_modified = sqlite3_column_int64(stmt, 1);
    state.last_ran = sqlite3_column_int64(stmt, 2);
    state.next_run = sqlite3_column_int64(stmt, 3);
    state.status = sqlite3_column_int(stmt, 4);
  }

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return states;
}

}
