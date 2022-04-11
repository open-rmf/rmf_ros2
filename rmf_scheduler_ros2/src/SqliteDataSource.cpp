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

void SqliteDataSource::_Transaction::create_trigger(
  const rmf_scheduler_msgs::msg::TriggerRecord& record)
{
  const rmf_scheduler_msgs::msg::Trigger& trigger = record.trigger;
  const rmf_scheduler_msgs::msg::TriggerState& state = record.state;

  std::string sql =
    R"(
INSERT OR REPLACE INTO Trigger (name, created_at, last_modified, at, payload_type, payload_data, last_ran, status) VALUES (
  ?, ?, ?, ?, ?, ?, ?, ?
)
  )";
  sqlite3_stmt* stmt;
  this->_store->_prepare_stmt(&stmt, sql, trigger.name, record.created_at,
    record.state.last_modified,
    trigger.at, trigger.payload.type,
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
  const std::string& trigger_name,
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
    trigger_name);

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
  payload_type INTEGER NOT NULL,
  payload_data BLOB NOT NULL,
  last_ran INTEGER NOT NULL,
  status INTEGER NOT NULL
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

rmf_scheduler_msgs::msg::Trigger SqliteDataSource::fetch_trigger(
  const std::string& name)
{
  std::string sql =
    R"(
SELECT name, at, payload_type, payload_data FROM Trigger
WHERE name = ?
  )";
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, name);

  if (sqlite3_step(stmt) != SQLITE_ROW)
  {
    throw DatabaseError(this->_db);
  }

  rmf_scheduler_msgs::msg::Trigger trigger;
  trigger.name = (const char*) sqlite3_column_text(stmt, 0);
  trigger.at = sqlite3_column_int64(stmt, 1);
  trigger.payload.type = sqlite3_column_int(stmt, 2);
  auto blob = (uint8_t*) sqlite3_column_blob(stmt, 3);
  auto payload_size = sqlite3_column_bytes(stmt, 3);
  using PayloadData = decltype(trigger.payload.data);
  trigger.payload.data = PayloadData{blob, blob + payload_size};

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return trigger;
}

rmf_scheduler_msgs::msg::TriggerState SqliteDataSource::fetch_trigger_state(
  const std::string& name)
{
  std::string sql =
    R"(
SELECT last_modified, last_ran, status FROM Trigger
WHERE name = ?
  )";
  sqlite3_stmt* stmt;
  this->_prepare_stmt(&stmt, sql, name);

  if (sqlite3_step(stmt) != SQLITE_ROW)
  {
    throw DatabaseError(this->_db);
  }

  rmf_scheduler_msgs::msg::TriggerState state;
  state.last_modified = sqlite3_column_int64(stmt, 0);
  state.last_ran = sqlite3_column_int64(stmt, 1);
  state.status = sqlite3_column_int(stmt, 2);

  if (sqlite3_finalize(stmt) != SQLITE_OK)
  {
    throw DatabaseError(this->_db);
  }

  return state;
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

}
