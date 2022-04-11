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

#pragma once

#include <exception>
#include <memory>
#include <string>

#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_record.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>

class sqlite3;
class sqlite3_stmt;

namespace rmf::scheduler {

class SqliteDataSource
{
private:
  using _SqlitePtr = std::shared_ptr<sqlite3>;

public:
  class DatabaseError : public std::exception
  {
  public:
    int code;

    explicit DatabaseError(_SqlitePtr db);
    explicit DatabaseError(int code, const char* errmsg);
    const char* what() const noexcept override;

  private: std::string errmsg;
  };

private:
  class _Transaction
  {
  public:
    _Transaction(SqliteDataSource* store);

    void create_trigger(const rmf_scheduler_msgs::msg::TriggerRecord& record);

    void save_trigger_state(
      const std::string& trigger_name,
      const rmf_scheduler_msgs::msg::TriggerState& state);

  private:
    SqliteDataSource* _store;
    _SqlitePtr _db;
  };

public:
  SqliteDataSource(const std::string& file);

  /// Only one transaction can be active at a time. Starting a 2nd transaction while the
  /// previous transaction has not been committed is a no-op.
  _Transaction begin_transaction();

  void commit_transaction();

  rmf_scheduler_msgs::msg::Trigger fetch_trigger(const std::string& name);
  rmf_scheduler_msgs::msg::TriggerState fetch_trigger_state(
    const std::string& name);

private:
  _SqlitePtr _db;

  template<typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
  void _bind_arg(sqlite3_stmt* stmt, int i, T arg);

  template<size_t I>
  void _bind_arg(sqlite3_stmt* stmt, int i,
    const rosidl_runtime_cpp::BoundedVector<uint8_t, I>& arg);

  void _bind_arg(sqlite3_stmt* stmt, int i, const std::string& arg);

  template<typename... Args>
  void _prepare_stmt(sqlite3_stmt** stmt, std::string_view sql,
    Args&& ... args);
};

}
