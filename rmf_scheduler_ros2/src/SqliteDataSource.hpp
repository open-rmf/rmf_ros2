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

#include "SqliteCursor.hpp"

#include <rmf_scheduler_msgs/msg/schedule.hpp>
#include <rmf_scheduler_msgs/msg/schedule_state.hpp>
#include <rmf_scheduler_msgs/msg/trigger.hpp>
#include <rmf_scheduler_msgs/msg/trigger_state.hpp>

#include <optional>
#include <string>

class sqlite3;
class sqlite3_stmt;

namespace rmf::scheduler {

class SqliteDataSource
{
private:
  /// A RAII managed sqlite transaction. The destructor will rollback the transaction
  /// if it has not already been committed.
  class _Transaction
  {
  public:
    /// Creates a new transaction.
    ///
    /// :param root: Indicates if this is a root transaction. Non-root transactions
    ///   have no effect on the database and are only used for deferring a
    ///   transaction to the root.
    _Transaction(SqliteDataSource* store, bool root);

    // disallow copying and moving to simplify lifetime management
    _Transaction(const _Transaction&) = delete;
    _Transaction(_Transaction&&) = delete;
    _Transaction& operator=(const _Transaction&) = delete;
    _Transaction& operator=(_Transaction&&) = delete;

    ~_Transaction();

    void commit();

    void rollback();

  private:
    SqliteDataSource* _store;
    bool _finished = false;

    void _finish();
  };

public:
  SqliteDataSource(const std::string& file);

  // disallow copying and moving to simplify lifetime management
  SqliteDataSource(const SqliteDataSource&) = delete;
  SqliteDataSource(SqliteDataSource&&) = delete;
  SqliteDataSource& operator=(const SqliteDataSource&) = delete;
  SqliteDataSource& operator=(SqliteDataSource&&) = delete;

  ~SqliteDataSource();

  /// Starts a transaction if not currently in one.
  ///
  /// Writes within a transaction needs to be committed by calling
  /// `_Transaction::commit()` for changes to persisent, if the `_Transaction`
  /// is not committed, it's destructor will automatically rollback the transaction.
  ///
  /// Calling this while already in a transaction will return a `_Transaction` that
  /// have no effect. This allows "recursive transactions" such that the decision
  /// to commit or rollback is controlled by the root.
  ///
  /// Note that due to how sqlite works, changes within a transaction that is not yet
  /// committed are still visible to other reads.
  _Transaction begin_transaction();

  std::optional<rmf_scheduler_msgs::msg::Schedule>
  fetch_schedule(const std::string& name);

  /// Fetch schedules which are CREATED or STARTED.
  SqliteCursor<rmf_scheduler_msgs::msg::Schedule>
  fetch_active_schedules();

  SqliteCursor<rmf_scheduler_msgs::msg::Schedule>
  fetch_schedules_created_after(int64_t created_after);

  std::optional<rmf_scheduler_msgs::msg::ScheduleState>
  fetch_schedule_state(const std::string& name);

  SqliteCursor<rmf_scheduler_msgs::msg::ScheduleState>
  fetch_schedule_states_modified_after(int64_t modified_after);

  SqliteCursor<std::string>
  fetch_schedules_in_group(const std::string& group);

  std::optional<rmf_scheduler_msgs::msg::Trigger>
  fetch_trigger(const std::string& name);

  /// Fetch triggers which are STARTED.
  SqliteCursor<rmf_scheduler_msgs::msg::Trigger>
  fetch_active_triggers();

  SqliteCursor<rmf_scheduler_msgs::msg::Trigger>
  fetch_triggers_created_after(int64_t created_after);

  std::optional<rmf_scheduler_msgs::msg::TriggerState>
  fetch_trigger_state(const std::string& name);

  SqliteCursor<rmf_scheduler_msgs::msg::TriggerState>
  fetch_trigger_states_modified_after(int64_t modified_after);

  SqliteCursor<std::string>
  fetch_triggers_in_group(const std::string& group);

  void create_schedule(
    const rmf_scheduler_msgs::msg::Schedule& schedule,
    const rmf_scheduler_msgs::msg::ScheduleState& state);

  void save_schedule_state(
    const rmf_scheduler_msgs::msg::ScheduleState& state);

  void create_trigger(
    const rmf_scheduler_msgs::msg::Trigger& trigger,
    const rmf_scheduler_msgs::msg::TriggerState& state);

  void save_trigger_state(
    const rmf_scheduler_msgs::msg::TriggerState& state);

private:
  sqlite3* _db;
  bool _in_transaction = false;

  template<typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
  void _bind_arg(sqlite3_stmt* stmt, int i, T arg);

  template<size_t I>
  void _bind_arg(sqlite3_stmt* stmt, int i,
    const rosidl_runtime_cpp::BoundedVector<uint8_t, I>& arg);

  void _bind_arg(sqlite3_stmt* stmt, int i, const std::string& arg);

  template<typename... Args>
  sqlite3_stmt* _prepare_stmt(std::string_view sql, Args&& ... args);

  template<typename... Args>
  SqliteCursor<rmf_scheduler_msgs::msg::Trigger> _fetch_triggers(
    const std::string& where, Args&& ... args);

  template<typename... Args>
  SqliteCursor<rmf_scheduler_msgs::msg::TriggerState> _fetch_trigger_states(
    const std::string& where, Args&& ... args);

  template<typename... Args>
  SqliteCursor<rmf_scheduler_msgs::msg::Schedule> _fetch_schedules(
    const std::string& where, Args&& ... args);

  template<typename... Args>
  SqliteCursor<rmf_scheduler_msgs::msg::ScheduleState> _fetch_schedule_states(
    const std::string& where, Args&& ... args);
};

}
