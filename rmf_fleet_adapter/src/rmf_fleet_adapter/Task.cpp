/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "Task.hpp"

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <rxcpp/rx-observable.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
Task::Task(std::string id, std::vector<std::unique_ptr<PendingPhase>> phases)
  : _id(std::move(id)),
    _pending_phases(std::move(phases))
{
  std::cout << "Constructing task" << std::endl;
  _status_obs = _status_publisher.get_observable();
  std::reverse(_pending_phases.begin(), _pending_phases.end());
}

//==============================================================================
std::shared_ptr<Task> Task::make(
    std::string id, PendingPhases phases)
{
  return std::shared_ptr<Task>(new Task(std::move(id), std::move(phases)));
}

//==============================================================================
void Task::begin()
{
  if (!_active_phase)
    _start_next_phase();
}

//==============================================================================
auto Task::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
auto Task::current_phase() -> const std::shared_ptr<ActivePhase>&
{
  return _active_phase;
}

//==============================================================================
auto Task::current_phase() const -> std::shared_ptr<const ActivePhase>
{
  return _active_phase;
}

//==============================================================================
auto Task::pending_phases() const -> const PendingPhases&
{
  return _pending_phases;
}

//==============================================================================
void Task::cancel()
{
  _pending_phases.clear();
  _active_phase->cancel();
}

//==============================================================================
const std::string& Task::id() const
{
  return _id;
}

//==============================================================================
void Task::_start_next_phase()
{
  if (_pending_phases.empty())
  {
    // All phases are now complete
    _active_phase = nullptr;
    _active_phase_subscription.unsubscribe();
    _status_publisher.get_subscriber().on_completed();

    return;
  }

  std::cout << "About to call shared_from_this()" << std::endl;
  auto task = shared_from_this();
  std::cout << "Done with shared_from_this()" << std::endl;
  _active_phase = _pending_phases.back()->begin();
  _pending_phases.pop_back();
  _active_phase_subscription =
      _active_phase->observe()
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
        [w = std::weak_ptr<Task>(task)](
        const rmf_task_msgs::msg::TaskSummary& msg)
        {
          const auto task = w.lock();
          if (!task)
          {
            std::cout << "cannot lock task" << std::endl;
            return;
          }

          auto summary = msg;
          // We have received a status update from the phase. We will forward
          // this to whoever is subscribing to the Task.
          summary.task_id = task->_id;

          // We don't want to say that the task is complete until the very end.
          if (summary.STATE_COMPLETED == summary.state)
            summary.state = summary.STATE_ACTIVE;

          task->_status_publisher.get_subscriber().on_next(summary);
        },
        [w = std::weak_ptr<Task>(task)](std::exception_ptr e)
        {
          const auto task = w.lock();
          if (!task)
            return;

          task->_pending_phases.clear();
          std::string exception_msg;
          try
          {
            if (e)
              std::rethrow_exception(e);
          }
          catch(const std::exception& e)
          {
            exception_msg = e.what();
          }

          StatusMsg msg;
          msg.state = msg.STATE_FAILED;
          msg.status = "Failure at phase ["
            + task->_active_phase->description() + "]: "
            + exception_msg;

          task->_status_publisher.get_subscriber().on_next(msg);
        },
        [w = std::weak_ptr<Task>(task)]()
        {
          const auto task = w.lock();
          if (!task)
            return;

          // We have received a completion notice from the phase
          task->_start_next_phase();
        });
}

//==============================================================================
auto Task::_process_summary(const StatusMsg& input_msg) -> StatusMsg
{
  auto output = input_msg;
  if (!_initial_time.has_value())
    _initial_time = output.start_time;
  else
    output.start_time = *_initial_time;

  rclcpp::Time end_time = output.end_time;
  for (const auto& pending : _pending_phases)
    end_time = end_time + rclcpp::Duration(pending->estimate_phase_duration());

  output.end_time = end_time;

  return output;
}

} // namespace rmf_fleet_adapter
