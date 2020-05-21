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
Task::Task(std::vector<std::unique_ptr<PendingPhase>> phases)
  : _pending_phases(std::move(phases))
{
  std::reverse(_pending_phases.begin(), _pending_phases.end());
  _start_next_phase();
}

//==============================================================================
auto Task::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_publisher.observe();
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
  _active_phase_subscription.unsubscribe();
  _active_phase->cancel();

  // TODO(MXG): Should we send .on_error() or .on_completed() to the
  // subscribers?
}

//==============================================================================
void Task::_start_next_phase()
{
  if (_pending_phases.empty())
  {
    // All phases are now complete
    _active_phase = nullptr;
    _active_phase_subscription.unsubscribe();
    _status_publisher.complete();

    return;
  }

  _active_phase = _pending_phases.back()->begin();
  _pending_phases.pop_back();
  _active_phase_subscription =
      _active_phase->observe()
      .observe_on(rxcpp::observe_on_event_loop())
      .subscribe(
        [this](const rmf_task_msgs::msg::TaskSummary& summary)
        {
          // We have received a status update from the phase. We will forward
          // this to whoever is subscribing to the Task.
          this->_status_publisher.publish(summary);
        },
        [this]()
        {
          // We have received a completion notice from the phase
          this->_start_next_phase();
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
