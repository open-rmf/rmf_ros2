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
  _status_obs = _status_publisher.get_observable();
  std::reverse(_pending_phases.begin(), _pending_phases.end());
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
  std::cout << "Remaining phases: " << _pending_phases.size() << std::endl;
  if (_pending_phases.empty())
  {
    std::cout << "Finished (sub)task" << std::endl;
    // All phases are now complete
    _active_phase = nullptr;
    _active_phase_subscription.unsubscribe();
    _status_publisher.get_subscriber().on_completed();

    return;
  }

  std::cout << "About to begin [" << _pending_phases.back()->description() << "]" << std::endl;
  _active_phase = _pending_phases.back()->begin();
  _pending_phases.pop_back();
  _active_phase_subscription =
      _active_phase->observe()
      .observe_on(rxcpp::serialize_event_loop())
      .subscribe(
        [this](const rmf_task_msgs::msg::TaskSummary& msg)
        {
          auto summary = msg;
          std::cout << "[" << _id << "] update on task phase: "
                    << summary.status << std::endl;
          // We have received a status update from the phase. We will forward
          // this to whoever is subscribing to the Task.
          summary.task_id = this->_id;
          this->_status_publisher.get_subscriber().on_next(summary);

          // DEBUG: We should be calling _start_next_phase() from the
          // on_completed callback instead of calling it here, but when we do
          // that, the GoToPlace task fails to call it after the MoveRobot phase
          // in test_Delivery.
//          if (summary.state == summary.STATE_COMPLETED)
//            this->_start_next_phase();
        },
//        [this](std::exception_ptr e)
//        {
//          _pending_phases.clear();
//          std::string exception_msg;
//          try
//          {
//            if (e)
//              std::rethrow_exception(e);
//          }
//          catch(const std::exception& e)
//          {
//            exception_msg = e.what();
//          }

//          StatusMsg msg;
//          msg.state = msg.STATE_FAILED;
//          msg.status = "Failure at phase ["+_active_phase->description()+"]: "
//            + exception_msg;

//          this->_status_publisher.publish(msg);
//        },
        [this]()
        {
          std::cout << "[" << _id << "] ========== Finished phase" << std::endl;
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
