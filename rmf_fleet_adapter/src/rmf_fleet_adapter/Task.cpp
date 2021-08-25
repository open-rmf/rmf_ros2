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

#include <malloc.h>

namespace rmf_fleet_adapter {

//==============================================================================
std::shared_ptr<Task> Task::make(
  std::string id,
  PendingPhases phases,
  rxcpp::schedulers::worker worker,
  rmf_traffic::Time deployment_time,
  rmf_task::agv::State finish_state,
  rmf_task::ConstRequestPtr request,
  CompletedPhases completed_phases)
{
  return std::make_shared<Task>(
    Task(std::move(id),
    std::move(phases),
    std::move(worker),
    deployment_time,
    finish_state,
    std::move(request),
    std::move(completed_phases)));
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
  if (_active_phase)
    _active_phase->cancel();
}

//==============================================================================
const std::string& Task::id() const
{
  return _id;
}

//==============================================================================
const rmf_task::ConstRequestPtr Task::request() const
{
  return _request;
}

//==============================================================================
const rmf_traffic::Time Task::deployment_time() const
{
  return _deployment_time;
}

//==============================================================================
const rmf_task::agv::State Task::finish_state() const
{
  return _finish_state;
}

//==============================================================================
auto Task::completed_phases() const -> const CompletedPhases
{
  return _completed_phases;
}

//==============================================================================
void Task::task_profile(Task::TaskProfileMsg profile)
{
  _profile = profile;
  return;
}

//==============================================================================
const Task::TaskProfileMsg& Task::task_profile() const
{
  return _profile;
}

//==============================================================================
Task::Task(
  std::string id,
  std::vector<std::unique_ptr<PendingPhase>> phases,
  rxcpp::schedulers::worker worker,
  rmf_traffic::Time deployment_time,
  rmf_task::agv::State finish_state,
  rmf_task::ConstRequestPtr request,
  std::vector<PhaseMsg> completed_phases)
: _id(std::move(id)),
  _pending_phases(std::move(phases)),
  _worker(std::move(worker)),
  _deployment_time(deployment_time),
  _finish_state(finish_state),
  _request(std::move(request)),
  _completed_phases(std::move(completed_phases))
{
  _status_obs = _status_publisher.get_observable();
  std::reverse(_pending_phases.begin(), _pending_phases.end());
}

//==============================================================================
void Task::_start_next_phase()
{
  if (_pending_phases.empty())
  {
    // All phases are now complete
    _active_phase = nullptr;
    _active_phase_subscription.get().unsubscribe();
    _status_publisher.get_subscriber().on_completed();

    // Sometimes difficult negotiations end up seizing an exceedingly large
    // amount of RAM. This function is used to allow the operating system
    // to take that RAM back after it's no longer needed. This is mostly
    // superficial, but it helps us know that the fleet adapter isn't leaking
    // huge amounts of memory.
    //
    // TODO(MXG): Remove this when the planner has been made more
    // memory-efficient.
    malloc_trim(0);

    return;
  }

  std::unique_ptr<PendingPhase> next_pending =
    std::move(_pending_phases.back());
  _pending_phases.pop_back();

  if (!next_pending)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Task::_start_next_phase] INTERNAL ERROR: Next phase has a null value");
    // *INDENT-ON*
  }
  _active_phase = next_pending->begin();
  _active_phase_estimate_duration = next_pending->estimate_phase_duration();
  _current_phase_start_time = std::chrono::steady_clock::now();

  _active_phase_subscription =
    _active_phase->observe()
    .observe_on(rxcpp::identity_same_worker(_worker))
    .subscribe(

    [w = weak_from_this()](
      const rmf_task_msgs::msg::TaskSummary& msg)
    {
      const auto task = w.lock();
      if (!task)
        return;

      auto summary = msg;
      // We have received a status update from the phase. We will forward
      // this to whoever is subscribing to the Task.
      summary.task_id = task->_id;
      task->_debug++;

      // We don't want to say that the task is complete until the very end.
      if (summary.STATE_COMPLETED == summary.state)
        summary.state = summary.STATE_ACTIVE;

      std::string print_completed = "";
      for (auto s : task->_debug_completed)
        print_completed += " " + s;

      summary.status += "\n | Remaining phases: "
          + std::to_string(task->_pending_phases.size() + 1)
          + ",  debug: " + std::to_string(task->_debug)
          + ",  debug: " + print_completed;

      summary.tiers.push_back(task->_generate_phase_tier());
      std::reverse(summary.tiers.begin(),summary.tiers.end()); // reverse

      task->_status_publisher.get_subscriber().on_next(summary);
    },
    [w = weak_from_this()](std::exception_ptr e)
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
      catch (const std::exception& e)
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
    [w = weak_from_this()]()
    {
      const auto task = w.lock();
      if (!task)
        return;

      // cache the the completd phase msg
      PhaseMsg phase;
      phase.title = task->_active_phase->title();
      phase.description = task->_active_phase->description();

      const auto p_d = task->_active_phase_estimate_duration;
      phase.predicted_duration = rclcpp::Duration(p_d);
      const auto rt_d = task->_active_phase->runtime_duration();
      phase.runtime_duration = rclcpp::Duration(rt_d);
        
      task->_completed_phases.push_back(phase);
      task->_debug += 10000;
      task->_debug_completed.push_back(task->_active_phase->title());

      // We have received a completion notice from the phase
      task->_start_next_phase();
    });
}

//==============================================================================
auto Task::_generate_phase_tier() -> PhaseTierMsg
{
  PhaseTierMsg phase_tier;
  phase_tier.current.title = _active_phase->title();
  phase_tier.current.description = _active_phase->description();
  const auto re_d = _active_phase->estimate_remaining_time();
  phase_tier.current.remaining_duration = rclcpp::Duration(re_d);
  const auto rt_d = _active_phase->runtime_duration();
  phase_tier.current.runtime_duration = rclcpp::Duration(rt_d);
  const auto p_d = _active_phase_estimate_duration;
  phase_tier.current.predicted_duration = rclcpp::Duration(p_d);

  // reverse pending phase according to impl sequence
  PhaseMsg phase;
  for(int i = _pending_phases.size()-1; i >= 0; i--)
  {
    phase.title = _pending_phases[i]->title();
    phase.description = _pending_phases[i]->description();
    const auto duration = 
      _pending_phases[i]->estimate_phase_duration();
    phase.predicted_duration = rclcpp::Duration(duration);
    phase.remaining_duration = phase.predicted_duration;
    phase_tier.pending.push_back(phase);
  }

  phase_tier.completed = _completed_phases;
  return phase_tier;
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
