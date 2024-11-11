/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "LegacyPhaseShim.hpp"

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
auto LegacyPhaseShim::Standby::make(
  std::shared_ptr<LegacyTask::PendingPhase> legacy,
  rxcpp::schedulers::worker worker,
  std::function<rmf_traffic::Time()> clock,
  const AssignIDPtr& id,
  std::function<void()> parent_update,
  std::optional<std::string> name) -> std::shared_ptr<Standby>
{
  auto standby = std::make_shared<Standby>();
  standby->_legacy = std::move(legacy);
  standby->_worker = std::move(worker);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(), "", "",
    rmf_task::Event::Status::Standby, {}, std::move(clock));

  if (name.has_value())
  {
    standby->_state->update_name(*name);
    standby->_state->update_detail(standby->_legacy->description());
  }
  else
  {
    standby->_state->update_name(standby->_legacy->description());
  }

  standby->_parent_update = std::move(parent_update);

  return standby;
}

//==============================================================================
auto LegacyPhaseShim::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration LegacyPhaseShim::Standby::duration_estimate() const
{
  return _legacy->estimate_phase_duration();
}

//==============================================================================
auto LegacyPhaseShim::Standby::begin(
  std::function<void()> /* none of the legacy phases have checkpoints */,
  std::function<void()> finished) -> ActivePtr
{
  if (!_legacy)
  {
    /* *INDENT-OFF* */
    throw std::runtime_error(
      "[rmf_fleet_adapter::events::LegacyPhaseShim::begin] "
      "Triggering begin twice!");
    /* *INDENT-ON* */
  }

  return Active::make(
    std::move(_legacy),
    std::move(_worker),
    std::move(_state),
    std::move(_parent_update),
    std::move(finished));
}

//==============================================================================
auto LegacyPhaseShim::Active::make(
  std::shared_ptr<LegacyTask::PendingPhase> legacy,
  rxcpp::schedulers::worker worker,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> parent_update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  using rmf_task_msgs::msg::TaskSummary;
  using rmf_task::Event;

  auto active = std::make_shared<Active>();
  active->_state = std::move(state);
  active->_parent_update = std::move(parent_update);
  active->_finished = std::move(finished);
  active->_legacy = legacy->begin();
  active->_subscription = active->_legacy->observe()
    .observe_on(rxcpp::identity_same_worker(worker))
    .subscribe(
    [w = active->weak_from_this()](
      const rmf_task_msgs::msg::TaskSummary& msg)
    {
      if (const auto self = w.lock())
      {
        bool need_update = false;
        if (msg.status != self->_last_status_message)
        {
          need_update = true;
          self->_last_status_message = msg.status;
          auto& log = self->_state->update_log();
          if (msg.state == TaskSummary::STATE_FAILED)
            log.error(msg.status);
          else
            log.info(msg.status);
        }

        /* *INDENT-OFF* */
        if (!self->_last_state_value.has_value()
            || *self->_last_state_value != msg.state)
        {
          need_update = true;
          self->_last_state_value = msg.state;
          if (msg.state == TaskSummary::STATE_QUEUED
              || msg.state == TaskSummary::STATE_PENDING)
            self->_state->update_status(Event::Status::Standby);
          else if (msg.state == TaskSummary::STATE_ACTIVE)
            self->_state->update_status(Event::Status::Underway);
          else if (msg.state == TaskSummary::STATE_COMPLETED)
            self->_state->update_status(Event::Status::Completed);
          else if (msg.state == TaskSummary::STATE_FAILED)
            self->_state->update_status(Event::Status::Failed);
          else if (msg.state == TaskSummary::STATE_CANCELED)
            self->_state->update_status(Event::Status::Canceled);
          else
            self->_state->update_status(Event::Status::Uninitialized);
        }
        /* *INDENT-ON* */

        if (need_update)
          self->_parent_update();
      }
    },
    [w = active->weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        if (self->_finished)
        {
          // We don't change the event status here because that should have been
          // done in the task summary update, and from here we don't know what
          // kind of finish the legacy phase may have had (e.g. completed,
          // failed, or canceled).
          const auto finished = self->_finished;
          self->_finished = nullptr;
          finished();
        }
      }
    });

  return active;
}

//==============================================================================
auto LegacyPhaseShim::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration LegacyPhaseShim::Active::remaining_time_estimate() const
{
  return _legacy->estimate_remaining_time();
}

//==============================================================================
auto LegacyPhaseShim::Active::backup() const -> Backup
{
  // Legacy phases don't have backups
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto LegacyPhaseShim::Active::interrupt(std::function<void()>) -> Resume
{
  // Legacy phases cannot be interrupted. We need to let the phase just run to
  // completion. We will never be able to safely trigger the task_is_interrupted
  // callback.
  return Resume::make([]() { /* do nothing */});
}

//==============================================================================
void LegacyPhaseShim::Active::cancel()
{
  _legacy->cancel();
}

//==============================================================================
void LegacyPhaseShim::Active::kill()
{
  // If we get a kill command then we'll just give up on whatever we're supposed
  // to do without concern for consequences. When an operator kills a task, it's
  // up to the operator to sort out the consequences.
  if (_finished)
  {
    const auto finished = _finished;
    _finished = nullptr;
    finished();
  }
}

} // namespace events
} // namespace rmf_fleet_adapter
