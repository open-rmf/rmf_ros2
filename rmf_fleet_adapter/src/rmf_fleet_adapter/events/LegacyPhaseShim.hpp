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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__LEGACYPHASESHIM_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__LEGACYPHASESHIM_HPP

#include "../LegacyTask.hpp"

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
/// This class is being used to gradually migrate the legacy task phase system
/// into the new task event system. It provides a shim for legacy "Phase"
/// implementations to be turned into Event objects.
///
/// We should endeavor to deprecate the use of this class over time as we work
/// on improving the code quality of RMF.
class LegacyPhaseShim : public rmf_task_sequence::Event
{
public:

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      std::shared_ptr<LegacyTask::PendingPhase> legacy,
      rxcpp::schedulers::worker worker,
      std::function<rmf_traffic::Time()> clock,
      const AssignIDPtr& id,
      std::function<void()> parent_update,
      std::optional<std::string> name = std::nullopt);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    std::shared_ptr<LegacyTask::PendingPhase> _legacy;
    rxcpp::schedulers::worker _worker;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _parent_update;
  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Active> make(
      std::shared_ptr<LegacyTask::PendingPhase> legacy,
      rxcpp::schedulers::worker worker,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> parent_update,
      std::function<void()> finished);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:
    std::shared_ptr<LegacyTask::ActivePhase> _legacy;
    rmf_task::events::SimpleEventStatePtr _state;
    std::string _last_status_message;
    std::optional<uint32_t> _last_state_value;
    std::function<void()> _parent_update;
    std::function<void()> _finished;
    rmf_rxcpp::subscription_guard _subscription;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__LEGACYPHASESHIM_HPP
