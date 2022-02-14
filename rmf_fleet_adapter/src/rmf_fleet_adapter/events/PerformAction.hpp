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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__PERFORMACTION_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__PERFORMACTION_HPP

#include "../agv/internal_RobotUpdateHandle.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/PerformAction.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

#include <nlohmann/json.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class PerformAction : public rmf_task_sequence::Event
{
public:

  using Description = rmf_task_sequence::events::PerformAction::Description;

  static void add(rmf_task_sequence::Event::Initializer& initializer);

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const rmf_task_sequence::events::PerformAction::Description& description,
      std::function<void()> update);

    /// Documentation inherited
    ConstStatePtr state() const final;

    /// Documentation inherited
    rmf_traffic::Duration duration_estimate() const final;

    /// Documentation inherited
    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:

    Standby(rmf_task_sequence::events::PerformAction::Description description);

    rmf_task_sequence::events::PerformAction::Description _description;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    rmf_traffic::Duration _time_estimate;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state;
    ActivePtr _active = nullptr;
  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:

    using ExecutionData =
      agv::RobotUpdateHandle::ActionExecution::Implementation::Data;

    static std::shared_ptr<Active> make(
      const AssignIDPtr& id,
      agv::RobotContextPtr context,
      const std::string& action_category,
      nlohmann::json action_description,
      rmf_traffic::Duration _time_estimate,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> update,
      std::function<void()> finished);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:

    Active(
      const std::string& action_category,
      nlohmann::json action_description,
      rmf_traffic::Duration time_estimate);

    void _execute_action();

    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    std::string _action_category;
    nlohmann::json _action_description;
    rmf_traffic::Duration _time_estimate;
    std::function<void()> _update;
    std::function<void()> _finished;
    rmf_task::events::SimpleEventStatePtr _state;
    rmf_traffic::Time _expected_finish_time;
    std::shared_ptr<void> _be_stubborn;
    std::weak_ptr<ExecutionData> _execution_data;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__PERFORMACTION_HPP
