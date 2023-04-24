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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__WAITUNTIL_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__WAITUNTIL_HPP

#include "../agv/RobotContext.hpp"

#include <rmf_task/events/SimpleEventState.hpp>

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/WaitFor.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class WaitUntil : public rmf_task_sequence::Event
{
public:

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      agv::RobotContextPtr context,
      rmf_traffic::Time until_time,
      const AssignIDPtr& id,
      std::function<void()> update);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    agv::RobotContextPtr _context;
    rmf_traffic::Time _until_time;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _update;
  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Active> make(
      agv::RobotContextPtr context,
      const rmf_traffic::Time until_time,
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
    agv::RobotContextPtr _context;
    rmf_traffic::Time _until_time;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _update;
    std::function<void()> _finished;
    std::optional<Eigen::Vector3d> _last_position;
    rclcpp::TimerBase::SharedPtr _timer;
    bool _is_interrupted = false;

    void _update_waiting();

    void _update_holding(
      rmf_traffic::Time now,
      Eigen::Vector3d position);
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__WAITUNTIL_HPP
