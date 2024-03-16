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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__WAITFORTRAFFIC_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__WAITFORTRAFFIC_HPP

#include "../agv/RobotContext.hpp"
#include "../LegacyTask.hpp"

#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class WaitForTraffic : public rmf_task_sequence::Event
{
public:

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      agv::RobotContextPtr context,
      PlanIdPtr plan_id,
      rmf_traffic::Dependencies dependencies,
      rmf_traffic::Time expected_time,
      const AssignIDPtr& id,
      std::function<void()> update);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    agv::RobotContextPtr _context;
    PlanIdPtr _plan_id;
    rmf_traffic::Dependencies _dependencies;
    rmf_traffic::Time _expected_time;
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
      rmf_traffic::PlanId plan_id,
      const rmf_traffic::Dependencies& dependencies,
      rmf_traffic::Time expected_time,
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
    void _consider_going();
    void _replan();

    using DependencySubscription =
      rmf_traffic::schedule::ItineraryViewer::DependencySubscription;

    agv::RobotContextPtr _context;
    rmf_traffic::PlanId _plan_id;
    std::vector<DependencySubscription> _dependencies;
    rmf_traffic::Time _expected_time;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _update;
    std::function<void()> _finished;
    rclcpp::TimerBase::SharedPtr _timer;
    std::optional<rmf_traffic::Time> _decision_made;
    rmf_rxcpp::subscription_guard _mutex_group_listener;
  };

};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__WAITFORTRAFFIC_HPP
