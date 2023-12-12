/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__LOCKMUTEXGROUP_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__LOCKMUTEXGROUP_HPP

#include "../agv/RobotContext.hpp"
#include "../services/FindPath.hpp"

#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
std::string all_str(const std::unordered_set<std::string>& all);

//==============================================================================
class LockMutexGroup : public rmf_task_sequence::Event
{
public:
  struct Data
  {
    std::unordered_set<std::string> mutex_groups;
    std::string hold_map;
    Eigen::Vector3d hold_position;
    rmf_traffic::Time hold_time;
    std::shared_ptr<rmf_traffic::PlanId> plan_id;
    std::shared_ptr<rmf_traffic::schedule::Itinerary> resume_itinerary;
    std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints;
    rmf_traffic::agv::Plan::Goal goal;

    std::string all_groups_str() const;
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:
    static std::shared_ptr<Standby> make(
      agv::RobotContextPtr context,
      const AssignIDPtr& id,
      Data data);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    Standby(Data data);
    agv::RobotContextPtr _context;
    rmf_task::events::SimpleEventStatePtr _state;
    Data _data;
  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:
    static std::shared_ptr<Active> make(
      agv::RobotContextPtr context,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> finished,
      Data data);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:
    Active(Data data);
    void _initialize();
    void _schedule(rmf_traffic::schedule::Itinerary itinerary) const;
    void _apply_cumulative_delay();
    bool _consider_plan_result(services::FindPath::Result result);
    agv::RobotContextPtr _context;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _finished;
    rmf_rxcpp::subscription_guard _listener;
    rclcpp::TimerBase::SharedPtr _delay_timer;
    std::shared_ptr<void> _stubborn;
    Data _data;
    std::unordered_set<std::string> _remaining;
    rmf_rxcpp::subscription_guard _plan_subscription;
    std::shared_ptr<services::FindPath> _find_path_service;
    rclcpp::TimerBase::SharedPtr _find_path_timeout;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__LOCKMUTEXGROUP_HPP
