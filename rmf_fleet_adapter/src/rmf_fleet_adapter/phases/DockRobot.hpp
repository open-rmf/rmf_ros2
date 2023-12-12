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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
struct DockRobot
{
  class Action;

  class ActivePhase : public LegacyTask::ActivePhase,
    public std::enable_shared_from_this<ActivePhase>
  {
  public:

    ActivePhase(
      agv::RobotContextPtr context,
      std::string dock_name,
      rmf_traffic::agv::Plan::Waypoint waypoint,
      rmf_traffic::PlanId plan_id);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

    std::shared_ptr<Action> action;
    rxcpp::observable<LegacyTask::StatusMsg> obs;
  private:
    friend class Action;

    agv::RobotContextPtr _context;
    std::string _dock_name;
    std::string _description;
    rmf_traffic::agv::Plan::Waypoint _waypoint;
    rmf_traffic::PlanId _plan_id;
    std::shared_ptr<void> _be_stubborn;
  };

  class PendingPhase : public LegacyTask::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::string dock_name,
      rmf_traffic::agv::Plan::Waypoint waypoint,
      PlanIdPtr plan_id);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _dock_name;
    std::string _description;
    rmf_traffic::agv::Plan::Waypoint _waypoint;
    PlanIdPtr _plan_id;
  };

  class Action
  {
  public:

    Action(std::weak_ptr<ActivePhase> phase);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:
    std::weak_ptr<ActivePhase> _phase;
  };
};

//==============================================================================
template<typename Subscriber>
void DockRobot::Action::operator()(const Subscriber& s)
{
  const auto active = _phase.lock();
  if (!active)
    return;

  active->_context->worker().schedule(
    [s, w = active->weak_from_this()](const auto&)
    {
      const auto active = w.lock();
      if (!active)
        return;

      LegacyTask::StatusMsg status;
      status.state = LegacyTask::StatusMsg::STATE_ACTIVE;
      status.status = "Docking [" + active->_context->requester_id() +
      "] into dock ["
      + active->_dock_name + "]";

      s.on_next(status);
      active->_context->command()->dock(
        active->_dock_name,
        [s, dock_name = active->_dock_name, context = active->_context,
        wp = active->_waypoint, plan_id = active->_plan_id]()
        {
          context->worker().schedule(
            [s, dock_name, context, wp, plan_id](const auto&)
            {
              LegacyTask::StatusMsg status;
              status.status = "Finished docking [" + context->requester_id()
              + "] into dock [" + dock_name + "]";
              status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
              for (const auto& c : wp.arrival_checkpoints())
              {
                context->itinerary().reached(plan_id, c.route_id,
                c.checkpoint_id);
              }

              if (wp.graph_index().has_value())
              {
                const auto& graph = context->navigation_graph();
                context->retain_mutex_groups(
                  {graph.get_waypoint(*wp.graph_index()).in_mutex_group()});
              }

              const auto now = context->now();
              const auto cumulative_delay = now - wp.time();
              context->itinerary().cumulative_delay(
                plan_id, cumulative_delay, std::chrono::seconds(1));
              s.on_next(status);
              s.on_completed();
            });
        });
    });
}

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP
