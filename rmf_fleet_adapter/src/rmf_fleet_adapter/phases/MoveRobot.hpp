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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace phases {

namespace {
//==============================================================================
inline std::string destination(
  const rmf_traffic::agv::Plan::Waypoint& wp,
  const rmf_traffic::agv::Graph& graph)
{
  if (wp.graph_index().has_value())
    return rmf_task::standard_waypoint_name(graph, *wp.graph_index());

  std::ostringstream oss;
  oss << "(" << wp.position().block<2, 1>(0, 0).transpose() << ")";
  return oss.str();
}
} // anonymous namespace

struct MoveRobot
{
  class Action;

  class ActivePhase : public LegacyTask::ActivePhase
  {
  public:

    ActivePhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _description;
    std::shared_ptr<Action> _action;
    rxcpp::observable<LegacyTask::StatusMsg> _obs;
    rxcpp::subjects::subject<bool> _cancel_subject;
    std::optional<rmf_traffic::Duration> _tail_period;
  };

  class PendingPhase : public LegacyTask::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    rmf_traffic::PlanId _plan_id;
    std::optional<rmf_traffic::Duration> _tail_period;
    std::string _description;
  };

  class Action : public std::enable_shared_from_this<Action>
  {
  public:

    Action(
      agv::RobotContextPtr& context,
      std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    rmf_traffic::PlanId _plan_id;
    std::optional<rmf_traffic::Duration> _tail_period;
    std::optional<rmf_traffic::Time> _last_tail_bump;
    std::size_t _next_path_index = 0;
  };
};

template<typename Subscriber>
void MoveRobot::Action::operator()(const Subscriber& s)
{
  const auto command = _context->command();
  if (!command)
    return;

  _context->command()->follow_new_path(
    _waypoints,
    [s, w_action = weak_from_this(), r = _context->requester_id()](
      std::size_t path_index, rmf_traffic::Duration estimate)
    {
      const auto action = w_action.lock();
      if (!action)
        return;

      if (path_index == action->_waypoints.size()-1
      && estimate < std::chrono::seconds(1)
      && action->_tail_period.has_value())
      {
        const auto now = action->_context->now();
        if (!action->_last_tail_bump.has_value()
        || *action->_last_tail_bump + *action->_tail_period < now)
        {
          action->_last_tail_bump = now;
          action->_context->worker().schedule(
            [context = action->_context, bump = *action->_tail_period](
              const auto&)
            {
              context->itinerary().delay(bump);
            });
        }
      }

      if (path_index != action->_next_path_index)
      {
        action->_next_path_index = path_index;
        LegacyTask::StatusMsg msg;
        msg.state = LegacyTask::StatusMsg::STATE_ACTIVE;

        if (path_index < action->_waypoints.size())
        {
          msg.status = "Heading towards "
          + destination(
            action->_waypoints[path_index],
            action->_context->planner()->get_configuration().graph());
        }
        else
        {
          // TODO(MXG): This should really be a warning, but the legacy phase shim
          // does not have a way for us to specify a warning.
          msg.status = "[Bug] [MoveRobot] Current path index was specified as ["
          + std::to_string(path_index) + "] but that exceeds the limit of ["
          + std::to_string(action->_waypoints.size()-1) + "]";
        }

        if (path_index > 0)
        {
          const auto& arrival =
          action->_waypoints[path_index - 1].arrival_checkpoints();

          for (const auto& c : arrival)
          {
            action->_context->itinerary()
            .reached(action->_plan_id, c.route_id, c.checkpoint_id);
          }
        }

        s.on_next(msg);
      }

      if (action->_next_path_index > action->_waypoints.size())
      {
        return;
      }

      const auto& target_wp = action->_waypoints[path_index];
      const auto& itinerary = action->_context->itinerary().itinerary();
      for (const auto& progress : target_wp.progress_checkpoints())
      {
        for (const auto& c : progress.checkpoints)
        {
          const auto& c_wp =
          itinerary[c.route_id].trajectory()[c.checkpoint_id];

          if (estimate < target_wp.time() - c_wp.time())
          {
            action->_context->itinerary()
            .reached(action->_plan_id, c.route_id, c.checkpoint_id);
          }
        }
      }

      const auto current_delay = action->_context->itinerary().delay();

      const rmf_traffic::Time now = action->_context->now();
      const auto planned_time = target_wp.time();
      const auto previously_expected_arrival = planned_time + current_delay;
      const auto newly_expected_arrival = now + estimate;
      const auto new_delay =
      newly_expected_arrival - previously_expected_arrival;

      if (std::chrono::milliseconds(500).count() < std::abs(new_delay.count()))
      {
        action->_context->worker().schedule(
          [context = action->_context, new_delay](const auto&)
          {
            context->itinerary().delay(new_delay);
          });
      }
    },
    [s, w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        for (const auto& c : self->_waypoints.back().arrival_checkpoints())
        {
          self->_context->itinerary().reached(
            self->_plan_id, c.route_id, c.checkpoint_id);
        }
      }

      LegacyTask::StatusMsg msg;
      msg.state = LegacyTask::StatusMsg::STATE_COMPLETED;
      msg.status = "move robot success";
      s.on_next(msg);

      s.on_completed();

    });
}

} // namespace phases
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
