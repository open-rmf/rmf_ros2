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

#include <rmf_traffic/Motion.hpp>

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
      PlanIdPtr plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    PlanIdPtr _plan_id;
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
    std::optional<std::size_t> _first_graph_index;

    rclcpp::TimerBase::SharedPtr _update_timeout_timer;
    rclcpp::Time _last_update_rostime;
    // TODO(MXG): Make this timeout configurable by users
    rmf_traffic::Duration _update_timeout = std::chrono::seconds(10);
  };
};

template<typename Subscriber>
void MoveRobot::Action::operator()(const Subscriber& s)
{
  const auto command = _context->command();
  if (!command)
    return;

  _last_update_rostime = _context->node()->now();
  _update_timeout_timer = _context->node()->try_create_wall_timer(
    _update_timeout, [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto now = self->_context->node()->now();
      if (now < self->_last_update_rostime + self->_update_timeout)
      {
        // The simulation is paused or running slowly, so we should allow more
        // patience before assuming that there's been a timeout.
        return;
      }

      self->_last_update_rostime = now;

      // The RobotCommandHandle seems to have frozen up. Perhaps a bug in the
      // user's code has caused the RobotCommandHandle to drop the command. We
      // will request a replan.
      RCLCPP_WARN(
        self->_context->node()->get_logger(),
        "Requesting replan for [%s] because its command handle seems to be "
        "unresponsive",
        self->_context->requester_id().c_str());
      self->_context->request_replan();
    });

  const auto update = [s, w_action = weak_from_this(), r = _context->requester_id()](
      std::size_t path_index, rmf_traffic::Duration estimate)
    {
      const auto action = w_action.lock();
      if (!action)
        return;

      action->_last_update_rostime = action->_context->node()->now();
      action->_update_timeout_timer->reset();

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
            [
              context = action->_context,
              bump = *action->_tail_period,
              plan_id = action->_plan_id
            ](
              const auto&)
            {
              if (const auto c = context->itinerary().cumulative_delay(plan_id))
              {
                context->itinerary().cumulative_delay(plan_id, *c + bump);
              }
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

        s.on_next(msg);
      }

      if (action->_next_path_index > action->_waypoints.size())
      {
        return;
      }

      if (action->_plan_id != action->_context->itinerary().current_plan_id())
      {
        // If the current Plan ID of the itinerary does not match the Plan ID
        // of this action, then we should not modify the delay here.
        return;
      }

      const auto& target_wp = action->_waypoints[path_index];
      using namespace std::chrono_literals;
      const rmf_traffic::Time now = action->_context->now();
      const auto planned_time = target_wp.time();
      const auto newly_expected_arrival = now + estimate;
      const auto new_cumulative_delay = newly_expected_arrival - planned_time;

      action->_context->worker().schedule(
        [
          w = action->weak_from_this(),
          now,
          new_cumulative_delay
        ](const auto&)
        {
          const auto self = w.lock();
          if (!self)
            return;
          std::cout << "delaying " << self->_context->requester_id() << " plan id "
            << self->_plan_id << " by " << rmf_traffic::time::to_seconds(new_cumulative_delay)
            << std::endl;
          const auto context = self->_context;
          const auto plan_id = self->_plan_id;
          context->itinerary().cumulative_delay(
            plan_id, new_cumulative_delay, 100ms);

          // This itinerary has been adjusted according to the latest delay
          // information, so our position along the trajectory is given by `now`
          const auto& itin = context->itinerary().itinerary();
          for (std::size_t i = 0; i < itin.size(); ++i)
          {
            const auto& traj = itin[i].trajectory();
            const auto t_it = traj.find(now);
            if (t_it != traj.end() && t_it != traj.begin())
            {
              std::size_t index = t_it->index() - 1;
              if (t_it->time() == now)
              {
                index = t_it->index();
              }

              context->itinerary().reached(plan_id, i, index);
            }
          }

          if (!context->locked_mutex_group().empty())
          {
            const auto adjusted_now = now - new_cumulative_delay;
            const auto& graph = context->navigation_graph();
            for (const auto& wp : self->_waypoints)
            {
              if (wp.time() > adjusted_now)
              {
                break;
              }

              if (wp.graph_index().has_value())
              {
                if (self->_first_graph_index.has_value())
                {
                  const auto nav = self->_context->nav_params();
                  const auto i_wp = *wp.graph_index();
                  const auto i_first = *self->_first_graph_index;
                  bool ignore =
                    (nav && nav->in_same_stack(i_wp, i_first))
                    || i_wp == i_first;

                  if (ignore)
                  {
                    // The first waypoint doesn't always have a mutex group
                    // associated.
                    continue;
                  }
                }

                const auto& g = graph.get_waypoint(*wp.graph_index())
                  .in_mutex_group();
                if (g.empty())
                {
                  std::cout << __LINE__ << ": Releasing mutex at " << *wp.graph_index()
                    << " <" << wp.position().transpose() << ">"
                    << " | " << rmf_traffic::time::to_seconds(wp.time().time_since_epoch())
                    << " vs " << rmf_traffic::time::to_seconds(adjusted_now.time_since_epoch())
                    << std::endl;
                  context->release_mutex_group();
                  break;
                }
              }
            }
          }
        });
    };

  const auto finish = [s, w = weak_from_this(), name = _context->requester_id()]()
    {
      std::cout << "PATH FINISHER TRIGGERED" << std::endl;
      if (const auto self = w.lock())
      {
        if (!self->_waypoints.empty())
        {
          for (const auto& c : self->_waypoints.back().arrival_checkpoints())
          {
            self->_context->itinerary().reached(
              self->_plan_id, c.route_id, c.checkpoint_id);

            std::cout << "finish reached " << self->_context->participant_id()
              << " | " << self->_context->itinerary().current_plan_id()
              << ":" << c.route_id << ":" << c.checkpoint_id
              << " #" << self->_context->itinerary().progress_version() << std::endl;
          }

          const auto last_index = self->_waypoints.back().graph_index();
          if (last_index.has_value())
          {
            const auto& graph = self->_context->navigation_graph();
            if (graph.get_waypoint(*last_index).in_mutex_group().empty())
            {
              std::cout << __LINE__ << ": Releasing mutex at end of path" << std::endl;
              self->_context->release_mutex_group();
            }
            else
            {
              std::cout << __LINE__ << ": Not releasing because last waypoint still has mutex: "
                << graph.get_waypoint(*last_index).in_mutex_group() << std::endl;
            }
          }
        }
        else
        {
          std::cout << __LINE__ << " EMPTY WAYPOINTS??? FOR " << self->_context->requester_id() << std::endl;
        }

        LegacyTask::StatusMsg msg;
        msg.state = LegacyTask::StatusMsg::STATE_COMPLETED;
        msg.status = "move robot success";
        s.on_next(msg);
        s.on_completed();
      }
      else
      {
        std::cout << " ###### MOVE ROBOT IS PREMATURELY DEAD FOR " << name << std::endl;
      }
    };

  _context->command()->follow_new_path(
    _waypoints,
    [worker = _context->worker(), update](
      std::size_t path_index, rmf_traffic::Duration estimate)
    {
      worker.schedule([path_index, estimate, update](const auto&)
        {
          update(path_index, estimate);
        });
    },
    [worker = _context->worker(), finish]()
    {
      worker.schedule([finish](const auto&)
        {
          finish();
        });
    });
}

} // namespace phases
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
