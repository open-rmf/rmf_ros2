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

#include "ExecutePlan.hpp"
#include "LegacyPhaseShim.hpp"
#include "WaitForTraffic.hpp"
#include "WaitUntil.hpp"
#include "LockMutexGroup.hpp"

#include "../phases/MoveRobot.hpp"
#include "../phases/DoorOpen.hpp"
#include "../phases/DoorClose.hpp"
#include "../phases/RequestLift.hpp"
#include "../phases/DockRobot.hpp"

#include "../agv/internal_EasyFullControl.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>

namespace rmf_fleet_adapter {
namespace events {

namespace {
//==============================================================================
using StandbyPtr = rmf_task_sequence::Event::StandbyPtr;
using UpdateFn = std::function<void()>;
using MakeStandby = std::function<StandbyPtr(UpdateFn)>;

std::string print_plan_waypoints(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  const rmf_traffic::agv::Graph& graph)
{
  std::stringstream ss;
  for (const auto& wp : waypoints)
    ss << "\n -- " << agv::print_plan_waypoint(wp, graph,
      waypoints.front().time());
  return ss.str();
}

//==============================================================================
struct LegacyPhaseWrapper
{
  LegacyPhaseWrapper(
    std::shared_ptr<LegacyTask::PendingPhase> phase_,
    rmf_traffic::Time time_,
    rmf_traffic::Dependencies dependencies_,
    std::optional<LockMutexGroup::Data> mutex_group_dependency_)
  : phase(std::move(phase_)),
    time(time_),
    dependencies(std::move(dependencies_)),
    mutex_group_dependency(std::move(mutex_group_dependency_))
  {
    // Do nothing
  }

  std::shared_ptr<LegacyTask::PendingPhase> phase;
  rmf_traffic::Time time;
  rmf_traffic::Dependencies dependencies;
  std::optional<LockMutexGroup::Data> mutex_group_dependency;
};

using LegacyPhases = std::vector<LegacyPhaseWrapper>;

using DockRobot = phases::DockRobot::PendingPhase;
using DoorOpen = phases::DoorOpen::PendingPhase;
using DoorClose = phases::DoorClose::PendingPhase;
using RequestLift = phases::RequestLift::PendingPhase;
using EndLift = phases::EndLiftSession::Pending;
using Move = phases::MoveRobot::PendingPhase;

//==============================================================================
MakeStandby make_wait_for_mutex(
  const agv::RobotContextPtr& context,
  const rmf_task_sequence::Event::AssignIDPtr& id,
  const LockMutexGroup::Data& data)
{
  return [context, id, data](UpdateFn update)
    {
      return LockMutexGroup::Standby::make(context, id, data);
    };
}

//==============================================================================
MakeStandby make_wait_for_traffic(
  const agv::RobotContextPtr& context,
  const PlanIdPtr plan_id,
  const rmf_traffic::Dependencies& deps,
  const rmf_traffic::Time time,
  const rmf_task_sequence::Event::AssignIDPtr& id)
{
  return [context, plan_id, deps, time, id](UpdateFn update)
    {
      return WaitForTraffic::Standby::make(
        context, plan_id, deps, time, id, std::move(update));
    };
}

//==============================================================================
void truncate_arrival(
  rmf_traffic::schedule::Itinerary& previous_itinerary,
  const rmf_traffic::agv::Plan::Waypoint& wp)
{
  std::size_t first_excluded_route = 0;
  for (const auto& c : wp.arrival_checkpoints())
  {
    first_excluded_route = std::max(first_excluded_route, c.route_id+1);
    auto& r = previous_itinerary.at(c.route_id);
    auto& t = r.trajectory();

    t.erase(t.begin() + (int)c.checkpoint_id, t.end());
  }

  for (std::size_t i = 0; i < previous_itinerary.size(); ++i)
  {
    const auto& t = previous_itinerary.at(i).trajectory();
    if (t.size() < 2)
    {
      // If we've reduced this trajectory down to nothing, then erase
      // it and all later routes. In the current version of RMF
      // we assume that routes with higher indices will never have an
      // earlier time value than the earliest of a lower index route.
      // This is an assumption we should relax in the future, but it
      // helps here for now.
      first_excluded_route =
        std::min(first_excluded_route, i);
    }
  }

  previous_itinerary.erase(
    previous_itinerary.begin()+first_excluded_route,
    previous_itinerary.end());
}

//==============================================================================
class EventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;

  EventPhaseFactory(
    agv::RobotContextPtr context,
    LegacyPhases& phases,
    const rmf_traffic::agv::Plan::Waypoint& initial_waypoint_,
    std::optional<rmf_traffic::agv::Plan::Waypoint> next_waypoint_,
    const PlanIdPtr plan_id,
    std::function<LockMutexGroup::Data(
      const std::unordered_set<std::string>&,
      const rmf_traffic::agv::Plan::Waypoint&)> make_current_mutex_groups,
    std::function<std::pair<bool, std::unordered_set<std::string>>(
      const rmf_traffic::agv::Plan::Waypoint&)> get_new_mutex_groups,
    std::shared_ptr<rmf_traffic::schedule::Itinerary>& previous_itinerary,
    const rmf_traffic::schedule::Itinerary& full_itinerary,
    bool& continuous)
  : initial_waypoint(initial_waypoint_),
    next_waypoint(std::move(next_waypoint_)),
    _context(std::move(context)),
    _phases(phases),
    _event_start_time(initial_waypoint_.time()),
    _plan_id(plan_id),
    _make_current_mutex_groups(std::move(make_current_mutex_groups)),
    _get_new_mutex_group(std::move(get_new_mutex_groups)),
    _previous_itinerary(previous_itinerary),
    _full_itinerary(full_itinerary),
    _continuous(continuous)
  {
    // Do nothing
  }

  rmf_traffic::agv::Plan::Waypoint initial_waypoint;
  std::optional<rmf_traffic::agv::Plan::Waypoint> next_waypoint;

  void execute(const Dock& dock) final
  {
    std::optional<LockMutexGroup::Data> event_mutex_group;
    if (next_waypoint.has_value() && next_waypoint->graph_index().has_value())
    {
      const auto [mutex_group_change, new_mutex_groups] =
        _get_new_mutex_group(*next_waypoint);

      if (mutex_group_change)
      {
        event_mutex_group = _make_current_mutex_groups(
          new_mutex_groups, initial_waypoint);
      }
    }

    assert(!_moving_lift);
    _phases.emplace_back(
      std::make_shared<phases::DockRobot::PendingPhase>(
        _context, dock.dock_name(),
        next_waypoint.value_or(initial_waypoint),
        _plan_id),
      _event_start_time, initial_waypoint.dependencies(), event_mutex_group);
    _continuous = false;
  }

  void execute(const DoorOpen& open) final
  {
    assert(!_moving_lift);
    const auto node = _context->node();
    _phases.emplace_back(
      std::make_shared<phases::DoorOpen::PendingPhase>(
        _context,
        open.name(),
        _context->requester_id(),
        _event_start_time + open.duration()),
      _event_start_time, initial_waypoint.dependencies(), std::nullopt);
    _continuous = true;
  }

  void execute(const DoorClose& close) final
  {
    assert(!_moving_lift);

    // TODO(MXG): Account for event duration in this phase
    const auto node = _context->node();
    _phases.emplace_back(
      std::make_shared<phases::DoorClose::PendingPhase>(
        _context,
        close.name(),
        _context->requester_id()),
      _event_start_time, initial_waypoint.dependencies(), std::nullopt);
    _continuous = true;
  }

  void execute(const LiftSessionBegin& open) final
  {
    truncate_arrival(*_previous_itinerary, initial_waypoint);

    _previous_itinerary =
      std::make_shared<rmf_traffic::schedule::Itinerary>(_full_itinerary);

    assert(!_moving_lift);
    const auto node = _context->node();
    _phases.emplace_back(
      std::make_shared<phases::RequestLift::PendingPhase>(
        _context,
        open.lift_name(),
        open.floor_name(),
        phases::RequestLift::Data{
          initial_waypoint.time(),
          phases::RequestLift::Located::Outside,
          _plan_id,
          std::nullopt,
          _previous_itinerary,
          initial_waypoint
        }),
      _event_start_time, initial_waypoint.dependencies(), std::nullopt);

    _continuous = true;
  }

  void execute(const LiftMove& move) final
  {
    // TODO(MXG): We should probably keep track of what lift is being moved to
    // make sure we weren't given a broken nav graph
    _lifting_duration += move.duration();
    _moving_lift = true;

    _continuous = true;
  }

  void execute(const LiftDoorOpen& open) final
  {
    std::string name;
    const auto& graph = _context->navigation_graph();
    const auto i1 = initial_waypoint.graph_index();
    if (const auto nav_params = _context->nav_params())
    {
      name = nav_params->get_vertex_name(graph, i1);
    }
    else if (i1.has_value())
    {
      if (const std::string* n = graph.get_waypoint(i1.value()).name())
      {
        name = *n;
      }
    }

    const auto node = _context->node();
    auto localize = agv::Destination::Implementation::make(
      open.floor_name(),
      initial_waypoint.position(),
      initial_waypoint.graph_index(),
      name,
      std::nullopt,
      nullptr);

    // TODO(MXG): The time calculation here should be considered more carefully.
    _phases.emplace_back(
      std::make_shared<phases::RequestLift::PendingPhase>(
        _context,
        open.lift_name(),
        open.floor_name(),
        phases::RequestLift::Data{
          _event_start_time + open.duration() + _lifting_duration,
          phases::RequestLift::Located::Inside,
          _plan_id,
          localize
        }),
      _event_start_time, initial_waypoint.dependencies(), std::nullopt);
    _moving_lift = false;

    _continuous = true;
  }

  void execute(const LiftSessionEnd& close) final
  {
    assert(!_moving_lift);
    const auto node = _context->node();
    _phases.emplace_back(
      std::make_shared<phases::EndLiftSession::Pending>(
        _context,
        close.lift_name(),
        close.floor_name()),
      _event_start_time, initial_waypoint.dependencies(), std::nullopt);

    _continuous = true;
  }

  void execute(const Wait&) final
  {
    // Do nothing
  }

  bool moving_lift() const
  {
    return _moving_lift;
  }

private:
  agv::RobotContextPtr _context;
  LegacyPhases& _phases;
  rmf_traffic::Time _event_start_time;
  PlanIdPtr _plan_id;
  std::function<LockMutexGroup::Data(
      const std::unordered_set<std::string>&,
      const rmf_traffic::agv::Plan::Waypoint&)> _make_current_mutex_groups;
  std::function<std::pair<bool, std::unordered_set<std::string>>(
      const rmf_traffic::agv::Plan::Waypoint&)> _get_new_mutex_group;
  std::shared_ptr<rmf_traffic::schedule::Itinerary>& _previous_itinerary;
  const rmf_traffic::schedule::Itinerary& _full_itinerary;
  bool& _continuous;
  bool _moving_lift = false;
  rmf_traffic::Duration _lifting_duration = rmf_traffic::Duration(0);
};

//==============================================================================
struct EventGroupInfo
{
  MakeStandby group;
  LegacyPhases::const_iterator tail;
};

//==============================================================================
std::optional<EventGroupInfo> search_for_door_group(
  LegacyPhases::const_iterator head,
  LegacyPhases::const_iterator end,
  const agv::RobotContextPtr& context,
  const PlanIdPtr plan_id,
  const rmf_task::Event::AssignIDPtr& id)
{
  const auto* door_open = dynamic_cast<const phases::DoorOpen::PendingPhase*>(
    head->phase.get());

  if (!door_open)
    return std::nullopt;

  // Look for a door close event for this same door
  auto tail = head;
  ++tail;
  auto moving_duration = rmf_traffic::Duration(0);
  while (tail != end)
  {
    const auto* tail_event = tail->phase.get();
    if (const auto* door_close = dynamic_cast<const DoorClose*>(tail_event))
    {
      if (door_open->door_name() != door_close->door_name())
      {
        // A different door is being closed, so we should not lump this all
        // together
        return std::nullopt;
      }

      // We have found the event where the robot is finished using the door.
      // Let's lump these events together.

      auto group_state = rmf_task::events::SimpleEventState::make(
        id->assign(), "Pass through [door:" + door_open->door_name() + "]",
        "", rmf_task::Event::Status::Standby, {}, context->clock());

      std::vector<MakeStandby> door_group;
      ++tail;
      for (auto it = head; it != tail; ++it)
      {
        if (it->mutex_group_dependency.has_value())
        {
          door_group.push_back(make_wait_for_mutex(
              context, id, it->mutex_group_dependency.value()));
        }

        if (it->phase)
        {
          door_group.push_back(
            [legacy = it->phase, context, id](UpdateFn update)
            {
              return LegacyPhaseShim::Standby::make(
                legacy, context->worker(), context->clock(), id, update);
            });
        }

        if (!it->dependencies.empty())
        {
          door_group.push_back(make_wait_for_traffic(
              context, plan_id, it->dependencies, it->time, id));
        }
      }

      return EventGroupInfo{
        [
          door_group = std::move(door_group),
          group_state = std::move(group_state)
        ](UpdateFn update)
        {
          return rmf_task_sequence::events::Bundle::standby(
            rmf_task_sequence::events::Bundle::Type::Sequence,
            door_group, group_state, update);
        },
        tail
      };
    }
    else if (const auto* move = dynamic_cast<const Move*>(tail_event))
    {
      moving_duration += move->estimate_phase_duration();
      if (std::chrono::minutes(1) < moving_duration)
      {
        // There is a lot of moving happening here, so we should not lump
        // this all together
        return std::nullopt;
      }
    }
    else
    {
      // If any other type of event is happening, we should not lump this
      // all together
      return std::nullopt;
    }

    ++tail;
  }

  return std::nullopt;
}

//==============================================================================
std::optional<EventGroupInfo> search_for_lift_group(
  LegacyPhases::const_iterator head,
  LegacyPhases::const_iterator end,
  const agv::RobotContextPtr& context,
  const PlanIdPtr plan_id,
  const rmf_task::Event::AssignIDPtr& event_id,
  const rmf_task::events::SimpleEventStatePtr& state)
{
  const auto lift_begin = dynamic_cast<const RequestLift*>(head->phase.get());

  if (!lift_begin)
    return std::nullopt;

  const auto& lift_name = lift_begin->lift_name();
  auto tail = head;
  ++tail;
  while (tail != end)
  {
    const auto* tail_event = tail->phase.get();
    if (const auto* lift_request = dynamic_cast<const RequestLift*>(tail_event))
    {
      if (lift_request->lift_name() != lift_name)
      {
        // A different lift is being interacted with before the current lift
        // interaction has finished. This is weird, so let's report it.
        state->update_log().warn(
          "Plan involves using [lift:" + lift_request->lift_name()
          + "] while the robot is already in a session with [lift:"
          + lift_name + "]. This may indicate a broken navigation graph. "
          "Please report this to the system integrator.");
        return std::nullopt;
      }
    }
    else if (const auto* lift_end = dynamic_cast<const EndLift*>(tail_event))
    {
      if (lift_end->lift_name() != lift_name)
      {
        // A different lift session is being ended before this one. This is
        // weird, so let's report it.
        state->update_log().warn(
          "Plan involves ending a session with [lift:" + lift_end->lift_name()
          + "] while [lift:" + lift_name + "] is in use. This may indicate "
          "a broken navigation graph. Please report this to the system "
          "integrator.");
        return std::nullopt;
      }

      auto category = "Take [lift:" + lift_name
        + "] to [floor:" + lift_end->destination() + "]";

      auto group_state = rmf_task::events::SimpleEventState::make(
        event_id->assign(), std::move(category),
        "", rmf_task::Event::Status::Standby, {}, context->clock());

      std::vector<MakeStandby> lift_group;
      ++tail;
      for (auto it = head; it != tail; ++it)
      {
        if (it->mutex_group_dependency.has_value())
        {
          lift_group.push_back(make_wait_for_mutex(
              context, event_id, it->mutex_group_dependency.value()));
        }

        if (it->phase)
        {
          lift_group.push_back(
            [legacy = it->phase, context, event_id](UpdateFn update)
            {
              return LegacyPhaseShim::Standby::make(
                legacy, context->worker(), context->clock(), event_id, update);
            });
        }

        if (!it->dependencies.empty())
        {
          lift_group.push_back(make_wait_for_traffic(
              context, plan_id, it->dependencies, it->time, event_id));
        }
      }

      return EventGroupInfo{
        [
          lift_group = std::move(lift_group),
          group_state = std::move(group_state)
        ](UpdateFn update)
        {
          return rmf_task_sequence::events::Bundle::standby(
            rmf_task_sequence::events::Bundle::Type::Sequence,
            lift_group, group_state, update);
        },
        tail
      };
    }

    ++tail;
  }

  if (tail == end)
  {
    state->update_log().warn(
      "Plan neglects to end a session with [lift:" + lift_name + "]. This may "
      "indicate a broken navigation graph. Please report this to the system "
      "integrator.");
  }

  return std::nullopt;
}

} // anonymous namespace

//==============================================================================
class LiftFinder : public rmf_traffic::agv::Graph::Lane::Executor
{
public:
  LiftFinder(std::string current_name_)
  : current_name(std::move(current_name_)),
    still_using(false)
  {
    // Do nothing
  }
  std::string current_name;
  bool still_using = false;

  void execute(const Dock& dock) final {}
  void execute(const Wait&) final {}
  void execute(const DoorOpen&) final {}
  void execute(const DoorClose&) final {}
  void execute(const LiftSessionBegin& e) final
  {
    // If we're going to re-begin using a lift, then we don't need to keep this
    // one locked. The new LiftSessionBegin event will re-lock the session.
    if (e.lift_name() == current_name)
      still_using = true;
  }
  void execute(const LiftMove& e) final
  {
    if (e.lift_name() == current_name)
      still_using = true;
  }
  void execute(const LiftDoorOpen& e) final
  {
    if (e.lift_name() == current_name)
      still_using = true;
  }
  void execute(const LiftSessionEnd& e) final
  {
    if (e.lift_name() == current_name)
      still_using = true;
  }
};

//==============================================================================
class DoorFinder : public rmf_traffic::agv::Graph::Lane::Executor
{
public:
  DoorFinder(std::string current_name_)
  : current_name(std::move(current_name_)),
    still_using(false)
  {
    // Do nothing
  }
  std::string current_name;
  bool still_using;

  void execute(const Dock& dock) final {}
  void execute(const Wait&) final {}
  void execute(const DoorOpen& open) final
  {
    if (open.name() == current_name)
      still_using = true;
  }
  void execute(const DoorClose& close) final
  {
    if (close.name() == current_name)
      still_using = true;
  }
  void execute(const LiftSessionBegin& e) final {}
  void execute(const LiftMove& e) final {}
  void execute(const LiftDoorOpen& e) final {}
  void execute(const LiftSessionEnd& e) final {}
};

//==============================================================================
std::optional<ExecutePlan> ExecutePlan::make(
  agv::RobotContextPtr context,
  rmf_traffic::PlanId recommended_plan_id,
  rmf_traffic::agv::Plan plan,
  rmf_traffic::agv::Plan::Goal goal,
  rmf_traffic::schedule::Itinerary full_itinerary,
  const rmf_task::Event::AssignIDPtr& event_id,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished,
  std::optional<rmf_traffic::Duration> tail_period)
{
  if (plan.get_waypoints().empty())
    return std::nullopt;

  auto plan_id = std::make_shared<rmf_traffic::PlanId>(recommended_plan_id);
  const auto& graph = context->navigation_graph();
  LegacyPhases legacy_phases;

  rmf_traffic::agv::Graph::LiftPropertiesPtr release_lift;
  const auto envelope = context->profile()->footprint()
    ->get_characteristic_length()/2.0;
  if (const auto* current_lift = context->current_lift_destination())
  {
    LiftFinder finder(current_lift->lift_name);
    for (const auto& wp : plan.get_waypoints())
    {
      if (wp.event())
      {
        wp.event()->execute(finder);
        if (finder.still_using)
          break;
      }
    }

    if (!finder.still_using)
    {
      const auto found_lift = graph.find_known_lift(current_lift->lift_name);
      if (found_lift)
      {
        RCLCPP_INFO(
          context->node()->get_logger(),
          "Robot [%s] will release lift [%s] after a replan because it is no "
          "longer needed.",
          context->requester_id().c_str(),
          current_lift->lift_name.c_str());

        release_lift = found_lift;
      }
      else
      {
        std::stringstream ss;
        for (const auto& l : graph.all_known_lifts())
        {
          ss << "[" << l->name() << "]";
        }

        RCLCPP_ERROR(
          context->node()->get_logger(),
          "Robot [%s] might be stuck with locking an unknown lift named [%s]. "
          "Known lifts include %s",
          context->requester_id().c_str(),
          current_lift->lift_name.c_str(),
          ss.str().c_str());
      }
    }
  }

  if (!plan.get_waypoints().front().graph_index().has_value())
  {
    const Eigen::Vector2d p0 =
      plan.get_waypoints().front().position().block<2, 1>(0, 0);
    const auto t0 = plan.get_waypoints().front().time();

    const auto first_graph_wp = [&]() -> std::optional<std::size_t>
      {
        for (const auto& wp : plan.get_waypoints())
        {
          if (wp.graph_index().has_value())
            return *wp.graph_index();
        }

        return std::nullopt;
      }();

    if (first_graph_wp.has_value())
    {
      const auto& wp = graph.get_waypoint(*first_graph_wp);
      const Eigen::Vector2d p1 = wp.get_location();
      const auto& map = wp.get_map_name();

      // Check if the line from the start of the plan to this waypoint crosses
      // through a door, and add a DoorOpen phase if it does
      for (const auto& door : graph.all_known_doors())
      {
        if (door->map() != map)
        {
          continue;
        }

        if (door->intersects(p0, p1, envelope))
        {
          RCLCPP_INFO(
            context->node()->get_logger(),
            "Ensuring door [%s] is open for [%s] after a replan",
            door->name().c_str(),
            context->requester_id().c_str());

          legacy_phases.emplace_back(
            std::make_shared<phases::DoorOpen::PendingPhase>(
              context, door->name(), context->requester_id(), t0),
            t0, rmf_traffic::Dependencies(), std::nullopt);
        }
      }

      // Check if the robot is going into a lift and summon the lift
      for (const auto& lift : graph.all_known_lifts())
      {
        if (lift->is_in_lift(p1, envelope))
        {
          auto side = phases::RequestLift::Located::Outside;
          if (lift->is_in_lift(p0, envelope))
          {
            side = phases::RequestLift::Located::Inside;
          }

          RCLCPP_INFO(
            context->node()->get_logger(),
            "Robot [%s] will summon lift [%s] to floor [%s] after a replan",
            context->requester_id().c_str(),
            lift->name().c_str(),
            map.c_str());

          legacy_phases.emplace_back(
            std::make_shared<phases::RequestLift::PendingPhase>(
              context, lift->name(), map,
              phases::RequestLift::Data{t0, side, plan_id}),
            t0, rmf_traffic::Dependencies(), std::nullopt);
        }
      }
    }
  }

  std::optional<std::string> release_door;
  if (context->holding_door().has_value())
  {
    const auto& current_door = *context->holding_door();
    DoorFinder finder(current_door);
    for (const auto& wp : plan.get_waypoints())
    {
      if (wp.event())
      {
        wp.event()->execute(finder);
        if (finder.still_using)
          break;
      }
    }

    if (!finder.still_using)
    {
      RCLCPP_INFO(
        context->node()->get_logger(),
        "Robot [%s] will release door [%s] after a replan because it is no "
        "longer needed.",
        context->requester_id().c_str(),
        current_door.c_str());
      release_door = current_door;
    }
  }

  auto initial_itinerary =
    std::make_shared<rmf_traffic::schedule::Itinerary>(full_itinerary);
  auto previous_itinerary = initial_itinerary;


  std::optional<rmf_traffic::Time> finish_time_estimate;
  for (const auto& r : plan.get_itinerary())
  {
    const auto check = r.trajectory().back().time();
    if (!finish_time_estimate.has_value() || *finish_time_estimate < check)
      finish_time_estimate = check;
  }

  if (!finish_time_estimate.has_value())
  {
    // If this is empty then the entire plan is empty... that's not supposed to
    // happen...
    return std::nullopt;
  }

  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints =
    plan.get_waypoints();

  std::vector<rmf_traffic::agv::Plan::Waypoint> move_through;
  std::optional<LockMutexGroup::Data> current_mutex_groups;
  std::unordered_set<std::string> remaining_mutex_groups;

  const auto make_current_mutex_groups = [&](
    const std::unordered_set<std::string>& new_mutex_groups,
    const rmf_traffic::agv::Plan::Waypoint& wp)
    {
      remaining_mutex_groups = new_mutex_groups;
      const rmf_traffic::Time hold_time = wp.time();
      const Eigen::Vector3d hold_position = wp.position();
      std::string hold_map;
      if (wp.graph_index().has_value())
      {
        hold_map =
          graph.get_waypoint(*wp.graph_index()).get_map_name();
      }
      else
      {
        // Find the map name of the first waypoint that is on the graph
        for (const auto& wp : waypoints)
        {
          if (wp.graph_index().has_value())
          {
            hold_map =
              graph.get_waypoint(*wp.graph_index()).get_map_name();
            break;
          }
        }

        if (hold_map.empty())
        {
          RCLCPP_ERROR(
            context->node()->get_logger(),
            "Cannot find a map for a mutex group transition needed "
            "by robot [%s]. There are [%lu] remaining waypoints. Please "
            "report this situation to the maintainers of RMF.",
            context->requester_id().c_str(),
            waypoints.size());
        }
      }

      truncate_arrival(*previous_itinerary, wp);

      auto expected_waypoints = waypoints;
      expected_waypoints.insert(expected_waypoints.begin(), wp);

      auto next_itinerary = std::make_shared<
        rmf_traffic::schedule::Itinerary>(full_itinerary);
      auto data = LockMutexGroup::Data{
        new_mutex_groups,
        hold_map,
        hold_position,
        hold_time,
        plan_id,
        next_itinerary,
        expected_waypoints,
        goal
      };

      previous_itinerary = data.resume_itinerary;

      return data;
    };

  const auto get_new_mutex_groups = [&](
    const rmf_traffic::agv::Plan::Waypoint& wp)
    {
      std::unordered_set<std::string> new_mutex_groups;
      if (wp.graph_index().has_value())
      {
        const auto& group =
          graph.get_waypoint(*wp.graph_index()).in_mutex_group();
        if (!group.empty())
        {
          new_mutex_groups.insert(group);
        }
      }

      for (const auto l : wp.approach_lanes())
      {
        const auto& lane = graph.get_lane(l);
        const auto& group = lane.properties().in_mutex_group();
        if (!group.empty())
        {
          new_mutex_groups.insert(group);
          break;
        }
      }

      bool mutex_group_change =
        (!new_mutex_groups.empty() && remaining_mutex_groups.empty());

      if (!mutex_group_change && !remaining_mutex_groups.empty())
      {
        for (const auto& new_group : new_mutex_groups)
        {
          if (remaining_mutex_groups.count(new_group) == 0)
          {
            mutex_group_change = true;
            break;
          }
        }
      }

      if (!mutex_group_change)
      {
        // We don't need to lock any new mutex groups, but we may be releasing
        // some. We should reduce the remaining group set to whatever is in the
        // new group set so that if we need to add some groups back later then
        // we recognize it.
        remaining_mutex_groups = new_mutex_groups;
      }

      return std::make_pair(mutex_group_change, new_mutex_groups);
    };

  while (!waypoints.empty())
  {
    auto it = waypoints.begin();
    bool event_occurred = false;
    for (; it != waypoints.end(); ++it)
    {
      const auto [mutex_group_change, new_mutex_groups] = get_new_mutex_groups(
        *it);
      if (mutex_group_change)
      {
        if (move_through.size() > 1)
        {
          auto next_mutex_group = make_current_mutex_groups(
            new_mutex_groups, move_through.back());

          legacy_phases.emplace_back(
            std::make_shared<phases::MoveRobot::PendingPhase>(
              context, move_through, plan_id, tail_period),
            move_through.back().time(), move_through.back().dependencies(),
            current_mutex_groups);

          auto last = move_through.back();
          move_through.clear();
          // Repeat the last waypoint so that follow_new_path has continuity.
          move_through.push_back(last);
          waypoints.erase(waypoints.begin(), it);

          current_mutex_groups = next_mutex_group;

          // We treat this the same as an event occurring to indicate that
          // we should keep looping.
          event_occurred = true;
          break;
        }
        else
        {
          // We don't need to put in a break because nothing has been moved
          // through yet. Just set the current_mutex_groups from this point.
          if (move_through.empty())
          {
            current_mutex_groups = make_current_mutex_groups(
              new_mutex_groups, *it);
          }
          else
          {
            assert(move_through.size() == 1);
            current_mutex_groups = make_current_mutex_groups(
              new_mutex_groups, move_through.back());
          }
        }
      }

      move_through.push_back(*it);
      const bool release_lift_here = release_lift &&
        it->graph_index().has_value() &&
        !release_lift->is_in_lift(it->position().block<2, 1>(0, 0), envelope);

      const bool release_door_here = release_door.has_value()
        && it->graph_index().has_value();

      if (it->event() || release_lift_here || release_door_here)
      {
        if (move_through.size() > 1)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::MoveRobot::PendingPhase>(
              context, move_through, plan_id, tail_period),
            it->time(), it->dependencies(), current_mutex_groups);
        }
        else if (current_mutex_groups.has_value())
        {
          legacy_phases.emplace_back(
            nullptr, it->time(),
            rmf_traffic::Dependencies(), current_mutex_groups);
        }
        current_mutex_groups = std::nullopt;

        if (release_lift_here)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::EndLiftSession::Pending>(
              context,
              release_lift->name(),
              ""),
            it->time(), rmf_traffic::Dependencies(), std::nullopt);

          release_lift = nullptr;
        }

        if (release_door_here)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::DoorClose::PendingPhase>(
              context,
              *release_door,
              context->requester_id()),
            it->time(), rmf_traffic::Dependencies(), std::nullopt);

          release_door = std::nullopt;
        }

        std::optional<rmf_traffic::agv::Plan::Waypoint> next_waypoint;
        auto next_it = it + 1;
        if (next_it != waypoints.end())
        {
          next_waypoint = *next_it;
        }

        move_through.clear();
        bool continuous = true;
        if (it->event())
        {
          EventPhaseFactory factory(
            context, legacy_phases, *it, next_waypoint, plan_id,
            make_current_mutex_groups, get_new_mutex_groups,
            previous_itinerary, full_itinerary,
            continuous);

          it->event()->execute(factory);
          while (factory.moving_lift())
          {
            const auto last_it = it;
            ++it;
            if (it == waypoints.end())
            {
              // This should not happen... this would imply that the goal was
              // inside of a lift
              return std::nullopt;
            }

            if (!it->event())
            {
              const double dist =
                (it->position().block<2, 1>(0, 0)
                - last_it->position().block<2, 1>(0, 0)).norm();

              if (dist > 0.5)
              {
                // We'll assume that this is just a misalignment in the maps
                state->update_log().warn(
                  "Plan involves a translation of [" + std::to_string(
                    dist)
                  + "m] while inside a lift. This may indicate an error in the "
                  "navigation graph. Please report this to the system integrator.");
              }

              continue;
            }

            factory.initial_waypoint = *it;
            it->event()->execute(factory);
          }
        }

        if (continuous)
        {
          // Have the next sequence of waypoints begin with the event waypoint
          // of this sequence.
          move_through.push_back(*it);
        }

        waypoints.erase(waypoints.begin(), it+1);
        event_occurred = true;
        break;
      }
      else if (!it->dependencies().empty())
      {
        if (move_through.size() > 1)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::MoveRobot::PendingPhase>(
              context, move_through, plan_id, tail_period),
            it->time(), it->dependencies(), current_mutex_groups);
        }
        else
        {
          legacy_phases.emplace_back(
            nullptr, it->time(), it->dependencies(), current_mutex_groups);
        }

        // Have the next sequence of waypoints begin with this one.
        move_through.clear();
        move_through.push_back(*it);

        waypoints.erase(waypoints.begin(), it+1);
        event_occurred = true;
        break;
      }
    }

    if (move_through.size() > 1)
    {
      // If we have more than one waypoint to move through, then create a
      // moving phase.
      //
      // If we reach this point in the code and move_through is greater than 1,
      // then we have reached the end of the path, so there is definitely no
      // need for any dependencies.
      legacy_phases.emplace_back(
        std::make_shared<phases::MoveRobot::PendingPhase>(
          context, move_through, plan_id, tail_period),
        finish_time_estimate.value(), rmf_traffic::Dependencies{},
        current_mutex_groups);
    }

    if (!event_occurred)
    {
      // If no event occurred on this loop, then we have reached the end of the
      // waypoint sequence, and we should simply clear it out.
      waypoints.clear();
    }
  }

  // Convert the legacy phases into task events.

  // We take the extra step of lumping related events into groups when we can
  // manage to identify such groups, e.g. passing through a door or taking a
  // lift.
  std::vector<MakeStandby> standbys;
  auto head = legacy_phases.cbegin();
  const auto end = legacy_phases.cend();
  while (head != end)
  {
    if (const auto door =
      search_for_door_group(head, end, context, plan_id, event_id))
    {
      standbys.push_back(door->group);
      head = door->tail;
    }
    else if (const auto lift = search_for_lift_group(
        head, end, context, plan_id, event_id, state))
    {
      standbys.push_back(lift->group);
      head = lift->tail;
    }
    else
    {
      if (head->mutex_group_dependency.has_value())
      {
        standbys.push_back(make_wait_for_mutex(
            context, event_id, head->mutex_group_dependency.value()));
      }

      if (head->phase)
      {
        standbys.push_back(
          [legacy = head->phase, context, event_id](UpdateFn update)
          {
            return LegacyPhaseShim::Standby::make(
              legacy, context->worker(), context->clock(), event_id, update);
          });
      }

      if (!head->dependencies.empty())
      {
        standbys.push_back(make_wait_for_traffic(
            context, plan_id, head->dependencies, head->time, event_id));
      }

      ++head;
    }
  }

  if (tail_period.has_value() && !legacy_phases.empty())
  {
    // A tail period was requested, so this is actually a ResponsiveWait action.
    // We will ensure that the task doesn't finish until the final time is
    // reached, even if the robot arrives at the final destination early.
    const auto wait_until_time = legacy_phases.back().time;
    standbys.push_back(
      [context, wait_until_time, event_id](UpdateFn update)
      -> rmf_task_sequence::Event::StandbyPtr
      {
        return WaitUntil::Standby::make(
          context, wait_until_time, event_id, std::move(update));
      });
  }

  std::size_t attempts = 0;
  if (!initial_itinerary->empty())
  {
    while (!context->itinerary().set(*plan_id, *initial_itinerary))
    {
      // Some mysterious behavior has been happening where plan_ids are invalid.
      // We will attempt to catch that here and try to learn more about what
      // could be causing that, while allowing progress to continue.
      std::string task_id = "<none>";
      if (context->current_task_id())
        task_id = *context->current_task_id();

      RCLCPP_ERROR(
        context->node()->get_logger(),
        "Failed to set plan_id [%lu] when current plan_id is [%lu] for [%s] in "
        "group [%s] while executing plan for task [%s]. Please report this bug "
        "to the RMF maintainers.",
        *plan_id,
        context->itinerary().current_plan_id(),
        context->name().c_str(),
        context->group().c_str(),
        task_id.c_str());
      state->update_log().error(
        "Invalid plan_id [" + std::to_string(*plan_id)
        + "] when current plan_id is ["
        + std::to_string(context->itinerary().current_plan_id())
        + "] Please notify an RMF developer.");

      *plan_id = context->itinerary().assign_plan_id();

      if (++attempts > 5)
      {
        std::stringstream ss_sizes;
        for (const auto& r : *initial_itinerary)
        {
          ss_sizes << "[" << r.map() << ":" << r.trajectory().size() << "]";
        }

        RCLCPP_ERROR(
          context->node()->get_logger(),
          "Requesting replan for [%s] in group [%s] because plan is repeatedly "
          "being rejected while performing task [%s]. Itinerary has [%lu] "
          "with sizes of %s.",
          context->name().c_str(),
          context->group().c_str(),
          task_id.c_str(),
          initial_itinerary->size(),
          ss_sizes.str().c_str());
        return std::nullopt;
      }
    }
  }

  auto sequence = rmf_task_sequence::events::Bundle::standby(
    rmf_task_sequence::events::Bundle::Type::Sequence,
    standbys, state, std::move(update))->begin([]() {}, std::move(finished));

  return ExecutePlan{
    std::move(plan),
    plan_id,
    finish_time_estimate.value(),
    std::move(sequence)
  };
}

} // namespace events
} // namespace rmf_fleet_adapter
