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

#include "../phases/MoveRobot.hpp"
#include "../phases/DoorOpen.hpp"
#include "../phases/DoorClose.hpp"
#include "../phases/RequestLift.hpp"
#include "../phases/DockRobot.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>

namespace rmf_fleet_adapter {
namespace events {

namespace {
//==============================================================================
using StandbyPtr = rmf_task_sequence::Event::StandbyPtr;
using UpdateFn = std::function<void()>;
using MakeStandby = std::function<StandbyPtr(UpdateFn)>;

//==============================================================================
struct LegacyPhaseWrapper
{
  LegacyPhaseWrapper(
    std::shared_ptr<LegacyTask::PendingPhase> phase_,
    rmf_traffic::Time time_,
    rmf_traffic::Dependencies dependencies_)
  : phase(std::move(phase_)),
    time(time_),
    dependencies(std::move(dependencies_))
  {
    // Do nothing
  }

  std::shared_ptr<LegacyTask::PendingPhase> phase;
  rmf_traffic::Time time;
  rmf_traffic::Dependencies dependencies;
};

using LegacyPhases = std::vector<LegacyPhaseWrapper>;

using DockRobot = phases::DockRobot::PendingPhase;
using DoorOpen = phases::DoorOpen::PendingPhase;
using DoorClose = phases::DoorClose::PendingPhase;
using RequestLift = phases::RequestLift::PendingPhase;
using EndLift = phases::EndLiftSession::Pending;
using Move = phases::MoveRobot::PendingPhase;

//==============================================================================
MakeStandby make_wait_for_traffic(
  const agv::RobotContextPtr& context,
  const rmf_traffic::Dependencies& deps,
  const rmf_traffic::Time time,
  const rmf_task_sequence::Event::AssignIDPtr& id)
{
  return [context, deps, time, id](UpdateFn update)
    {
      return WaitForTraffic::Standby::make(
        context, deps, time, id, std::move(update));
    };
}

//==============================================================================
class EventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;

  EventPhaseFactory(
    agv::RobotContextPtr context,
    LegacyPhases& phases,
    rmf_traffic::Time event_start_time,
    const rmf_traffic::Dependencies& dependencies,
    bool& continuous)
  : _context(std::move(context)),
    _phases(phases),
    _event_start_time(event_start_time),
    _dependencies(dependencies),
    _continuous(continuous)
  {
    // Do nothing
  }

  void execute(const Dock& dock) final
  {
    assert(!_moving_lift);
    _phases.emplace_back(
      std::make_shared<phases::DockRobot::PendingPhase>(
        _context, dock.dock_name()),
      _event_start_time, _dependencies);
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
      _event_start_time, _dependencies);
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
      _event_start_time, _dependencies);
    _continuous = true;
  }

  void execute(const LiftSessionBegin& open) final
  {
    assert(!_moving_lift);
    const auto node = _context->node();
    _phases.emplace_back(
      std::make_shared<phases::RequestLift::PendingPhase>(
        _context,
        open.lift_name(),
        open.floor_name(),
        _event_start_time,
        phases::RequestLift::Located::Outside),
      _event_start_time, _dependencies);

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
    const auto node = _context->node();

    // TODO(MXG): The time calculation here should be considered more carefully.
    _phases.emplace_back(
      std::make_shared<phases::RequestLift::PendingPhase>(
        _context,
        open.lift_name(),
        open.floor_name(),
        _event_start_time + open.duration() + _lifting_duration,
        phases::RequestLift::Located::Inside),
      _event_start_time, _dependencies);
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
      _event_start_time, _dependencies);

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
  const rmf_traffic::Dependencies& _dependencies;
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
        door_group.push_back(
          [legacy = it->phase, context, id](UpdateFn update)
          {
            return LegacyPhaseShim::Standby::make(
              legacy, context->worker(), context->clock(), id, update);
          });

        if (!it->dependencies.empty())
        {
          door_group.push_back(make_wait_for_traffic(
              context, it->dependencies, it->time, id));
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
        lift_group.push_back(
          [legacy = it->phase, context, event_id](UpdateFn update)
          {
            return LegacyPhaseShim::Standby::make(
              legacy, context->worker(), context->clock(), event_id, update);
          });

        if (!it->dependencies.empty())
        {
          lift_group.push_back(make_wait_for_traffic(
              context, it->dependencies, it->time, event_id));
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
std::optional<ExecutePlan> ExecutePlan::make(
  agv::RobotContextPtr context,
  rmf_traffic::PlanId plan_id,
  rmf_traffic::agv::Plan plan,
  rmf_traffic::schedule::Itinerary full_itinerary,
  const rmf_task::Event::AssignIDPtr& event_id,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished,
  std::optional<rmf_traffic::Duration> tail_period)
{
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

  LegacyPhases legacy_phases;
  while (!waypoints.empty())
  {
    auto it = waypoints.begin();
    bool event_occurred = false;
    for (; it != waypoints.end(); ++it)
    {
      move_through.push_back(*it);

      if (it->event())
      {
        if (move_through.size() > 1)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::MoveRobot::PendingPhase>(
              context, move_through, plan_id, tail_period),
            it->time(), it->dependencies());
        }

        move_through.clear();
        bool continuous = true;
        EventPhaseFactory factory(
          context, legacy_phases, it->time(), it->dependencies(), continuous);
        it->event()->execute(factory);
        while (factory.moving_lift())
        {
          const auto last_it = it;
          ++it;
          if (!it->event())
          {
            const double dist =
              (it->position().block<2, 1>(0, 0)
              - last_it->position().block<2, 1>(0, 0)).norm();

            if (dist < 0.5)
            {
              // We'll assume that this is just a misalignment in the maps
              continue;
            }

            state->update_log().warn(
              "Plan involves a translation of [" + std::to_string(dist)
              + "m] while inside a lift. This may indicate an error in the "
              "navigation graph. Please report this to the system integrator.");
          }

          it->event()->execute(factory);
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
            it->time(), it->dependencies());
        }
        else
        {
          legacy_phases.emplace_back(nullptr, it->time(), it->dependencies());
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
        finish_time_estimate.value(), rmf_traffic::Dependencies{});
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
    if (const auto door = search_for_door_group(head, end, context, event_id))
    {
      standbys.push_back(door->group);
      head = door->tail;
    }
    else if (const auto lift = search_for_lift_group(
        head, end, context, event_id, state))
    {
      standbys.push_back(lift->group);
      head = lift->tail;
    }
    else
    {
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
            context, head->dependencies, head->time, event_id));
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

  auto sequence = rmf_task_sequence::events::Bundle::standby(
    rmf_task_sequence::events::Bundle::Type::Sequence,
    standbys, state, std::move(update))->begin([]() {}, std::move(finished));

  std::size_t attempts = 0;
  while (!context->itinerary().set(plan_id, std::move(full_itinerary)))
  {
    // Some mysterious behavior has been happening where plan_ids are invalid.
    // We will attempt to catch that here and try to learn more about what
    // could be causing that, while allowing progress to continue.
    std::string task_id = "<none>";
    if (context->current_task_id())
      task_id = *context->current_task_id();

    RCLCPP_ERROR(
      context->node()->get_logger(),
      "Invalid plan_id [%lu] when current plan_id is [%lu] for [%s] in group "
      "[%s] while performing task [%s]. Please notify an RMF developer.",
      plan_id,
      context->itinerary().current_plan_id(),
      context->name().c_str(),
      context->group().c_str(),
      task_id.c_str());
    state->update_log().error(
      "Invalid plan_id [" + std::to_string(plan_id) + "] when current plan_id "
      "is [" + std::to_string(context->itinerary().current_plan_id()) + "] "
      "Please notify an RMF developer.");

    plan_id = context->itinerary().assign_plan_id();

    if (++attempts > 5)
    {
      RCLCPP_ERROR(
        context->node()->get_logger(),
        "Requesting replan for [%s] in group [%s] because plan is repeatedly "
        "being rejected while performing task [%s]",
        context->name().c_str(),
        context->group().c_str(),
        task_id.c_str());
      return std::nullopt;
    }
  }

  return ExecutePlan{
    std::move(plan),
    finish_time_estimate.value(),
    std::move(sequence)
  };
}

} // namespace events
} // namespace rmf_fleet_adapter
