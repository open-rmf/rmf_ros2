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

#include "GoToPlace.hpp"
#include "MoveRobot.hpp"
#include "DoorOpen.hpp"
#include "DoorClose.hpp"
#include "RequestLift.hpp"
#include "DockRobot.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto GoToPlace::Active::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Active::estimate_remaining_time() const
{
  if (_plan)
  {
    if (_plan->get_itinerary().empty())
      return rmf_traffic::Duration(0);

    const auto& traj = _plan->get_itinerary().back().trajectory();
    if (traj.size() == 0)
    {
      // This shouldn't happen
      assert(false);
      return rmf_traffic::Duration(0);
    }

    const auto t = traj.back().time();
    return t - _context->now();
  }

  return rmf_traffic::time::from_seconds(_latest_time_estimate);
}

//==============================================================================
void GoToPlace::Active::emergency_alarm(const bool on)
{
  if (_emergency_active == on)
    return;

  _emergency_active = on;
  if (_emergency_active)
  {
    cancel();
    find_emergency_plan();
  }
  else
  {
    find_plan();
  }
}
//==============================================================================
void GoToPlace::Active::cancel()
{
  if (_subtasks)
    _subtasks->cancel();
}

//==============================================================================
const std::string& GoToPlace::Active::description() const
{
  return _description;
}

//==============================================================================
void GoToPlace::Active::respond(
  const TableViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  auto approval_cb = [w = weak_from_this()](
      const rmf_traffic::agv::Plan& plan)
      -> rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion>
  {
    if (auto active = w.lock())
    {
      active->execute_plan(plan);
      return active->_context->itinerary().version();
    }

    return rmf_utils::nullopt;
  };

  services::ProgressEvaluator evaluator;
  if (table_viewer->parent_id())
  {
    const auto& s = table_viewer->sequence();
    assert(s.size() >= 2);
    evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
  }

  std::shared_ptr<services::Negotiate> negotiate;
  if (_emergency_active)
  {
    negotiate = services::Negotiate::emergency_pullover(
          _context->planner(), _context->location(), table_viewer, responder,
          std::move(approval_cb), evaluator);
  }
  else
  {
    negotiate = services::Negotiate::path(
          _context->planner(), _context->location(), _goal, table_viewer,
          responder, std::move(approval_cb), evaluator);
  }

  _negotiate_services[negotiate] =
      rmf_rxcpp::make_job<services::Negotiate::Result>(negotiate)
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [w = weak_from_this(), negotiate](const auto& result)
  {
    if (auto phase = w.lock())
    {
      result.respond();
      phase->_negotiate_services.erase(result.service);
    }
  });
}

//==============================================================================
GoToPlace::Active::Active(
  agv::RobotContextPtr context,
  rmf_traffic::agv::Plan::Goal goal,
  double original_time_estimate)
  : _context(std::move(context)),
    _goal(std::move(goal)),
    _latest_time_estimate(original_time_estimate)
{
  _status_obs = _status_publisher.get_observable();

  _description = "Sending [" + _context->requester_id() + "] to ["
      + std::to_string(_goal.waypoint()) + "]";
  _negotiator_license = _context->set_negotiator(this);

  StatusMsg initial_msg;
  initial_msg.status =
      "Planning a move to [" + std::to_string(_goal.waypoint()) + "] for ["
      + _context->requester_id() + "]";
  const auto now = _context->node()->now();
  initial_msg.start_time = now;
  initial_msg.end_time = now + rclcpp::Duration(_latest_time_estimate);
  _status_publisher.get_subscriber().on_next(initial_msg);
}

//==============================================================================
void GoToPlace::Active::find_plan()
{
  if (_emergency_active)
    return find_emergency_plan();

  _pullover_service = nullptr;
  _find_path_service = std::make_shared<services::FindPath>(
        _context->planner(), _context->location(), _goal,
        _context->schedule()->snapshot(), _context->itinerary().id());

  std::cout << "Searching for plan for [" << _context->requester_id() << "]" << std::endl;
  _plan_subscription = rmf_rxcpp::make_job<services::FindPath::Result>(
        _find_path_service)
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [w = weak_from_this(), r = _context->requester_id()](
        const services::FindPath::Result& result)
  {
    const auto phase = w.lock();
    if (!phase)
    {
      std::cout << " !!!! GoToPlace expired for [" << r
                << "] before a plan was found!" << std::endl;
      return;
    }

    if (!result)
    {
      std::cout << " !!!! FAILED TO FIND PLAN FOR [" << phase->_context->requester_id() << "]" << std::endl;
      // This shouldn't happen, but let's try to handle it gracefully
      phase->_status_publisher.get_subscriber().on_error(
            std::make_exception_ptr(std::runtime_error("Cannot find a plan")));

      // TODO(MXG): Instead of canceling, should we retry later?
      phase->_subtasks->cancel();
      return;
    }

    std::cout << "Found plan for [" << r << "]" << std::endl;
    phase->execute_plan(*std::move(result));
  },
      [w = weak_from_this(), r= _context->requester_id()]()
  {
    const auto phase = w.lock();
    if (!phase)
    {
      std::cout << "!!!! GoToPlace expired for [" << r
                << "] before plan completion was triggered!" << std::endl;
      return;
    }

    std::cout << "Plan completion given for [" << r << "]" << std::endl;
  });
}

//==============================================================================
void GoToPlace::Active::find_emergency_plan()
{
  StatusMsg emergency_msg;
  emergency_msg.status = "Planning an emergency pullover for ["
      + _context->requester_id() + "]";
  emergency_msg.start_time = _context->node()->now();
  emergency_msg.end_time = emergency_msg.start_time;
  _status_publisher.get_subscriber().on_next(emergency_msg);

  _find_path_service = nullptr;
  _pullover_service = std::make_shared<services::FindEmergencyPullover>(
        _context->planner(), _context->location(),
        _context->schedule()->snapshot(), _context->itinerary().id());

  _plan_subscription = rmf_rxcpp::make_job<
      services::FindEmergencyPullover::Result>(_pullover_service)
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [w = weak_from_this()](
        const services::FindEmergencyPullover::Result& result)
  {
    const auto phase = w.lock();
    if (!phase)
      return;

    if (!result)
    {
      std::cout << " !!!! FAILED TO FIND EMERGENCY PLAN FOR [" << phase->_context->requester_id() << "]" << std::endl;
      // This shouldn't happen, but let's try to handle it gracefully
      phase->_status_publisher.get_subscriber().on_error(
            std::make_exception_ptr(std::runtime_error("Cannot find a plan")));

      // TODO(MXG): Instead of canceling, should we retry later?
      phase->_subtasks->cancel();
      return;
    }

    phase->execute_plan(*std::move(result));
    phase->_performing_emergency_task = true;
  });
}

namespace {
//==============================================================================
class EventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;

  EventPhaseFactory(
      agv::RobotContextPtr context,
      Task::PendingPhases& phases)
    : _context(std::move(context)),
      _phases(phases)
  {
    // Do nothing
  }

  void execute(const Dock& dock) final
  {
    _phases.push_back(
          std::make_unique<phases::DockRobot::PendingPhase>(
            _context, dock.dock_name()));
  }

  void execute(const DoorOpen& open) final
  {
    const auto node = _context->node();
    _phases.push_back(
          std::make_unique<phases::DoorOpen::PendingPhase>(
            open.name(),
            _context->requester_id(),
            node,
            node->door_state(),
            node->door_supervisor(),
            node->door_request()));
  }

  void execute(const DoorClose& close) final
  {
    const auto node = _context->node();
    _phases.push_back(
          std::make_unique<phases::DoorClose::PendingPhase>(
            close.name(),
            _context->requester_id(),
            node,
            node->door_supervisor(),
            node->door_request()));
  }

  void execute(const LiftDoorOpen& open) final
  {
    const auto node = _context->node();
    _phases.push_back(
          std::make_unique<phases::RequestLift::PendingPhase>(
            _context->requester_id(),
            node,
            open.lift_name(),
            open.floor_name(),
            node->lift_state(),
            node->lift_request()));
  }

  void execute(const LiftDoorClose& /*close*/) final
  {
    // Not supported yet
  }

  void execute(const LiftMove& /*move*/) final
  {
    // Not supported yet
  }

private:
  agv::RobotContextPtr _context;
  Task::PendingPhases& _phases;
};

} // anonymous namespace

//==============================================================================
void GoToPlace::Active::execute_plan(rmf_traffic::agv::Plan new_plan)
{
  _plan = std::move(new_plan);

  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints =
      _plan->get_waypoints();

  Task::PendingPhases sub_phases;
  while (!waypoints.empty())
  {
    std::vector<rmf_traffic::agv::Plan::Waypoint> move_through;
    auto it = waypoints.begin();
    for (; it != waypoints.end(); ++it)
    {
      move_through.push_back(*it);

      if (it->event())
      {
        if (move_through.size() > 1)
        {
          sub_phases.push_back(
              std::make_unique<MoveRobot::PendingPhase>(
                  _context, move_through));
        }

        move_through.clear();

        EventPhaseFactory factory(_context, sub_phases);
        it->event()->execute(factory);

        waypoints.erase(waypoints.begin(), it+1);
        break;
      }
    }

    if (move_through.size() > 1)
    {
      /// If we have more than one waypoint to move through, then create a
      /// moving phase.
      sub_phases.push_back(
          std::make_unique<MoveRobot::PendingPhase>(_context, move_through));
    }

    if (!move_through.empty())
    {
      // If we made it into this if-statement, then we have reached the end of
      // the waypoints, because otherwise an event would have interrupted the
      // for-loop and cleared out the move_through sequence.
      waypoints.clear();
    }
  }

  _subtasks = Task::make(
        _description, std::move(sub_phases), _context->worker());

  _status_subscription = _subtasks->observe()
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [weak = weak_from_this(), r = _context->name()](const StatusMsg& msg)
        {
          if (const auto phase = weak.lock())
          {
            std::cout << "[" << r << "] done with subtask: " << msg.status << std::endl;
            phase->_status_publisher.get_subscriber().on_next(msg);
          }
          else
            std::cout << " 1!!! LOST GoToPlace FOR [" << r << "]" << std::endl;
        },
        [weak = weak_from_this(), r = _context->name()](std::exception_ptr e)
        {
          if (const auto phase = weak.lock())
            phase->_status_publisher.get_subscriber().on_error(e);
          else
            std::cout << " 2!!! LOST GoToPlace FOR [" << r << "]" << std::endl;
        },
        [weak = weak_from_this(), r = _context->name()]()
        {
          if (const auto phase = weak.lock())
          {
            if (!phase->_emergency_active)
            {
              std::cout << "[" << r << "] completed GoToPlace: " << phase->_description << std::endl;
              phase->_status_publisher.get_subscriber().on_completed();
            }

            // If an emergency is active, then eventually the alarm should get
            // turned off, which should trigger a non-emergency replanning. That
            // new plan will create a new set of subtasks, and when that new set
            // of subtasks is complete, then we will consider this GoToPlace
            // phase to be complete.
          }
          else
            std::cout << " 3!!! LOST GoToPlace FOR [" << r << "]" << std::endl;
        }
   );

  _subtasks->begin();
  _context->worker().schedule(
        [c = _context, itinerary = _plan->get_itinerary()](const auto&)
  {
    c->itinerary().set(itinerary);
  });
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> GoToPlace::Pending::begin()
{
  auto active =
      std::shared_ptr<Active>(new Active(_context, _goal, _time_estimate));

  active->find_plan();

  return active;
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Pending::estimate_phase_duration() const
{
  return rmf_traffic::time::from_seconds(_time_estimate);
}

//==============================================================================
const std::string& GoToPlace::Pending::description() const
{
  return _description;
}

//==============================================================================
GoToPlace::Pending::Pending(
  agv::RobotContextPtr context,
  rmf_traffic::agv::Plan::Goal goal,
  double time_estimate)
: _context(std::move(context)),
  _goal(std::move(goal)),
  _time_estimate(time_estimate)
{
  _description = "Send robot to [" + std::to_string(_goal.waypoint()) + "]";
}

//==============================================================================
auto GoToPlace::make(
    agv::RobotContextPtr context,
    rmf_traffic::agv::Plan::Start start_estimate,
    rmf_traffic::agv::Plan::Goal goal) -> std::unique_ptr<Pending>
{
  auto estimate_options = context->planner()->get_default_options();
  estimate_options.validator(nullptr);

  auto estimate = context->planner()->setup(
        start_estimate, goal, estimate_options);

  if (!estimate.cost_estimate())
  {
    RCLCPP_ERROR(
          context->node()->get_logger(),
          "[GoToPlace] Unable to find any path for robot [%s] to get from "
          "waypoint [%d] to waypoint [%d]",
          context->name().c_str(), start_estimate.waypoint(), goal.waypoint());
    return nullptr;
  }

  const double cost = *estimate.cost_estimate();
  return std::unique_ptr<Pending>(
        new Pending(std::move(context), std::move(goal), cost));
}

} // namespace phases
} // namespace rmf_fleet_adapter
