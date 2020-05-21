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

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto GoToPlace::Active::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_publisher.observe();
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
  const Responder& responder,
  const bool *)
{
  // TODO(MXG): Create a job that responds to the negotiation
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
  _description = "Moving to [" + std::to_string(_goal.waypoint()) + "]";
  _negotiator_subscription = _context->set_negotiator(this);

  StatusMsg initial_msg;
  initial_msg.status =
      "Planning a move to [" + std::to_string(_goal.waypoint()) + "]";
  const auto now = _context->node().now();
  initial_msg.start_time = now;
  initial_msg.end_time = now + rclcpp::Duration(_latest_time_estimate);
  _status_publisher.publish(initial_msg);

  find_plan();
}

//==============================================================================
void GoToPlace::Active::find_plan()
{
  // TODO(MXG): Create a job to find a plan
}

//==============================================================================
void GoToPlace::Active::find_emergency_plan()
{
  // TODO(MXG): Create a job to find an emergency plan
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> GoToPlace::Pending::begin()
{
  return std::shared_ptr<Task::ActivePhase>(
        new Active(_context, _goal, _time_estimate));
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
  _description = "Moving to [" + std::to_string(_goal.waypoint()) + "]";
}

//==============================================================================
auto GoToPlace::make(
    agv::RobotContextPtr context,
    rmf_traffic::agv::Plan::Start start_estimate,
    rmf_traffic::agv::Plan::Goal goal) -> std::unique_ptr<Pending>
{
  auto estimate_options = context->planner().get_default_options();
  estimate_options.validator(nullptr);

  auto estimate = context->planner().setup(
        start_estimate, goal, estimate_options);

  if (!estimate.cost_estimate())
  {
    RCLCPP_ERROR(
          context->node().get_logger(),
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
