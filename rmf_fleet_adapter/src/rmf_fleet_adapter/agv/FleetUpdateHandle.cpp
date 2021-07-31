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

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include "internal_FleetUpdateHandle.hpp"
#include "internal_RobotUpdateHandle.hpp"
#include "RobotContext.hpp"

#include "../tasks/Delivery.hpp"
#include "../tasks/Loop.hpp"

#include <rmf_task/agv/Constraints.hpp>
#include <rmf_task/agv/Parameters.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_task_msgs/msg/clean.hpp>
#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>

namespace rmf_fleet_adapter {
namespace agv {

namespace {
//==============================================================================
class LiaisonNegotiator : public rmf_traffic::schedule::Negotiator
{
public:

  LiaisonNegotiator(
    std::shared_ptr<rmf_traffic::schedule::Negotiator> negotiator)
  : w_negotiator(negotiator)
  {
    // Do nothing
  }

  std::weak_ptr<rmf_traffic::schedule::Negotiator> w_negotiator;

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final
  {
    const auto negotiator = w_negotiator.lock();
    if (!negotiator)
    {
      // If we no longer have access to the upstream negotiator, then we simply
      // forfeit.
      //
      // TODO(MXG): Consider issuing a warning here
      return responder->forfeit({});
    }

    negotiator->respond(table_viewer, responder);
  }

};
} // anonymous namespace

//==============================================================================
void FleetUpdateHandle::Implementation::dock_summary_cb(
  const DockSummary::SharedPtr& msg)
{
  for (const auto& dock : msg->docks)
  {
    if (dock.fleet_name == name)
    {
      dock_param_map.clear();
      for (const auto& param : dock.params)
        dock_param_map.insert({param.start, param});
      break;
    }
  }

  return;
}

//==============================================================================
void FleetUpdateHandle::Implementation::bid_notice_cb(
  const BidNotice::SharedPtr msg)
{
  if (task_managers.empty())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet [%s] does not have any robots to accept task [%s]. Use "
      "FleetUpdateHadndle::add_robot(~) to add robots to this fleet. ",
      name.c_str(), msg->task_profile.task_id.c_str());
    return;
  }

  if (msg->task_profile.task_id.empty())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Received BidNotice for a task with invalid task_id. Request will be "
      "ignored.");
    return;
  }

  // TODO remove this block when we support task revival
  if (bid_notice_assignments.find(msg->task_profile.task_id)
    != bid_notice_assignments.end())
    return;

  if (!accept_task)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Fleet [%s] is not configured to accept any task requests. Use "
      "FleetUpdateHadndle::accept_task_requests(~) to define a callback "
      "for accepting requests", name.c_str());

    return;
  }

  if (!accept_task(msg->task_profile))
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet [%s] is configured to not accept task [%s]",
      name.c_str(),
      msg->task_profile.task_id.c_str());

    return;
  }

  if (!task_planner)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Fleet [%s] is not configured with parameters for task planning."
      "Use FleetUpdateHandle::set_task_planner_params(~) to set the "
      "parameters required.", name.c_str());

    return;
  }

  // Determine task type and convert to request pointer
  rmf_task::ConstRequestPtr new_request = nullptr;
  const auto& task_profile = msg->task_profile;
  const auto& task_type = task_profile.description.task_type;
  const rmf_traffic::Time start_time =
    rmf_traffic_ros2::convert(task_profile.description.start_time);
  // TODO (YV) get rid of ID field in RequestPtr
  std::string id = msg->task_profile.task_id;
  const auto& graph = (*planner)->get_configuration().graph();

  // Generate the priority of the request. The current implementation supports
  // binary priority
  auto priority =
    task_profile.description.priority.value > 0 ?
    rmf_task::BinaryPriorityScheme::make_high_priority() :
    rmf_task::BinaryPriorityScheme::make_low_priority();

  // Process Cleaning task
  if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_CLEAN)
  {
    if (task_profile.description.clean.start_waypoint.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [clean.start_waypoint] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    // Check for valid start waypoint
    const std::string start_wp_name =
      task_profile.description.clean.start_waypoint;
    const auto start_wp = graph.find_waypoint(start_wp_name);
    if (!start_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), start_wp_name.c_str(), id.c_str());

      return;
    }

    // Get dock parameters
    const auto clean_param_it = dock_param_map.find(start_wp_name);
    if (clean_param_it == dock_param_map.end())
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Dock param for dock_name:[%s] unavailable. Rejecting BidNotice with "
        "task_id:[%s]", start_wp_name.c_str(), id.c_str());

      return;
    }
    const auto& clean_param = clean_param_it->second;

    // Check for valid finish waypoint
    const std::string& finish_wp_name = clean_param.finish;
    const auto finish_wp = graph.find_waypoint(finish_wp_name);
    if (!finish_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), finish_wp_name.c_str(), id.c_str());

      return;
    }

    // Interpolate docking waypoint into trajectory
    std::vector<Eigen::Vector3d> positions;
    for (const auto& location: clean_param.path)
      positions.push_back({location.x, location.y, location.yaw});
    rmf_traffic::Trajectory cleaning_trajectory =
      rmf_traffic::agv::Interpolate::positions(
      (*planner)->get_configuration().vehicle_traits(),
      start_time,
      positions);

    if (cleaning_trajectory.size() == 0)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Unable to generate cleaning trajectory from positions specified "
        " in DockSummary msg for [%s]", start_wp_name.c_str());

      return;
    }

    new_request = rmf_task::requests::Clean::make(
      start_wp->index(),
      finish_wp->index(),
      cleaning_trajectory,
      id,
      start_time,
      priority);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Clean request for task_id:[%s]", id.c_str());
  }

  else if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_DELIVERY)
  {
    const auto& delivery = task_profile.description.delivery;
    if (delivery.pickup_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.pickup_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (delivery.pickup_dispenser.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.pickup_dispenser] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (delivery.dropoff_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (delivery.dropoff_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (delivery.dropoff_ingestor.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_ingestor] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    const auto pickup_wp = graph.find_waypoint(delivery.pickup_place_name);
    if (!pickup_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), delivery.pickup_place_name.c_str(), id.c_str());

      return;
    }

    const auto dropoff_wp = graph.find_waypoint(delivery.dropoff_place_name);
    if (!dropoff_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), delivery.dropoff_place_name.c_str(), id.c_str());

      return;
    }

    new_request = rmf_task::requests::Delivery::make(
      pickup_wp->index(),
      delivery.pickup_dispenser,
      dropoff_wp->index(),
      delivery.dropoff_ingestor,
      delivery.items,
      id,
      start_time,
      priority);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Delivery request for task_id:[%s]", id.c_str());

  }
  else if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_LOOP)
  {
    const auto& loop = task_profile.description.loop;
    if (loop.start_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.start_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (loop.finish_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.finish_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]", id.c_str());

      return;
    }

    if (loop.num_loops < 1)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.num_loops: %d] in TaskProfile is invalid."
        "Rejecting BidNotice with task_id:[%s]", loop.num_loops, id.c_str());

      return;
    }

    const auto start_wp = graph.find_waypoint(loop.start_name);
    if (!start_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), loop.start_name.c_str(), id.c_str());

      return;
    }

    const auto finish_wp = graph.find_waypoint(loop.finish_name);
    if (!finish_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), loop.finish_name.c_str(), id.c_str());

      return;
    }

    new_request = rmf_task::requests::Loop::make(
      start_wp->index(),
      finish_wp->index(),
      loop.num_loops,
      id,
      start_time,
      priority);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Loop request for task_id:[%s]", id.c_str());
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid TaskType [%d] in TaskProfile. Rejecting BidNotice with "
      "task_id:[%s]",
      task_type.type, id.c_str());

    return;
  }

  if (!new_request)
    return;
  generated_requests.insert({id, new_request});
  task_profile_map.insert({id, task_profile});

  const auto allocation_result = allocate_tasks(new_request);

  if (!allocation_result.has_value())
    return;

  const auto& assignments = allocation_result.value();

  const double cost = task_planner->compute_cost(assignments);

  // Display computed assignments for debugging
  std::stringstream debug_stream;
  debug_stream << "Cost: " << cost << std::endl;
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    debug_stream << "--Agent: " << i << std::endl;
    for (const auto& a : assignments[i])
    {
      const auto& s = a.state();
      const double request_seconds =
        a.request()->earliest_start_time().time_since_epoch().count()/1e9;
      const double start_seconds =
        a.deployment_time().time_since_epoch().count()/1e9;
      const rmf_traffic::Time finish_time = s.finish_time();
      const double finish_seconds = finish_time.time_since_epoch().count()/1e9;
      debug_stream << "    <" << a.request()->id() << ": " << request_seconds
                   << ", " << start_seconds
                   << ", "<< finish_seconds << ", " << 100* s.battery_soc()
                   << "%>" << std::endl;
    }
  }
  debug_stream << " ----------------------" << std::endl;

  RCLCPP_DEBUG(node->get_logger(), "%s", debug_stream.str().c_str());

  // Publish BidProposal
  rmf_task_msgs::msg::BidProposal bid_proposal;
  bid_proposal.fleet_name = name;
  bid_proposal.task_profile = task_profile;
  bid_proposal.prev_cost = current_assignment_cost;
  bid_proposal.new_cost = cost;

  // Map robot index to name to populate robot_name in BidProposal
  std::unordered_map<std::size_t, std::string> robot_name_map;
  std::size_t index = 0;
  for (const auto& t : task_managers)
  {
    robot_name_map.insert({index, t.first->name()});
    ++index;
  }

  index = 0;
  for (const auto& agent : assignments)
  {
    for (const auto& assignment : agent)
    {
      if (assignment.request()->id() == id)
      {
        bid_proposal.finish_time = rmf_traffic_ros2::convert(
          assignment.state().finish_time());
        if (robot_name_map.find(index) != robot_name_map.end())
          bid_proposal.robot_name = robot_name_map[index];
        break;
      }
    }
    ++index;
  }

  bid_proposal_pub->publish(bid_proposal);
  RCLCPP_INFO(
    node->get_logger(),
    "Submitted BidProposal to accommodate task [%s] by robot [%s] with new cost [%f]",
    id.c_str(), bid_proposal.robot_name.c_str(), cost);

  // Store assignments in internal map
  bid_notice_assignments.insert({id, assignments});

}

//==============================================================================
void FleetUpdateHandle::Implementation::dispatch_request_cb(
  const DispatchRequest::SharedPtr msg)
{
  if (msg->fleet_name != name)
    return;

  const std::string id = msg->task_profile.task_id;
  DispatchAck dispatch_ack;
  dispatch_ack.dispatch_request = *msg;
  dispatch_ack.success = false;

  if (msg->method == DispatchRequest::ADD)
  {
    const auto task_it = bid_notice_assignments.find(id);
    if (task_it == bid_notice_assignments.end())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Received DispatchRequest for task_id:[%s] before receiving BidNotice. "
        "This request will be ignored.",
        id.c_str());
      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Bid for task_id:[%s] awarded to fleet [%s]. Processing request...",
      id.c_str(),
      name.c_str());

    auto& assignments = task_it->second;

    if (assignments.size() != task_managers.size())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "The number of available robots does not match that in the assignments "
        "for task_id:[%s]. This request will be ignored.",
        id.c_str());
      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    // Here we make sure none of the tasks in the assignments has already begun
    // execution. If so, we replan assignments until a valid set is obtained
    // and only then update the task manager queues
    const auto request_it = generated_requests.find(id);
    if (request_it == generated_requests.end())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Unable to find generated request for task_id:[%s]. This request will "
        "be ignored.",
        id.c_str());
      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    bool valid_assignments = is_valid_assignments(assignments);
    if (!valid_assignments)
    {
      // TODO: This replanning is blocking the main thread. Instead, the
      // replanning should run on a separate worker and then deliver the
      // result back to the main worker.
      const auto replan_results = allocate_tasks(request_it->second);
      if (!replan_results)
      {
        RCLCPP_WARN(
          node->get_logger(),
          "Unable to replan assignments when accommodating task_id:[%s]. This "
          "request will be ignored.",
          id.c_str());
        dispatch_ack_pub->publish(dispatch_ack);
        return;
      }
      assignments = replan_results.value();
      // We do not need to re-check if assignments are valid as this function
      // is being called by the ROS2 executor and is running on the main
      // rxcpp worker. Hence, no new tasks would have started during this replanning.
    }

    std::size_t index = 0;
    for (auto& t : task_managers)
    {
      t.second->set_queue(assignments[index], task_profile_map);
      ++index;
    }

    current_assignment_cost = task_planner->compute_cost(assignments);
    assigned_requests.insert({id, request_it->second});
    dispatch_ack.success = true;
    dispatch_ack_pub->publish(dispatch_ack);

    RCLCPP_INFO(
      node->get_logger(),
      "Assignments updated for robots in fleet [%s] to accommodate task_id:[%s]",
      name.c_str(), id.c_str());
  }

  else if (msg->method == DispatchRequest::CANCEL)
  {
    // We currently only support cancellation of a queued task.
    // TODO: Support cancellation of an active task.

    // When a queued task is to be cancelled, we simply re-plan and re-allocate
    // task assignments for the request set containing all the queued tasks
    // excluding the task to be cancelled.
    if (cancelled_task_ids.find(id) != cancelled_task_ids.end())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Request with task_id:[%s] has already been cancelled.",
        id.c_str());

      dispatch_ack.success = true;
      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    auto request_to_cancel_it = assigned_requests.find(id);
    if (request_to_cancel_it == assigned_requests.end())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Unable to cancel task with task_id:[%s] as it is not assigned to "
        "fleet:[%s].",
        id.c_str(), name.c_str());

      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    std::unordered_set<std::string> executed_tasks;
    for (const auto& [context, mgr] : task_managers)
    {
      const auto& tasks = mgr->get_executed_tasks();
      executed_tasks.insert(tasks.begin(), tasks.end());
    }

    // Check if received request is to cancel an active task
    if (executed_tasks.find(id) != executed_tasks.end())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Unable to cancel active task with task_id:[%s]. Only queued tasks may "
        "be cancelled.",
        id.c_str());

      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    // Re-plan assignments while ignoring request for task to be cancelled
    const auto replan_results = allocate_tasks(
      nullptr, request_to_cancel_it->second);

    if (!replan_results.has_value())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Unable to re-plan assignments when cancelling task with task_id:[%s]",
        id.c_str());

      dispatch_ack_pub->publish(dispatch_ack);
      return;
    }

    const auto& assignments = replan_results.value();
    std::size_t index = 0;
    for (auto& t : task_managers)
    {
      t.second->set_queue(assignments[index], task_profile_map);
      ++index;
    }

    current_assignment_cost = task_planner->compute_cost(assignments);

    dispatch_ack.success = true;
    dispatch_ack_pub->publish(dispatch_ack);
    cancelled_task_ids.insert(id);

    RCLCPP_INFO(
      node->get_logger(),
      "Task with task_id:[%s] has successfully been cancelled. Assignments "
      "updated for robots in fleet [%s].",
      id.c_str(), name.c_str());
  }

  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Received DispatchRequest for task_id:[%s] with invalid method. Only "
      "ADD and CANCEL methods are supported. This request will be ignored.",
      id.c_str());
    return;
  }

}

//==============================================================================
auto FleetUpdateHandle::Implementation::is_valid_assignments(
  Assignments& assignments) const -> bool
{
  std::unordered_set<std::string> executed_tasks;
  for (const auto& [context, mgr] : task_managers)
  {
    const auto& tasks = mgr->get_executed_tasks();
    executed_tasks.insert(tasks.begin(), tasks.end());
  }

  for (const auto& agent : assignments)
  {
    for (const auto& a : agent)
    {
      if (executed_tasks.find(a.request()->id()) != executed_tasks.end())
        return false;
    }
  }

  return true;
}

//==============================================================================
std::optional<std::size_t> FleetUpdateHandle::Implementation::
get_nearest_charger(
  const rmf_traffic::agv::Planner::Start& start)
{
  if (charging_waypoints.empty())
    return std::nullopt;

  double min_cost = std::numeric_limits<double>::max();
  std::optional<std::size_t> nearest_charger = std::nullopt;
  for (const auto& wp : charging_waypoints)
  {
    const rmf_traffic::agv::Planner::Goal goal{wp};
    const auto& planner_result = (*planner)->setup(start, goal);
    const auto ideal_cost = planner_result.ideal_cost();
    if (ideal_cost.has_value() && ideal_cost.value() < min_cost)
    {
      min_cost = ideal_cost.value();
      nearest_charger = wp;
    }
  }

  return nearest_charger;
}

//==============================================================================
void FleetUpdateHandle::Implementation::fleet_state_publish_period(
  std::optional<rmf_traffic::Duration> value)
{
  if (value.has_value())
  {
    fleet_state_timer = node->create_wall_timer(
      std::chrono::seconds(1), [this]() { this->publish_fleet_state(); });
  }
  else
  {
    fleet_state_timer = nullptr;
  }
}

namespace {
//==============================================================================
rmf_fleet_msgs::msg::RobotState convert_state(const TaskManager& mgr)
{
  const RobotContext& context = *mgr.context();

  const auto mode = mgr.robot_mode();

  auto location = [&]() -> rmf_fleet_msgs::msg::Location
    {
      if (context.location().empty())
      {
        // TODO(MXG): We should emit some kind of critical error if this ever
        // happens
        return rmf_fleet_msgs::msg::Location();
      }

      const auto& graph = context.planner()->get_configuration().graph();
      const auto& l = context.location().front();
      const auto& wp = graph.get_waypoint(l.waypoint());
      const Eigen::Vector2d p = l.location().value_or(wp.get_location());

      return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
        .t(rmf_traffic_ros2::convert(l.time()))
        .x(p.x())
        .y(p.y())
        .yaw(l.orientation())
        .level_name(wp.get_map_name())
        // NOTE(MXG): This field is only used by the fleet drivers. For now, we
        // will just fill it with a zero.
        .index(0);
    } ();


  return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotState>()
    .name(context.name())
    .model(context.description().owner())
    .task_id(mgr.current_task() ? mgr.current_task()->id() : "")
    // TODO(MXG): We could keep track of the seq value and increment it once
    // with each publication. This is not currently an important feature
    // outside of the fleet driver, so for now we just set it to zero.
    .seq(0)
    .mode(std::move(mode))
    // We multiply by 100 to convert from the [0.0, 1.0] range to percentage
    .battery_percent(context.current_battery_soc()*100.0)
    .location(std::move(location))
    // NOTE(MXG): The path field is only used by the fleet drivers. For now,
    // we will just fill it with a zero. We could consider filling it in based
    // on the robot's plan, but that seems redundant with the traffic schedule
    // information.
    .path({});
}
} // anonymous namespace

//==============================================================================
void FleetUpdateHandle::Implementation::publish_fleet_state() const
{
  std::vector<rmf_fleet_msgs::msg::RobotState> robot_states;
  for (const auto& [context, mgr] : task_managers)
    robot_states.emplace_back(convert_state(*mgr));

  auto fleet_state = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::FleetState>()
    .name(name)
    .robots(std::move(robot_states));

  fleet_state_pub->publish(std::move(fleet_state));
}

//==============================================================================
auto FleetUpdateHandle::Implementation::allocate_tasks(
  rmf_task::ConstRequestPtr new_request,
  rmf_task::ConstRequestPtr ignore_request) const -> std::optional<Assignments>
{
  // Collate robot states, constraints and combine new requestptr with
  // requestptr of non-charging tasks in task manager queues
  std::vector<rmf_task::agv::State> states;
  std::vector<rmf_task::ConstRequestPtr> pending_requests;
  std::string id = "";

  if (new_request)
  {
    pending_requests.push_back(new_request);
    id = new_request->id();
  }

  for (const auto& t : task_managers)
  {
    states.push_back(t.second->expected_finish_state());
    const auto requests = t.second->requests();
    pending_requests.insert(
      pending_requests.end(), requests.begin(), requests.end());
  }

  // Remove the request to be ignored if present
  if (ignore_request)
  {
    auto ignore_request_it = pending_requests.end();
    for (auto it = pending_requests.begin(); it != pending_requests.end(); ++it)
    {
      auto pending_request = *it;
      if (pending_request->id() == ignore_request->id())
        ignore_request_it = it;
    }
    if (ignore_request_it != pending_requests.end())
    {
      pending_requests.erase(ignore_request_it);
      RCLCPP_INFO(
        node->get_logger(),
        "Request with task_id:[%s] will be ignored during task allocation.",
        ignore_request->id().c_str());
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Request with task_id:[%s] is not present in any of the task queues.",
        ignore_request->id().c_str());
    }
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Planning for [%ld] robot(s) and [%ld] request(s)",
    states.size(),
    pending_requests.size());

  // Generate new task assignments
  const auto result = task_planner->optimal_plan(
    rmf_traffic_ros2::convert(node->now()),
    states,
    pending_requests,
    nullptr);

  auto assignments_ptr = std::get_if<
    rmf_task::agv::TaskPlanner::Assignments>(&result);

  if (!assignments_ptr)
  {
    auto error = std::get_if<
      rmf_task::agv::TaskPlanner::TaskPlannerError>(&result);

    if (*error == rmf_task::agv::TaskPlanner::TaskPlannerError::low_battery)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s] due to"
        " insufficient initial battery charge for all robots in this fleet.",
        id.c_str());
    }

    else if (*error ==
      rmf_task::agv::TaskPlanner::TaskPlannerError::limited_capacity)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s] due to"
        " insufficient battery capacity to accommodate one or more requests by"
        " any of the robots in this fleet.", id.c_str());
    }

    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s]",
        id.c_str());
    }

    return std::nullopt;
  }

  const auto assignments = *assignments_ptr;

  if (assignments.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "[TaskPlanner] Failed to compute assignments for task_id:[%s]",
      id.c_str());

    return std::nullopt;
  }

  return assignments;
}

//==============================================================================
void FleetUpdateHandle::add_robot(
  std::shared_ptr<RobotCommandHandle> command,
  const std::string& name,
  const rmf_traffic::Profile& profile,
  rmf_traffic::agv::Plan::StartSet start,
  std::function<void(std::shared_ptr<RobotUpdateHandle>)> handle_cb)
{

  if (start.empty())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[FleetUpdateHandle::add_robot] StartSet is empty. Adding a robot to a "
      "fleet requires at least one rmf_traffic::agv::Plan::Start to be "
      "specified.");
    // *INDENT-ON*
  }

  rmf_traffic::schedule::ParticipantDescription description(
    name,
    _pimpl->name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    profile);

  _pimpl->writer->async_make_participant(
    std::move(description),
    [worker = _pimpl->worker,
    command = std::move(command),
    start = std::move(start),
    handle_cb = std::move(handle_cb),
    fleet = shared_from_this()](
      rmf_traffic::schedule::Participant participant)
    {
      const auto charger_wp = fleet->_pimpl->get_nearest_charger(start[0]);

      if (!charger_wp.has_value())
      {
        // *INDENT-OFF*
        throw std::runtime_error(
          "[FleetUpdateHandle::add_robot] Unable to find nearest charging "
          "waypoint. Adding a robot to a fleet requires at least one charging"
          "waypoint to be present in its navigation graph.");
        // *INDENT-ON*
      }

      rmf_task::agv::State state = rmf_task::agv::State{
        start[0], charger_wp.value(), 1.0};
      auto context = std::make_shared<RobotContext>(
        RobotContext{
          std::move(command),
          std::move(start),
          std::move(participant),
          fleet->_pimpl->snappable,
          fleet->_pimpl->planner,
          fleet->_pimpl->node,
          fleet->_pimpl->worker,
          fleet->_pimpl->default_maximum_delay,
          state,
          fleet->_pimpl->task_planner
        });

      // We schedule the following operations on the worker to make sure we do not
      // have a multiple read/write race condition on the FleetUpdateHandle.
      worker.schedule(
        [context, fleet, node = fleet->_pimpl->node,
        handle_cb = std::move(handle_cb)](const auto&)
        {
          // TODO(MXG): We need to perform this test because we do not currently
          // support the distributed negotiation in unit test environments. We
          // should create an abstract NegotiationRoom interface in rmf_traffic and
          // use that instead.
          if (fleet->_pimpl->negotiation)
          {
            context->_negotiation_license =
            fleet->_pimpl->negotiation
            ->register_negotiator(
              context->itinerary().id(),
              std::make_unique<LiaisonNegotiator>(context));
          }

          RCLCPP_INFO(
            node->get_logger(),
            "Added a robot named [%s] with participant ID [%ld]",
            context->name().c_str(),
            context->itinerary().id());

          if (handle_cb)
          {
            handle_cb(RobotUpdateHandle::Implementation::make(std::move(context)));
          }
          else
          {
            RCLCPP_WARN(
              node->get_logger(),
              "FleetUpdateHandle::add_robot(~) was not provided a callback to "
              "receive the RobotUpdateHandle of the new robot. This means you will "
              "not be able to update the state of the new robot. This is likely to "
              "be a fleet adapter development error.");
            return;
          }

          fleet->_pimpl->task_managers.insert({context,
            TaskManager::make(context)});
        });
    });
}

//==============================================================================
void FleetUpdateHandle::close_lanes(std::vector<std::size_t> lane_indices)
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), lane_indices = std::move(lane_indices)](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto& current_lane_closures =
      (*self->_pimpl->planner)->get_configuration().lane_closures();

      bool any_changes = false;
      for (const auto& lane : lane_indices)
      {
        if (current_lane_closures.is_open(lane))
        {
          any_changes = true;
          break;
        }
      }

      if (!any_changes)
      {
        // No changes are needed to the planner
        return;
      }

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_lane_closures = new_config.lane_closures();
      for (const auto& lane : lane_indices)
        new_lane_closures.close(lane);

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));
    });
}

//==============================================================================
void FleetUpdateHandle::open_lanes(std::vector<std::size_t> lane_indices)
{
  _pimpl->worker.schedule(
    [w = weak_from_this(), lane_indices = std::move(lane_indices)](const auto&)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto& current_lane_closures =
      (*self->_pimpl->planner)->get_configuration().lane_closures();

      bool any_changes = false;
      for (const auto& lane : lane_indices)
      {
        if (current_lane_closures.is_closed(lane))
        {
          any_changes = true;
          break;
        }
      }

      if (!any_changes)
      {
        // No changes are needed to the planner
        return;
      }

      auto new_config = (*self->_pimpl->planner)->get_configuration();
      auto& new_lane_closures = new_config.lane_closures();
      for (const auto& lane : lane_indices)
        new_lane_closures.open(lane);

      *self->_pimpl->planner =
      std::make_shared<const rmf_traffic::agv::Planner>(
        new_config, rmf_traffic::agv::Planner::Options(nullptr));
    });
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_task_requests(
  AcceptTaskRequest check)
{
  _pimpl->accept_task = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_delivery_requests(
  AcceptDeliveryRequest check)
{
  _pimpl->accept_delivery = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::default_maximum_delay(
  std::optional<rmf_traffic::Duration> value)
{
  _pimpl->default_maximum_delay = value;
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::Duration>
FleetUpdateHandle::default_maximum_delay() const
{
  return _pimpl->default_maximum_delay;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::fleet_state_publish_period(
  std::optional<rmf_traffic::Duration> value)
{
  if (value.has_value())
  {
    _pimpl->fleet_state_timer = _pimpl->node->try_create_wall_timer(
      value.value(),
      [me = weak_from_this()]()
      {
        if (const auto self = me.lock())
          self->_pimpl->publish_fleet_state();
      });
  }
  else
  {
    _pimpl->fleet_state_timer = nullptr;
  }

  return *this;
}

//==============================================================================
bool FleetUpdateHandle::set_task_planner_params(
  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
  double recharge_threshold,
  double recharge_soc,
  bool account_for_battery_drain)
{
  if (battery_system &&
    motion_sink &&
    ambient_sink &&
    tool_sink &&
    (recharge_threshold >= 0.0 && recharge_threshold <= 1.0) &&
    (recharge_soc >= 0.0 && recharge_threshold <= 1.0))
  {
    const rmf_task::agv::Parameters parameters{
      *_pimpl->planner,
      *battery_system,
      motion_sink,
      ambient_sink,
      tool_sink};
    const rmf_task::agv::Constraints constraints{
      recharge_threshold,
      recharge_soc,
      account_for_battery_drain};
    const rmf_task::agv::TaskPlanner::Configuration task_config{
      parameters,
      constraints,
      _pimpl->cost_calculator};
    _pimpl->task_planner = std::make_shared<rmf_task::agv::TaskPlanner>(
      std::move(task_config));

    // Here we update the task planner in all the RobotContexts.
    // The TaskManagers rely on the parameters in the task planner for
    // automatic retreat. Hence, we also update them whenever the
    // task planner here is updated.
    for (const auto& t : _pimpl->task_managers)
      t.first->task_planner(_pimpl->task_planner);

    return true;
  }

  return false;
}

//==============================================================================
FleetUpdateHandle::FleetUpdateHandle()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
