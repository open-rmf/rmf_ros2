/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "FleetAdapterNode.hpp"

#include "../rmf_fleet_adapter/ParseGraph.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/executors.hpp>

#include "Actions.hpp"
#include "Tasks.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    const std::string& fleet_name,
    const std::string& graph_file,
    rmf_traffic::agv::VehicleTraits traits,
    const rmf_traffic::Duration delay_threshold,
    const rmf_traffic::Duration plan_time,
    const rmf_traffic::Duration wait_time)
{
  const auto node = std::shared_ptr<FleetAdapterNode>(
        new FleetAdapterNode(fleet_name, delay_threshold, plan_time));

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
        *node, rmf_traffic::schedule::query_everything().spacetime());

  auto submit_trajectories = node->create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  auto delay_trajectories = node->create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  auto replace_trajectories = node->create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  auto erase_trajectories = node->create_client<EraseTrajectories>(
        rmf_traffic_ros2::EraseTrajectoriesSrvName);

  auto resolve_conflicts = node->create_client<ResolveConflicts>(
        rmf_traffic_ros2::ResolveConflictsSrvName);

  rmf_utils::optional<GraphInfo> graph_info =
      parse_graph(graph_file, traits, *node);
  if (!graph_info)
    return nullptr;

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= submit_trajectories->service_is_ready();
    ready &= delay_trajectories->service_is_ready();
    ready &= replace_trajectories->service_is_ready();
    ready &= erase_trajectories->service_is_ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      node->start(
            Fields{
              std::move(*graph_info),
              std::move(traits),
              mirror_future.get(),
              std::move(submit_trajectories),
              std::move(delay_trajectories),
              std::move(replace_trajectories),
              std::move(erase_trajectories),
              std::move(resolve_conflicts)
            });

      return node;
    }
  }

  RCLCPP_ERROR(
        node->get_logger(),
        "Timeout after waiting ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time))
        + "] to connect to the schedule");

  return nullptr;
}

//==============================================================================
FleetAdapterNode::RobotContext::RobotContext(
    std::string name,
    Location location_)
: location(std::move(location_)),
  _name(std::move(name))
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::RobotContext::next_task()
{
  if (!_task_queue.empty())
  {
    _task = std::move(_task_queue.front());
    _task_queue.erase(_task_queue.begin());
    _task->next();
    return;
  }

  // TODO(MXG): If the task queue is empty, have the robot move back to its
  // home.
}

//==============================================================================
void FleetAdapterNode::RobotContext::add_task(std::unique_ptr<Task> new_task)
{
  if (!_task)
  {
    // No task is currently active, so set this new task as the active one and
    // begin executing it.
    _task = std::move(new_task);
    _task->next();
  }
  else
  {
    // A task is currently active, so add this task to the queue. We're going to
    // do first-come first-serve for task requests for now.
    _task_queue.emplace_back(std::move(_task));
  }
}

//==============================================================================
void FleetAdapterNode::RobotContext::discard_task(Task* discarded_task)
{
  if (_task.get() == discarded_task)
  {
    return next_task();
  }

  const auto it = std::find_if(_task_queue.begin(), _task_queue.end(),
               [&](const std::unique_ptr<Task>& task)
  {
    return task.get() == discarded_task;
  });

  if (it != _task_queue.end())
    _task_queue.erase(it);
}

//==============================================================================
void FleetAdapterNode::RobotContext::interrupt()
{
  if (_task)
    _task->interrupt();
}

//==============================================================================
void FleetAdapterNode::RobotContext::resume()
{
  if (_task)
    _task->resume();
}

//==============================================================================
std::size_t FleetAdapterNode::RobotContext::num_tasks() const
{
  const std::size_t n_queue = _task_queue.size();
  return _task? n_queue + 1 : n_queue;
}

//==============================================================================
const std::string& FleetAdapterNode::RobotContext::robot_name() const
{
  return _name;
}

//==============================================================================
const std::string& FleetAdapterNode::get_fleet_name() const
{
  return _fleet_name;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_plan_time() const
{
  return _plan_time;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_delay_threshold() const
{
  return _delay_threshold;
}

//==============================================================================
const rmf_traffic::agv::Planner& FleetAdapterNode::get_planner() const
{
  return _field->planner;
}

//==============================================================================
const rmf_traffic::agv::Graph& FleetAdapterNode::get_graph() const
{
  return _field->graph_info.graph;
}

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Start>
FleetAdapterNode::compute_plan_starts(const Location& location)
{
  // Add 3 seconds to the current time to give us some buffer
  // TODO(MXG): Make this configurable
  const auto now = rmf_traffic_ros2::convert(get_clock()->now())
      + std::chrono::seconds(3);

  const Eigen::Vector2d p_location = {location.x, location.y};
  const double start_yaw = static_cast<double>(location.yaw);

  const auto& graph = _field->graph_info.graph;

  for (std::size_t i=0; i < graph.num_waypoints(); ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    const Eigen::Vector2d wp_location = wp.get_location();

    if ( (p_location - wp_location).norm() < 0.05 )
    {
      // This waypoint is very close to the real location, so we will assume
      // that the robot is located here.
      return {rmf_traffic::agv::Plan::Start(now, wp.index(), start_yaw)};
    }
  }

  std::vector<rmf_traffic::agv::Plan::Start> starts;

  double closest_lane_dist = std::numeric_limits<double>::infinity();
  std::size_t closest_lane = std::numeric_limits<std::size_t>::max();
  for (std::size_t i=0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const Eigen::Vector2d p0 = graph.get_waypoint(
          lane.entry().waypoint_index()).get_location();
    const Eigen::Vector2d p1 = graph.get_waypoint(
          lane.exit().waypoint_index()).get_location();

    const double lane_length = (p1 - p0).norm();
    const Eigen::Vector2d pn = (p1 - p0)/lane_length;

    const Eigen::Vector2d p_l = p_location - p0;
    const double p_l_projection = p_l.dot(pn);

    double lane_dist = std::numeric_limits<double>::infinity();
    if (p_l_projection < 0.0)
    {
      // If it's negative then its closest point on the lane is the entry point
      lane_dist = p_l.norm();
    }
    else if (p_l_projection > lane_length)
    {
      // If it's larger than the lane length, then its closest point on the lane
      // is the exit point.
      lane_dist = (p_location - p1).norm();
    }
    else
    {
      // If it's between the entry and the exit points, then we should compute
      // its distance away from the lane line.
      lane_dist = (p_l - p_l_projection*pn).norm();
    }

    if (lane_dist < 0.25)
    {
      // If the robot is within 25cm of the lane, go ahead and add it as one of
      // the options to consider.
      starts.emplace_back(
            rmf_traffic::agv::Plan::Start(
              now, lane.exit().waypoint_index(), start_yaw, p_location, i));
    }
  }

  if (starts.empty())
  {
    // None of the lanes were very close, so we'll go ahead and use the one that
    // seems closest.
    if (closest_lane_dist > 1.0)
    {
      RCLCPP_WARN(
            get_logger(),
            "The robot appears to be [" + std::to_string(closest_lane_dist)
            + "] meters away from its closest lane on the graph ["
            + std::to_string(closest_lane) + "]!");
    }

    starts.emplace_back(
          rmf_traffic::agv::Plan::Start(
            now, graph.get_lane(closest_lane).exit().waypoint_index(),
            start_yaw, p_location, closest_lane));
  }

  return starts;
}

//==============================================================================
auto FleetAdapterNode::get_waypoint_keys() const -> const WaypointKeys&
{
  return _field->graph_info.keys;
}

//==============================================================================
auto FleetAdapterNode::get_waypoint_names() const -> const WaypointNames&
{
  return _field->graph_info.waypoint_names;
}

//==============================================================================
const std::vector<std::size_t>& FleetAdapterNode::get_parking_spots() const
{
  return _field->graph_info.parking_spots;
}

//==============================================================================
auto FleetAdapterNode::get_fields() const -> const Fields&
{
  return *_field;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(
    const std::string& fleet_name,
    rmf_traffic::Duration delay_threshold,
    rmf_traffic::Duration plan_time)
: rclcpp::Node(fleet_name + "__full_control_fleet_adapter"),
  _fleet_name(fleet_name),
  _delay_threshold(delay_threshold),
  _plan_time(plan_time)
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::start(Fields fields)
{
  _field = std::move(fields);
  _field->mirror.update();

  const auto default_qos = rclcpp::SystemDefaultsQoS();

  _delivery_sub = create_subscription<Delivery>(
        DeliveryTopicName, default_qos,
        [&](Delivery::UniquePtr msg)
  {
    this->delivery_request(std::move(msg));
  });

  _loop_request_sub = create_subscription<LoopRequest>(
        LoopRequestTopicName, default_qos,
        [&](LoopRequest::UniquePtr msg)
  {
    this->loop_request(std::move(msg));
  });

  _dispenser_result_sub = create_subscription<DispenserResult>(
        DispenserResultTopicName, default_qos,
        [&](DispenserResult::UniquePtr msg)
  {
    this->dispenser_result_update(std::move(msg));
  });

  _dispenser_state_sub = create_subscription<DispenserState>(
        DispenserStateTopicName, default_qos,
        [&](DispenserState::UniquePtr msg)
  {
    this->dispenser_state_update(std::move(msg));
  });

  _fleet_state_sub = create_subscription<FleetState>(
        FleetStateTopicName, default_qos,
        [&](FleetState::UniquePtr msg)
  {
    this->fleet_state_update(std::move(msg));
  });

  _door_state_sub = create_subscription<DoorState>(
        DoorStateTopicName, default_qos,
        [&](DoorState::UniquePtr msg)
  {
    this->door_state_update(std::move(msg));
  });

  _lift_state_sub = create_subscription<LiftState>(
        LiftStateTopicName, default_qos,
        [&](LiftState::UniquePtr msg)
  {
    this->lift_state_update(std::move(msg));
  });

  _schedule_conflict_sub = create_subscription<ScheduleConflict>(
        rmf_traffic_ros2::ScheduleConflictTopicName, default_qos,
        [&](ScheduleConflict::UniquePtr msg)
  {
    this->schedule_conflict_update(std::move(msg));
  });

  _emergency_notice_sub = create_subscription<EmergencyNotice>(
        rmf_traffic_ros2::EmergencyTopicName, default_qos,
        [&](EmergencyNotice::UniquePtr msg)
  {
    this->emergency_notice_update(std::move(msg));
  });

  path_request_publisher = create_publisher<PathRequest>(
        PathRequestTopicName, default_qos);

  door_request_publisher = create_publisher<DoorRequest>(
        AdapterDoorRequestTopicName, default_qos);

  lift_request_publisher = create_publisher<LiftRequest>(
        AdapterLiftRequestTopicName, default_qos);

  dispenser_request_publisher = create_publisher<DispenserRequest>(
        DispenserRequestTopicName, default_qos);

  task_summary_publisher = create_publisher<TaskSummary>(
        TaskSummaryTopicName, default_qos);
}

//==============================================================================
void FleetAdapterNode::delivery_request(Delivery::UniquePtr msg)
{
  if (_contexts.empty())
  {
    RCLCPP_ERROR(
          get_logger(),
          "No robots appear to be online!");
    return;
  }

  auto task_insertion = _received_tasks.insert(msg->task_id);
  if (!task_insertion.second)
  {
    // We've already received and processed this task in the past, so we can
    // ignore it.
    return;
  }

  auto context = _contexts.begin()->second.get();

  auto task = make_delivery(this, context, *msg);
  if (task)
    context->add_task(std::move(task));
}

//==============================================================================
void FleetAdapterNode::loop_request(LoopRequest::UniquePtr msg)
{
  if (msg->robot_type != _fleet_name)
    return;

  std::size_t fewest = std::numeric_limits<std::size_t>::max();
  RobotContext* fewest_context = nullptr;
  for (const auto& c : _contexts)
  {
    const std::size_t n = c.second->num_tasks();
    if (n < fewest)
    {
      fewest = n;
      fewest_context = c.second.get();
    }
  }

  if (fewest_context)
  {
    auto task = make_loop(this, fewest_context, std::move(*msg));
    if (task)
      fewest_context->add_task(std::move(task));

    return;
  }

  RCLCPP_ERROR(
        get_logger(),
        "Cannot assign a robot to looping task [" + msg->task_id
        + "] because no robots have reported their existence.");

  rmf_task_msgs::msg::TaskSummary summary;
  summary.status = "Services unavailable";
  summary.start_time = get_clock()->now();
  summary.end_time = summary.start_time;
  summary.task_id = msg->task_id;
  summary.submission_time = summary.start_time;

  task_summary_publisher->publish(summary);

}

//==============================================================================
void FleetAdapterNode::dispenser_result_update(DispenserResult::UniquePtr msg)
{
  for (auto* listener : dispenser_result_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::dispenser_state_update(DispenserState::UniquePtr msg)
{
  for (auto* listener : dispenser_state_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr msg)
{
  const auto& fleet_state = *msg;
  if (fleet_state.name != _fleet_name)
    return;

  for (const auto& robot : fleet_state.robots)
  {
    const auto insertion = _contexts.insert(
          std::make_pair(robot.name, nullptr));
    const bool inserted = insertion.second;
    const auto it = insertion.first;

    if (inserted)
    {
      it->second = std::make_unique<RobotContext>(
            RobotContext{robot.name, robot.location});
    }
    else
    {
      it->second->location = robot.location;
    }

    for (auto* listener : it->second->state_listeners)
      listener->receive(robot);
  }
}

//==============================================================================
void FleetAdapterNode::door_state_update(DoorState::UniquePtr msg)
{
  for (const auto& listener : door_state_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::lift_state_update(LiftState::UniquePtr msg)
{
  for (const auto& listener : lift_state_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::schedule_conflict_update(ScheduleConflict::UniquePtr msg)
{
  for (const auto& listener : schedule_conflict_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::emergency_notice_update(EmergencyNotice::UniquePtr msg)
{
  const bool active_emergency = msg->data;
  if (active_emergency == _in_emergency_mode)
    return;

  _in_emergency_mode = active_emergency;

  if (active_emergency)
  {
    for (const auto& c : _contexts)
      c.second->interrupt();
  }

  if (!active_emergency)
  {
    for (const auto& c : _contexts)
      c.second->resume();
  }
}

} // namespace full_control
} // namespace rmf_fleet_adapter
