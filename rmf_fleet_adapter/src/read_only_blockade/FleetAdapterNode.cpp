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

#include "FleetAdapterNode.hpp"

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

#include "../rmf_fleet_adapter/load_param.hpp"

#include "../rmf_fleet_adapter/make_trajectory.hpp"

namespace rmf_fleet_adapter {
namespace read_only_blockade {

std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make()
{
  auto node = std::shared_ptr<FleetAdapterNode>(new FleetAdapterNode);

  const auto wait_time =
    get_parameter_or_default_time(*node, "discovery_timeout", 10.0);

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  auto graph = rmf_fleet_adapter::agv::parse_graph(graph_file, node->_traits);
  auto planner = rmf_traffic::agv::Planner(
    rmf_traffic::agv::Planner::Configuration(std::move(graph), node->_traits),
    rmf_traffic::agv::Planner::Options(nullptr)
  );

  node->_delay_threshold =
    get_parameter_or_default_time(*node, "delay_threshold", 5.0);

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    node, rmf_traffic::schedule::query_all());

  auto writer = rmf_traffic_ros2::schedule::Writer::make(node);

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= writer->ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      auto mirror = mirror_future.get();
      auto snapshot = mirror.view();
      node->_connect = Connections{
        std::move(writer),
        rmf_traffic_ros2::blockade::Writer::make(*node),
        std::move(mirror),
        rmf_traffic_ros2::schedule::Negotiation(*node, snapshot),
        std::move(planner)
      };

      node->_fleet_state_subscription =
        node->create_subscription<FleetState>(
        FleetStateTopicName, rclcpp::SystemDefaultsQoS(),
        [self = node.get()](FleetState::UniquePtr msg)
        {
          self->fleet_state_update(std::move(msg));
        });

      return node;
    }
  }

  return nullptr;
}

//==============================================================================
bool FleetAdapterNode::ignore_fleet(const std::string& fleet_name) const
{
  if (!_fleet_name.empty() && fleet_name != _fleet_name)
    return true;

  return false;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode()
: rclcpp::Node("fleet_adapter"),
  _fleet_name(get_fleet_name_parameter(*this)),
  _traits(get_traits_or_default(*this, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5))
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr new_state)
{
  if (ignore_fleet(new_state->name))
    return;

  std::lock_guard<std::mutex> lock(_async_mutex);
  for (const auto& robot : new_state->robots)
  {
    if (robot.name.empty())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Empty name field for RobotState is not supported!");

      return;
    }

    const auto insertion = _robots.insert(
      std::make_pair(robot.name, nullptr));

    if (insertion.second)
      register_robot(robot);
    else if (insertion.first->second)
      update_robot(robot, insertion.first);
  }
}

//==============================================================================
void FleetAdapterNode::register_robot(const RobotState& state)
{
  using namespace std::chrono_literals;
  rmf_traffic::schedule::ParticipantDescription description{
    state.name,
    _fleet_name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
    _traits.profile()
  };

  _connect->schedule_writer->async_make_participant(
    std::move(description),
    [this](rmf_traffic::schedule::Participant received_participant)
    {
      std::lock_guard<std::mutex> lock(this->_async_mutex);
      auto participant =
      std::make_shared<rmf_traffic::schedule::Participant>(
        std::move(received_participant));

      const auto radius =
      participant->description().profile()
      .vicinity()->get_characteristic_length();

      auto blockade = this->_connect->blockade_writer->make_participant(
        participant->id(), radius,
        [](const auto, const auto&)
        {
          // We ignore new ranges because we don't really care about getting
          // permission from the blockade moderator.
        });

      using TableViewerPtr = rmf_traffic::schedule::Negotiator::TableViewerPtr;
      using ResponderPtr = rmf_traffic::schedule::Negotiator::ResponderPtr;

      auto negotiated_delay = std::make_shared<rmf_traffic::Duration>(0s);

      auto license = this->_connect->negotiation.register_negotiator(
        participant->id(),
        [participant, negotiated_delay](
          TableViewerPtr viewer,
          ResponderPtr responder)
        {
          auto approval_cb = [participant, negotiated_delay](
            const rmf_traffic::Duration t)
          {
            *negotiated_delay += t;
            participant->delay(t);
            return participant->version();
          };

          rmf_traffic::schedule::StubbornNegotiator(*participant)
          .acceptable_waits({5s, 10s, 20s, 30s}, std::move(approval_cb))
          .additional_margins({20s})
          .respond(viewer, responder);
        });

      this->_robots.at(participant->description().name()) =
      std::make_unique<Robot>(
        Robot{
          participant,
          std::move(blockade),
          std::move(license),
          std::move(negotiated_delay),
          std::nullopt,
          std::nullopt
        });
    });
}

//==============================================================================
namespace {
bool different_location(
  const rmf_traffic::schedule::Participant& participant,
  const rmf_fleet_msgs::msg::RobotState& state)
{
  if (participant.itinerary().size() != 1)
    return true;

  const auto& route = participant.itinerary().front();

  const auto& location = state.location;
  if (route.map() != location.level_name)
    return true;

  const auto& trajectory = route.trajectory();
  if (trajectory.size() != 2)
    return true;

  const double threshold = 1e-2;
  const double trajectory_diff =
    (trajectory.front().position().head<2>()
    - trajectory.back().position().head<2>()).norm();

  if (trajectory_diff > threshold)
    return true;

  const Eigen::Vector2d p = {location.x, location.y};
  const double location_diff =
    (trajectory.back().position().head<2>() - p).norm();

  if (location_diff > threshold)
    return true;

  return false;
}
} // anonymous namespace

//==============================================================================
void FleetAdapterNode::update_robot(
  const RobotState& state,
  const Robots::iterator& it)
{
  auto& robot = it->second;
  const auto now = rmf_traffic_ros2::convert(this->now());

  if (robot->current_goal.has_value())
  {
    if (state.task_id.empty())
    {
      // Switch to not having a task
      robot->current_goal = std::nullopt;
      robot->schedule->set(
        robot->schedule->assign_plan_id(), make_hold(state, now));
      robot->blockade.cancel();
      return;
    }
    else if (robot->current_goal.value() == state.task_id
      && robot->expectation.has_value())
    {
      update_progress(state, *robot, now);
      return;
    }
    else
    {
      make_plan(state, *robot, now);
      return;
    }
  }
  else
  {
    // It doesn't hurt to make sure we're not blockading anything
    robot->blockade.cancel();
    robot->expectation = std::nullopt;

    if (state.task_id.empty())
    {
      // Just report our current location to the schedule
      if (robot->schedule->itinerary().empty())
      {
        robot->schedule->set(
          robot->schedule->assign_plan_id(), make_hold(state, now));
        return;
      }
      else if (different_location(*robot->schedule, state))
      {
        robot->schedule->set(
          robot->schedule->assign_plan_id(), make_hold(state, now));
        return;
      }
      else
      {
        // The robot is still at the same location that it was in last time, so
        // we can just add a delay to the schedule
        robot->schedule->delay(make_delay(*robot->schedule, now));
        return;
      }
    }
    else
    {
      make_plan(state, *robot, now);
      return;
    }
  }
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::make_delay(
  const rmf_traffic::schedule::Participant& schedule,
  rmf_traffic::Time now)
{
  const auto original_t =
    schedule.itinerary().front().trajectory().front().time();

  return now - original_t;
}

//==============================================================================
std::vector<rmf_traffic::Route> FleetAdapterNode::make_hold(
  const RobotState& state,
  rmf_traffic::Time start) const
{
  return
    {
      {
        state.location.level_name,
        ::make_hold(state.location, start, _hold_duration)
      }
    };
}

//==============================================================================
namespace {
FleetAdapterNode::Robot::Expectation convert_to_expectation(
  const rmf_traffic::agv::Plan& plan,
  const rmf_fleet_msgs::msg::RobotState& state,
  const rmf_traffic::agv::Graph& graph)
{
  const auto& plan_waypoints = plan.get_waypoints();

  std::vector<rmf_traffic::blockade::Writer::Checkpoint> path;
  path.reserve(plan_waypoints.size());
  std::vector<rmf_traffic::Time> timing;
  timing.reserve(plan_waypoints.size());

  std::string last_level_name = state.location.level_name;
  for (std::size_t i = 0; i < plan_waypoints.size(); ++i)
  {
    const auto& wp = plan_waypoints[i];
    const Eigen::Vector2d p = wp.position().head<2>();
    const rmf_traffic::Time t = wp.time();

    if (i > 0)
    {
      const Eigen::Vector2d last_p = plan_waypoints[i-1].position().head<2>();
      const auto delta = (p - last_p).norm();

      if (delta < 0.01)
        continue;
    }

    const auto map_name = [&]() -> std::string
      {
        if (wp.graph_index().has_value())
          return graph.get_waypoint(wp.graph_index().value()).get_map_name();

        return last_level_name;
      } ();

    last_level_name = map_name;

    path.push_back({p, map_name, true});
    timing.push_back(t);
  }

  return FleetAdapterNode::Robot::Expectation{
    std::move(path), std::move(timing)
  };
}

//==============================================================================
void add_offset_itinerary(
  rmf_traffic::Duration offset,
  const std::vector<rmf_traffic::Route>& original,
  std::vector<rmf_traffic::Route>& output)
{
  auto shadow = original;
  for (auto& item : shadow)
  {
    if (item.trajectory().empty())
      continue;

    const auto initial_time = *item.trajectory().start_time();
    item.trajectory().front().adjust_times(offset);
    item.trajectory().insert(
      initial_time,
      item.trajectory().front().position(),
      Eigen::Vector3d::Zero());
  }

  output.insert(output.end(), shadow.begin(), shadow.end());
}
} // anonymous namespace

//==============================================================================
void FleetAdapterNode::make_plan(
  const RobotState& state,
  Robot& robot,
  const rmf_traffic::Time now)
{
  const auto& graph = _connect->planner.get_configuration().graph();
  const auto* goal_wp = graph.find_waypoint(state.task_id);
  if (!goal_wp)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Unable to find a waypoint named [%s] for fleet [%s]",
      state.task_id.c_str(),
      _fleet_name.c_str());
    return;
  }

  const auto& location = state.location;
  const Eigen::Vector3d p = {location.x, location.y, location.yaw};
  const auto starts = rmf_traffic::agv::compute_plan_starts(
    _connect->planner.get_configuration().graph(),
    location.level_name,
    p, now,
    _waypoint_snap_distance,
    _lane_snap_distance);

  if (starts.empty())
  {
    std::stringstream ss;
    ss << "Unable to snap [" << state.name << "] onto the nav graph for fleet ["
       << _fleet_name << "]. Map: [" << location.level_name << "], position: ("
       << location.x << ", " << location.y << "), yaw: " << location.yaw;

    RCLCPP_ERROR(get_logger(), "%s", ss.str().c_str());
    return;
  }

  const auto result = _connect->planner.plan(starts, goal_wp->index());
  if (!result.success())
  {
    std::stringstream ss;
    ss << "Unable to find a plan for [" << state.name << "] for fleet ["
       << _fleet_name << "] to navigate from map [" << location.level_name
       << "], position (" << location.x << ", " << location.y << ") to the "
       << "waypoint named [" << state.task_id << "], graph index ["
       << goal_wp->index() << "]";

    RCLCPP_ERROR(get_logger(), "%s", ss.str().c_str());
    return;
  }

  if (result->get_waypoints().size() < 2)
  {
    // TODO(MXG): Refactor this condition with the one down below
    // We don't actually need to go anywhere
    robot.schedule->set(
      robot.schedule->assign_plan_id(), make_hold(state, now));
    robot.blockade.cancel();
    robot.expectation = std::nullopt;
    robot.current_goal = std::nullopt;
    return;
  }

  robot.expectation = convert_to_expectation(*result, state, graph);

  if (robot.expectation->path.size() == 1)
  {
    // We don't actually need to go anywhere
    robot.schedule->set(
      robot.schedule->assign_plan_id(), make_hold(state, now));
    robot.blockade.cancel();
    robot.expectation = std::nullopt;
    robot.current_goal = std::nullopt;
    return;
  }

  auto original = result->get_itinerary();
  const auto& last_wp = original.back().trajectory().back();
  original.back().trajectory().insert(
    last_wp.time() + std::chrono::seconds(60),
    last_wp.position(),
    Eigen::Vector3d::Zero());

  auto itinerary = original;
  add_offset_itinerary(std::chrono::seconds(5), original, itinerary);
  add_offset_itinerary(std::chrono::seconds(10), original, itinerary);
  add_offset_itinerary(std::chrono::seconds(15), original, itinerary);
  add_offset_itinerary(std::chrono::seconds(20), original, itinerary);

  robot.schedule->set(robot.schedule->assign_plan_id(), std::move(itinerary));
  robot.blockade.set(robot.expectation->path);

  // Immediately report that all checkpoints are ready. This will (hopefully)
  // block all traffic light robots from trying to enter our space.
  for (std::size_t i = 0; i < robot.blockade.path().size(); ++i)
    robot.blockade.ready(i);

  robot.current_goal = state.task_id;
}

//==============================================================================
std::optional<std::size_t> FleetAdapterNode::get_last_reached(
  const FleetAdapterNode::RobotState& state,
  const FleetAdapterNode::Robot& robot) const
{
  const auto& path = robot.expectation->path;

  const auto& location = state.location;
  const Eigen::Vector2d p = {location.x, location.y};

  for (std::size_t i = robot.blockade.last_reached(); i < path.size()-1; ++i)
  {
    const Eigen::Vector2d p0 = path[i].position;
    if ((p - p0).norm() < _waypoint_snap_distance)
    {
      // If we are still close to the start of the lane the robot is outside of
      // any earlier lanes, then we will say the robot's last reached point is
      // the start of this lane.
      return i;
    }

    const Eigen::Vector2d p1 = path[i+1].position;
    if ((p-p1).norm() <= _waypoint_snap_distance)
    {
      // If we are still close to the end of the lane, then we will say that the
      // robot's last reached the start checkpoint of this lane. That will help
      // other robots to keep their distance.
      return i;
    }

    const double lane_length = (p1 - p0).norm();
    const Eigen::Vector2d pn = (p1 - p0) / lane_length;
    const Eigen::Vector2d p_l = p - p0;
    const double p_l_projection = p_l.dot(pn);

    if (0.0 <= p_l_projection && p_l_projection <= lane_length)
    {
      // The robot is between the endpoints of the lane; now let's see if it
      // is near enough to the lane.
      const double lane_dist = (p_l - p_l_projection*pn).norm();
      if (lane_dist <= _lane_snap_distance)
      {
        // If the robot is close enough to this lane, we'll say that the last
        // checkpoint it reached was the lane's start point.
        return i;
      }
    }
  }

  return std::nullopt;
}

//==============================================================================
void FleetAdapterNode::update_arrival(
  const FleetAdapterNode::RobotState& state,
  FleetAdapterNode::Robot& robot,
  const rmf_traffic::Time now,
  const std::size_t last_reached_checkpoint)
{
  robot.blockade.reached(last_reached_checkpoint);

  const auto next_waypoint = last_reached_checkpoint + 1;
  if (next_waypoint >= robot.expectation->timing.size()
    || next_waypoint >= robot.expectation->path.size())
  {
    RCLCPP_ERROR(
      get_logger(),
      "[FleetAdapterNode::update_arrival] Index mismatch: Target index [%lu], "
      "timing vector size: [%lu], path vector size: [%lu]",
      next_waypoint,
      robot.expectation->timing.size(),
      robot.expectation->path.size());
    return;
  }

  const Eigen::Vector2d p0 = {state.location.x, state.location.y};
  const Eigen::Vector2d p1 = robot.expectation->path.at(next_waypoint).position;
  std::vector<Eigen::Vector3d> input_positions;
  input_positions.push_back({p0.x(), p0.y(), 0.0});
  input_positions.push_back({p1.x(), p1.y(), 0.0});
  const auto newly_expected_arrival =
    rmf_traffic::agv::Interpolate::positions(_traits, now, input_positions)
    .back().time();

  const auto current_delay = robot.schedule->delay();
  const auto planned_time = robot.expectation->timing.at(next_waypoint);
  const auto previously_expected_arrival = planned_time + current_delay;
  const auto new_delay = newly_expected_arrival - previously_expected_arrival;

  update_delay(new_delay, robot);
}

//==============================================================================
void FleetAdapterNode::update_delay(
  rmf_traffic::Duration new_delay,
  Robot& robot)
{
  using namespace std::chrono_literals;
  auto& negotiated_delay = *robot.negotiated_delay;
  if (negotiated_delay <= new_delay)
  {
    new_delay -= negotiated_delay;
    negotiated_delay = 0s;
  }
  else
  {
    negotiated_delay -= new_delay;
    new_delay = 0s;
  }

  robot.schedule->delay(new_delay);
}

//==============================================================================
void FleetAdapterNode::update_progress(
  const RobotState& state,
  Robot& robot,
  rmf_traffic::Time now)
{
  const auto last_reached_checkpoint = get_last_reached(state, robot);
  if (last_reached_checkpoint.has_value())
  {
    // We need to update the last reached checkpoint and the schedule timing
    update_arrival(state, robot, now, *last_reached_checkpoint);
  }
  else
  {
    // We need to replan because the robot is too far from our expected plan.
    make_plan(state, robot, now);
  }
}

} // namespace read_only_blockade
} // namespace rmf_fleet_adapter
