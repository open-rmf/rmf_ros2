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

#include "estimation.hpp"

#include <rmf_traffic_ros2/Time.hpp>

//==============================================================================
void check_path_finish(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info)
{
  // The robot believes it has reached the end of its path.
  const auto& wp = info.waypoints.back();
  const auto& l = state.location;
  const Eigen::Vector2d p{l.x, l.y};
  const double dist = (p - wp.position().block<2, 1>(0, 0)).norm();

  if (wp.graph_index().has_value())
  {
    info.last_known_wp = *wp.graph_index();

    if (dist > 2.0)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Robot named [%s] belonging to fleet [%s] is very far [%fm] from where "
        "it is supposed to be, but its remaining path is empty. This means the "
        "robot believes it is finished, but it is not where it's supposed to be.",
        info.robot_name.c_str(), info.fleet_name.c_str(), dist);
      estimate_state(node, state.location, info);
      return;
    }

    if (dist > 0.5)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "The robot is somewhat far [%fm] from where it is supposed to be, "
        "but we will proceed anyway.",
        dist);

      const auto& last_wp = info.waypoints[info.waypoints.size()-2];
      estimate_midlane_state(
        state.location, last_wp.graph_index(), info.waypoints.size()-1, info);
    }
    else
    {
      // We are close enough to the goal that we will say the robot is
      // currently located there.
      info.updater->update_position(*wp.graph_index(), l.yaw);
    }
  }
  else
  {
    estimate_state(node, state.location, info);
  }

  assert(info.waypoints.size() >= 2);

  assert(info.path_finished_callback);
  info.path_finished_callback();
  info.path_finished_callback = nullptr;
  info.next_arrival_estimator = nullptr;
}

//==============================================================================
void estimate_path_traveling(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info)
{
  assert(!state.path.empty());

  const std::size_t remaining_count = state.path.size();
  const std::size_t i_target_wp = info.waypoints.size() - remaining_count;
  info.target_plan_index = i_target_wp;
  const auto& target_wp = info.waypoints.at(i_target_wp);

  const auto& l = state.location;
  const auto& p = target_wp.position();
  const auto interp = rmf_traffic::agv::Interpolate::positions(
    *info.traits, std::chrono::steady_clock::now(), {{l.x, l.y, l.yaw}, p});
  const auto next_arrival = interp.back().time() - interp.front().time();
  const auto now = rmf_traffic_ros2::convert(node->now());
  if (target_wp.time() < now + next_arrival)
  {
    assert(info.next_arrival_estimator);
    // It seems the robot cannot arrive on time, so we report the earliest that
    // the robot can make it to its next target waypoint.
    info.next_arrival_estimator(i_target_wp, next_arrival);
  }
  else
  {
    assert(info.next_arrival_estimator);
    // It seems the robot will arrive on time, so we'll report that.
    info.next_arrival_estimator(i_target_wp, target_wp.time() - now);
  }

  rmf_utils::optional<std::size_t> lane_start;
  if (i_target_wp > 1)
  {
    lane_start = info.waypoints[i_target_wp-1].graph_index();
    if (lane_start)
      info.last_known_wp = *lane_start;

    return estimate_midlane_state(l, lane_start, i_target_wp, info);
  }

  estimate_state(node, state.location, info);
}

//==============================================================================
void estimate_midlane_state(
  const rmf_fleet_msgs::msg::Location& l,
  rmf_utils::optional<std::size_t> lane_start,
  const std::size_t next_index,
  TravelInfo& info)
{
  assert(0 < next_index && next_index < info.waypoints.size());
  const auto& target_wp = info.waypoints.at(next_index);
  if (!lane_start && info.last_known_wp)
  {
    // Let's see if the current position is reasonably between the last known
    // waypoint and the target waypoint.
    const auto& base_wp = info.graph->get_waypoint(*info.last_known_wp);
    const Eigen::Vector2d p0 = base_wp.get_location();
    const Eigen::Vector2d p1 = target_wp.position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_location{l.x, l.y};

    const double lane_length = (p1 - p0).norm();

    if (lane_length > 1e-8)
    {
      const Eigen::Vector2d pn = (p1 - p0) / lane_length;
      const Eigen::Vector2d p_l = p_location - p0;
      const double p_l_projection = p_l.dot(pn);
      const double lane_dist = (p_l - p_l_projection*pn).norm();

      if (0.0 <= p_l_projection && p_l_projection <= lane_length
        && lane_dist < 2.0)
      {
        lane_start = *info.last_known_wp;
      }
    }
  }

  const std::optional<std::size_t> target_gi = [&]()
    -> std::optional<std::size_t>
    {
      // At least one future waypoint must have a graph index
      if (target_wp.graph_index())
        return *target_wp.graph_index();

      for (std::size_t i = next_index+1; i < info.waypoints.size(); ++i)
      {
        const auto gi = info.waypoints[i].graph_index();
        if (gi)
          return *gi;
      }

      return std::nullopt;
    } ();

  if (target_gi.has_value())
  {
    if (lane_start.has_value())
    {
      const auto last_gi = *lane_start;
      if (last_gi == target_gi)
      {
        // This implies that the robot is either waiting at or rotating on the
        // waypoint.
        return info.updater->update_position(*target_gi, l.yaw);
      }
      else if (const auto* forward_lane =
        info.graph->lane_from(last_gi, *target_gi))
      {
        // This implies that the robot is moving down a lane.
        std::vector<std::size_t> lanes;
        lanes.push_back(forward_lane->index());

        const auto* reverse_lane = info.graph->lane_from(*target_gi, last_gi);
        if (reverse_lane)
        {
          if (!reverse_lane->entry().event())
          {
            // We don't allow the robot to turn back mid-lane if the reverse lane
            // has an entry event, because if that entry event is docking, then it
            // needs to be triggered for the robot to approach the exit.
            //
            // TODO(MXG): This restriction isn't needed for reversing on door or
            // lift events, so with some effort we could loosen this restriction
            // to only apply to docking.
            lanes.push_back(reverse_lane->index());
          }
        }

        return info.updater->update_position({l.x, l.y, l.yaw},
            std::move(lanes));
      }
    }

    return info.updater->update_position({l.x, l.y, l.yaw}, *target_gi);
  }

  info.updater->update_position(l.level_name, {l.x, l.y, l.yaw});
}

//==============================================================================
void estimate_state(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& l,
  TravelInfo& info)
{
  std::string last_known_map = l.level_name;
  if (info.last_known_wp)
  {
    const auto& last_known_wp = info.graph->get_waypoint(*info.last_known_wp);
    const Eigen::Vector2d p_last = last_known_wp.get_location();
    const Eigen::Vector2d p{l.x, l.y};
    const double dist = (p_last - p).norm();
    if (dist < 0.25)
    {
      // We will assume that the robot is meant to be on this last known
      // waypoint.
      info.updater->update_position(last_known_wp.index(), l.yaw);
      return;
    }
    else if (dist < 1.5)
    {
      // We will assume that the robot is meant to be at this last known
      // waypoint, but is kind of diverged.
      info.updater->update_position({l.x, l.y, l.yaw}, last_known_wp.index());
      return;
    }

    if (last_known_map.empty())
      last_known_map = last_known_wp.get_map_name();
  }

  if (last_known_map.empty() && l.level_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Robot named [%s] belonging to fleet [%s] is lost because we cannot "
      "figure out what floor it is on. Please publish the robot's current "
      "floor name in the level_name field of its RobotState.",
      info.robot_name.c_str(), info.fleet_name.c_str());
    return;
  }

  info.updater->update_position(last_known_map, {l.x, l.y, l.yaw});
}

//==============================================================================
void estimate_waypoint(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& l,
  TravelInfo& info)
{
  std::string last_known_map = l.level_name;
  if (last_known_map.empty() && info.last_known_wp)
  {
    last_known_map = info.graph->get_waypoint(*info.last_known_wp)
      .get_map_name();
  }

  const Eigen::Vector2d p(l.x, l.y);
  const rmf_traffic::agv::Graph::Waypoint* closest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < info.graph->num_waypoints(); ++i)
  {
    const auto& wp = info.graph->get_waypoint(i);
    const Eigen::Vector2d p_wp = wp.get_location();
    const double dist = (p - p_wp).norm();
    if (dist < nearest_dist)
    {
      closest_wp = &wp;
      nearest_dist = dist;
    }
  }

  assert(closest_wp);

  if (nearest_dist > 0.5)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot named [%s] belonging to fleet [%s] is expected to be on a "
      "waypoint, but the nearest waypoint is [%fm] away.",
      info.robot_name.c_str(), info.fleet_name.c_str(), nearest_dist);
  }

  info.updater->update_position(closest_wp->index(), l.yaw);
}
