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

//==============================================================================
void check_path_finish(rclcpp::Node* node,
    const rmf_fleet_msgs::msg::RobotState& state,
    TravelInfo& info)
{
  // The robot believes it has reached the end of its path.
  const auto& wp = info.waypoints.back();
  const auto& l = state.location;
  const Eigen::Vector2d p{l.x, l.y};
  const double dist = (p - wp.position().block<2,1>(0, 0)).norm();

  assert(wp.graph_index());
  info.last_known_wp = *wp.graph_index();

  assert(info.waypoints.size() > 2);

  if (dist > 2.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "The robot is very far [%fm] from where it is supposed to be, but "
      "its remaining path is empty. This means the robot believes it is"
      "finished, but it is not where it is supposed to be.", dist);
    estimate_state(node, state.location, info);
    return;
  }

  if (dist > 0.5)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "The robot is somewhat far [%fm] from where it is supposed to be, "
      "but we will proceed anyway.");

    const auto& last_wp = info.waypoints[info.waypoints.size()-2];
    estimate_midlane_state(state.location, last_wp.graph_index(), wp, info);
  }
  else
  {
    // We are close enough to the goal that we will say the robot is
    // currently located there.
    info.updater->update_position(*wp.graph_index(), l.yaw);
  }

  info.path_finished_callback();
  info.path_finished_callback = nullptr;
  info.next_arrival_estimator = nullptr;
}

//==============================================================================
void estimate_path_traveling(
    const rmf_fleet_msgs::msg::RobotState& state,
    TravelInfo& info)
{
  const std::size_t remaining_count = state.path.size();
  const std::size_t i_target_wp = info.waypoints.size() - remaining_count;
  const auto& target_wp = info.waypoints[i_target_wp];

  const auto& l = state.location;
  const auto& p = target_wp.position();
  const auto interp = rmf_traffic::agv::Interpolate::positions(
        *info.traits, std::chrono::steady_clock::now(), {{l.x, l.y, l.yaw}, p});
  const auto next_arrival = interp.back().time() - interp.front().time();
  info.next_arrival_estimator(i_target_wp, next_arrival);

  rmf_utils::optional<std::size_t> lane_start;
  if (i_target_wp > 1)
  {
    lane_start = info.waypoints[i_target_wp-1].graph_index();
    if (lane_start)
      info.last_known_wp = *lane_start;
  }

  return estimate_midlane_state(l, lane_start, target_wp, info);
}

//==============================================================================
void estimate_midlane_state(
    const rmf_fleet_msgs::msg::Location& l,
    rmf_utils::optional<std::size_t> lane_start,
    const rmf_traffic::agv::Plan::Waypoint& target_wp,
    TravelInfo& info)
{
  if (!lane_start && info.last_known_wp)
  {
    // Let's see if the current position is reasonably between the last known
    // waypoint and the target waypoint.
    const auto& base_wp = info.graph->get_waypoint(*info.last_known_wp);
    const Eigen::Vector2d p0 = base_wp.get_location();
    const Eigen::Vector2d p1 = target_wp.position().block<2,1>(0,0);
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

  if (lane_start)
  {
    std::vector<std::size_t> lanes;
    const auto last_gi = *lane_start;
    const auto target_gi = *target_wp.graph_index();
    const auto* forward_lane = info.graph->lane_from(last_gi, target_gi);
    assert(forward_lane);
    lanes.push_back(forward_lane->index());

    if (const auto* reverse_lane = info.graph->lane_from(target_gi, last_gi))
    {
      if (!reverse_lane->entry().event())
      {
        // We don't allow the robot to turn back mid-lane if the reverse lane
        // has an entry event, because if that entry event is docking, then it
        // needs to be triggered for the robot to approach the exit.
        //
        // TODO(MXG): This restriction isn't needed for reversing on door or
        // lift events, so with some effort we could loosen this restriction to
        // only apply to docking.
        lanes.push_back(reverse_lane->index());
      }
    }

    info.updater->update_position({l.x, l.y, l.yaw}, std::move(lanes));
    return;
  }

  // The target should always have a graph index, because only the first
  // waypoint in a command should ever be lacking a graph index.
  assert(target_wp.graph_index());
  info.updater->update_position({l.x, l.y, l.yaw}, *target_wp.graph_index());
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
          "floor name in the level_name field of its RobotState.");
    return;
  }

  info.updater->update_position(last_known_map, {l.x, l.y, l.yaw});
}
