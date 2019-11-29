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

#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

namespace rmf_fleet_adapter {
namespace read_only {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::Duration delay_threshold,
    rmf_traffic::Duration wait_time)
{
  const auto stop_time = std::chrono::steady_clock::now() + wait_time;

  auto node = std::shared_ptr<FleetAdapterNode>(
        new FleetAdapterNode(
          fleet_name,
          std::move(traits),
          delay_threshold));

  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= node->_client_submit_trajectories->service_is_ready();
    ready &= node->_client_replace_trajectories->service_is_ready();
    ready &= node->_client_delay_trajectories->service_is_ready();

    if (ready)
      return node;
  }

  RCLCPP_INFO(
        node->get_logger(),
        "Timeout while trying to connect to traffic schedule");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::Duration delay_threshold)
: rclcpp::Node(fleet_name + "__read_only_fleet_adapter"),
  _fleet_name(fleet_name),
  _traits(std::move(traits)),
  _delay_threshold(delay_threshold)
{
  _client_submit_trajectories =
      create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  _client_delay_trajectories =
      create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  _client_replace_trajectories =
      create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  _fleet_state_subscription =
      create_subscription<FleetState>(
        FleetStateTopicName, rclcpp::SystemDefaultsQoS(),
        [&](FleetState::UniquePtr msg)
  {
    this->fleet_state_update(std::move(msg));
  });
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr state)
{
  if (state->name != _fleet_name)
    return;

  for(const auto& robot : state->robots)
  {
    const auto it = _schedule_entries.find(robot.name);
    if (it == _schedule_entries.end())
      submit_robot(robot);
    else
      update_robot(robot, it);
  }
}

//==============================================================================
void FleetAdapterNode::submit_robot(const RobotState& state)
{
  rmf_traffic::Trajectory trajectory = make_trajectory(state);

  SubmitTrajectories::Request msg;
  msg.trajectories.emplace_back(rmf_traffic_ros2::convert(trajectory));
  msg.fleet.fleet_id = _fleet_name;
  msg.fleet.type = rmf_traffic_msgs::msg::FleetProperties::TYPE_NO_CONTROL;

  using SubmitTrajectoriesFuture =
      rclcpp::Client<SubmitTrajectories>::SharedFuture;
  using namespace std::chrono_literals;

  _client_submit_trajectories->async_send_request(
        std::make_shared<SubmitTrajectories::Request>(std::move(msg)),
        [=](const SubmitTrajectoriesFuture future)
  {
    if (!future.valid() || std::future_status::ready != future.wait_for(0s))
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get response from schedule");
      return;
    }

    const auto response = *future.get();
    if (!response.error.empty())
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Error response from schedule: " + response.error);
      return;
    }

    const std::string robot_name = state.name;
    this->update_id(robot_name, response.current_version);
  });

  auto& entry = _schedule_entries[state.name];
  for (const auto& location : state.path)
    entry.path.push_back(location);
  entry.trajectory = std::move(trajectory);
  entry.dirty_id = true;
}

//==============================================================================
void FleetAdapterNode::update_robot(
    const RobotState& state,
    const ScheduleEntries::iterator& it)
{
  if(handle_delay(state, it))
    return;

  rmf_traffic::Trajectory trajectory = make_trajectory(state);

  ReplaceTrajectories::Request msg;
  msg.replace_ids.push_back(it->second.schedule_id);
  msg.trajectories.emplace_back(rmf_traffic_ros2::convert(trajectory));

  using ReplaceTrajectoriesFuture =
      rclcpp::Client<ReplaceTrajectories>::SharedFuture;
  using namespace std::chrono_literals;

  _client_replace_trajectories->async_send_request(
        std::make_shared<ReplaceTrajectories::Request>(std::move(msg)),
        [=](const ReplaceTrajectoriesFuture future)
  {
    if (!future.valid() || std::future_status::ready != future.wait_for(0s))
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get response from schedule");
      return;
    }

    const auto response = *future.get();
    if (!response.error.empty())
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Error response from schedule: " + response.error);
      return;
    }

    const std::string robot_name = state.name;
    this->update_id(robot_name, response.current_version);
  });

  auto& entry = it->second;
  entry.path.clear();
  for (const auto& location : state.path)
    entry.path.push_back(location);

  entry.trajectory = std::move(trajectory);
  entry.dirty_id = true;
}

//==============================================================================
bool FleetAdapterNode::handle_delay(
    const RobotState& state,
    const ScheduleEntries::iterator& it)
{
  auto& entry = it->second;

  if (entry.dirty_id)
  {
    // The current schedule ID is dirty and is waiting for an update, so it's
    // too soon to make more schedule changes to it. Hopefully this shouldn't
    // happen often, so we'll issue a warning about it.
    RCLCPP_DEBUG(
          get_logger(),
          "Schedule ID [" + std::to_string(entry.schedule_id) + "] for robot ["
          + state.name + "] is dirty. We cannot update it again until we get "
          "a reply from the schdule.");

    // We'll return true to avoid any further attempts to update the schedule.
    return true;
  }

  if (entry.path.size() < state.path.size())
  {
    // If the state has more points in its path than what is remembered from
    // before, then it must have a new path that it is following, so sending a
    // delay is not sufficient.
    return false;
  }

  for (std::size_t i=1; i <= state.path.size(); ++i)
  {
    const auto& l_state = state.path[state.path.size()-i];
    const auto& l_entry = entry.path[entry.path.size()-i];

    const Eigen::Vector3d p_state{l_state.x, l_state.y, l_state.yaw};
    const Eigen::Vector3d p_entry{l_entry.x, l_entry.y, l_entry.yaw};

    if ((p_state - p_entry).norm() > 1e-8)
      return false;
  }

  entry.path.clear();
  for (const auto& location : state.path)
    entry.path.push_back(location);

  const auto new_trajectory = make_trajectory(state);
  const auto time_difference =
      *new_trajectory.finish_time() - *entry.trajectory.finish_time();
  if (std::abs(time_difference.count()) < _delay_threshold.count())
  {
    // The difference between the current finishing time estimate and the
    // previous finishing time estimate is less than the threshold for reporting
    // a delay. This implies that the difference in the estimate may be an
    // artifact of the estimating and not indicative of a real delay.
    //
    // We will keep the current trajectory as it is in the schedule to avoid
    // needless schedule noise.
    return true;
  }

  if (time_difference.count() < 0)
  {
    // The AGV is ahead of schedule. This shouldn't generally happen because
    // there shouldn't be a situation where the robot moves faster than the
    // estimate. We will log this for debugging reference, but otherise ignore
    // it for now.
    RCLCPP_ERROR(
          get_logger(),
          "BUG: Robot [" + state.name + "] is unexpectedly ["
          + std::to_string(-rmf_traffic::time::to_seconds(time_difference))
          + "] seconds ahead of schedule. This should not happen; the real "
          + "robots should only ever be behind the predicted schedule.");

    // We'll return true to avoid a needless schedule replacement.
    return true;
  }

  // There was a considerable difference between the scheduled finishing time
  // estimate and the latest finishing time estimate, so we will notify the
  // schedule of a delay.
  const auto from_time =
      rmf_traffic_ros2::convert(state.location.t) - time_difference;

  const auto t_it = entry.trajectory.find(from_time);
  if (t_it == entry.trajectory.end())
  {
    // I can't think of a situation where this could happen, so let's report it
    // as an error and debug it later.
    RCLCPP_ERROR(
          get_logger(),
          "BUG: Robot [" + state.name + "] has a delay, but we cannot identify "
          "where its schedule should be pushed back. This should not happen; "
          "please report this.");

    // We'll return false so that the old trajectory can be replaced with the
    // new one, and hopefully everything keeps working okay.
    return false;
  }

  t_it->adjust_finish_times(time_difference);

  DelayTrajectories::Request msg;
  msg.delay = time_difference.count();
  msg.from_time = from_time.time_since_epoch().count();
  msg.delay_ids.push_back(entry.schedule_id);

  using DelayTrajectoriesFuture =
      rclcpp::Client<DelayTrajectories>::SharedFuture;
  using namespace std::chrono_literals;

  _client_delay_trajectories->async_send_request(
        std::make_shared<DelayTrajectories::Request>(std::move(msg)),
        [=](const DelayTrajectoriesFuture future)
  {
    // Think about how to refactor this logic to share it between this and the
    // other service requests.
    if (!future.valid() || std::future_status::ready != future.wait_for(0s))
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get response from schedule");
      return;
    }

    const auto response = *future.get();
    if (!response.error.empty())
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Error response from schedule: " + response.error);
      return;
    }

    const std::string robot_name = state.name;
    this->update_id(robot_name, response.current_version);
  });

  // Return true to indicate that the delay has been handled.
  return true;
}

//==============================================================================
rmf_traffic::Trajectory FleetAdapterNode::make_trajectory(
    const RobotState& state) const
{
  // TODO(MXG): Account for the multi-floor use case
  const std::string& map_name = state.location.level_name;

  std::vector<Eigen::Vector3d> positions;
  positions.push_back({state.location.x, state.location.y, state.location.yaw});
  for (const auto& location : state.path)
    positions.push_back({location.x, location.y, location.yaw});

  auto trajectory = rmf_traffic::agv::Interpolate::positions(
        map_name, _traits, rmf_traffic_ros2::convert(state.location.t),
        positions);

  if (trajectory.size() < 2)
  {
    const auto& segment = trajectory.front();
    const Eigen::Vector3d p = segment.get_finish_position();
    RCLCPP_WARN(
          get_logger(),
          "The path given for [" + state.name + "] resulted in a single-point "
          "trajectory. Specifying a 10-second waiting trajectory at ("
          + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ", "
          + std::to_string(p[2]) + ")");

    trajectory.insert(
          segment.get_finish_time() + std::chrono::seconds(10),
          _traits.get_profile(), p, Eigen::Vector3d::Zero());
  }

  return trajectory;
}

//==============================================================================
void FleetAdapterNode::update_id(const std::string& name, uint64_t id)
{
  const auto it = _schedule_entries.find(name);
  if (it == _schedule_entries.end())
  {
    RCLCPP_INFO(
          get_logger(),
          "[read_only::FleetAdapterNode] Updating unrecognized robot: " + name);
    return;
  }

  it->second.schedule_id = id;
  it->second.dirty_id = false;
}

} // namespace read_only
} // namespace rmf_fleet_adapter
