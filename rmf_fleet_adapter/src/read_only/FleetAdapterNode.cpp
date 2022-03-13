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

#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/executors.hpp>

#include "../rmf_fleet_adapter/load_param.hpp"

#include "../rmf_fleet_adapter/make_trajectory.hpp"

namespace rmf_fleet_adapter {
namespace read_only {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make()
{
  auto node = std::shared_ptr<FleetAdapterNode>(new FleetAdapterNode);

  const auto wait_time =
    get_parameter_or_default_time(*node, "discovery_timeout", 10.0);

  node->_delay_threshold =
    get_parameter_or_default_time(*node, "delay_threshold", 5.0);

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
    node, rmf_traffic::schedule::query_all());

  node->_writer = rmf_traffic_ros2::schedule::Writer::make(node);

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= node->_writer->ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      node->_mirror = mirror_future.get();
      node->_negotiation = rmf_traffic_ros2::schedule::Negotiation(
        *node, node->_mirror->view());

      // Don't subscribe until everything else is ready
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

  RCLCPP_INFO(
    node->get_logger(),
    "Timeout while trying to connect to traffic schedule");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::ScheduleEntry::ScheduleEntry(
  FleetAdapterNode* node,
  std::string name,
  std::mutex& async_mutex)
{
  rmf_traffic::schedule::ParticipantDescription description{
    std::move(name),
    node->_fleet_name,
    rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
    node->_traits.profile()
  };

  async_make_schedule_manager(
    *node, *node->_writer, &node->_negotiation.value(), std::move(description),
    [this, node](ScheduleManager manager)
    {
      this->schedule = std::move(manager);

      this->schedule->set_negotiator(
        [this, node](
          const rmf_traffic::schedule::Negotiation::Table::ViewerPtr& table,
          const rmf_traffic::schedule::Negotiator::ResponderPtr& responder)
        {
          const auto itinerary = this->schedule->participant().itinerary();

          const auto proposals = table->base_proposals();
          const auto& profile = this->schedule->description().profile();
          for (const auto& p : proposals)
          {
            const auto other_participant =
            node->_mirror->view()->get_participant(p.participant);

            if (!other_participant)
            {
              // TODO(MXG): This is lazy and sloppy. For now we just reject the
              // negotiation if we don't know about the other participant. In
              // the future, we should have a way to wait until the participant
              // information is available.
              assert(false);
              return responder->forfeit({});
            }

            const auto& other_profile = other_participant->profile();
            for (const auto& other_route : p.itinerary)
            {
              for (const auto& route : itinerary)
              {
                if (route.map() != other_route.map())
                  continue;

                if (rmf_traffic::DetectConflict::between(
                  profile, route.trajectory(), nullptr,
                  other_profile, other_route.trajectory(), nullptr))
                {
                  rmf_traffic::schedule::Itinerary alternative;
                  alternative.reserve(itinerary.size());
                  for (const auto& r : itinerary)
                    alternative.emplace_back(r);

                  return responder->reject({std::move(alternative)});
                }
              }
            }
          }

          std::vector<rmf_traffic::Route> submission;
          submission.reserve(itinerary.size());
          for (const auto& r : itinerary)
            submission.push_back(r);

          return responder->submit(
            this->schedule->participant().current_plan_id(),
            std::move(submission));
        });
    }, async_mutex);
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
  // Do nothing. Everything else is initialized in make()
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr state)
{
  if (ignore_fleet(state->name))
    return;

  for (const auto& robot : state->robots)
  {
    const auto insertion = _schedule_entries.insert(
      std::make_pair(robot.name, nullptr));

    if (insertion.second)
      register_robot(robot, insertion.first);
    else if (insertion.first->second->schedule)
      update_robot(robot, insertion.first);
  }
}

//==============================================================================
void FleetAdapterNode::push_route(
  const RobotState& state,
  const ScheduleEntries::iterator& it)
{
  it->second->path.clear();
  for (const auto& location : state.path)
    it->second->path.push_back(location);

  it->second->cumulative_delay = std::chrono::seconds(0);
  it->second->route = make_route(state, _traits, it->second->sitting);
  it->second->schedule->push_routes({*it->second->route});
}

//==============================================================================
void FleetAdapterNode::register_robot(
  const RobotState& state,
  const ScheduleEntries::iterator& it)
{
  it->second = std::make_unique<ScheduleEntry>(this, state.name, _async_mutex);
  // TODO(MXG): We could consider queuing up the current route of this robot
  // so that it will be broadcasted as soon as the participant is registered,
  // but it's simpler to just wait until the next state message is received,
  // and then modify the schedule at that point.
}

//==============================================================================
void FleetAdapterNode::update_robot(
  const RobotState& state,
  const ScheduleEntries::iterator& it)
{
  if (handle_delay(state, it))
    return;

  push_route(state, it);
}

//==============================================================================
bool FleetAdapterNode::handle_delay(
  const RobotState& state,
  const ScheduleEntries::iterator& it)
{
  if (!it->second->route)
    return false;

  auto& entry = *it->second;

  if (entry.path.size() < state.path.size())
  {
    // If the state has more points in its path than what is remembered from
    // before, then it must have a new path that it is following, so sending a
    // delay is not sufficient.
    return false;
  }

  for (std::size_t i = 1; i <= state.path.size(); ++i)
  {
    const auto& l_state = state.path[state.path.size()-i];
    const auto& l_entry = entry.path[entry.path.size()-i];

    const Eigen::Vector3d p_state{l_state.x, l_state.y, l_state.yaw};
    const Eigen::Vector3d p_entry{l_entry.x, l_entry.y, l_entry.yaw};

    // TODO(MXG): Make this threshold configurable
    if ((p_state - p_entry).norm() > 1e-8)
      return false;
  }

  entry.path.clear();
  for (const auto& location : state.path)
    entry.path.push_back(location);

  bool sitting = false;
  auto new_trajectory = make_trajectory(state, _traits, sitting);

  if (entry.sitting && sitting)
  {
    // Check if the robot is still sitting in the same location.
    const Eigen::Vector3d p_entry =
      entry.route->trajectory().back().position();
    assert(
      (entry.route->trajectory().front().position() - p_entry).norm() <
      1e-12);

    const auto& l_state = state.location;
    const Eigen::Vector3d p_state{l_state.x, l_state.y, l_state.yaw};

    // TODO(MXG): Make this threshold configurable
    if ((p_state.block<2, 1>(0, 0) - p_entry.block<2, 1>(0, 0)).norm() > 0.05)
      return false;

    // TODO(MXG): Make this threshold configurable
    if (std::abs(p_state[2] - p_entry[2]) > 10.0*M_PI/180.0)
      return false;

    // Every 3 seconds we'll extend the finish time for the trajectory
    // TODO(MXG): Make these parameters configurable
    const auto current_time = rmf_traffic_ros2::convert(state.location.t);
    const auto next_finish_time = current_time + std::chrono::seconds(10);
    const auto delay = next_finish_time -
      *entry.route->trajectory().finish_time();
    if (delay > std::chrono::seconds(1))
    {
      entry.cumulative_delay += delay;
      if (entry.cumulative_delay >= MaxCumulativeDelay)
        return false;

      entry.route->trajectory().back().adjust_times(delay);
      entry.schedule->push_delay(delay);
    }

    return true;
  }
  else if (sitting)
  {
    // The new robot state indicates the robot should be sitting, but it is not
    // already considered to be sitting. Therefore we will kick it back to
    // push_trajectory() to put a new sitting trajectory on the schedule.
    return false;
  }

  const auto time_difference =
    *new_trajectory.finish_time() - *entry.route->trajectory().finish_time();

//  std::cout << "Calculating delay: ["
//            << rmf_traffic::time::to_seconds(time_difference) << "]" << std::endl;

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

  entry.cumulative_delay += time_difference;
  if (entry.cumulative_delay >= MaxCumulativeDelay)
    return false;

  // There was a considerable difference between the scheduled finishing time
  // estimate and the latest finishing time estimate, so we will notify the
  // schedule of a delay.
  const auto from_time =
    rmf_traffic_ros2::convert(state.location.t) - time_difference;

  const auto t_it = entry.route->trajectory().find(from_time);
  if (t_it == entry.route->trajectory().end())
  {
    const auto t_start = *entry.route->trajectory().start_time();
    if (from_time <= t_start)
    {
      // This is okay. It just means we will push back the entire trajectory
      // in the schedule.
      entry.route->trajectory().front().adjust_times(time_difference);
    }
    else
    {
      // I can't think of a situation where this could happen, so let's report
      // it as an error and debug it later.
      const auto t_start =
        entry.route->trajectory().start_time()->time_since_epoch().count();
      const auto t_finish =
        entry.route->trajectory().finish_time()->time_since_epoch().count();

      RCLCPP_ERROR(
        get_logger(),
        "BUG: Robot [%s] has a delay which starts from [%s], but "
        "we cannot identify where its schedule should be pushed back ["
        "%s --> %s]. This should not happen; please report this.",
        state.name.c_str(),
        std::to_string(from_time.time_since_epoch().count()).c_str(),
        std::to_string(t_start).c_str(),
        std::to_string(t_finish).c_str());

      // We'll return false so that the old trajectory can be replaced with the
      // new one, and hopefully everything keeps working okay.
      return false;
    }
  }
  else
  {
    if (time_difference.count() < 0)
      entry.route->trajectory().begin()->adjust_times(time_difference);
    else
      t_it->adjust_times(time_difference);
  }

  entry.schedule->push_delay(time_difference);

  // Return true to indicate that the delay has been handled.
  return true;
}

} // namespace read_only
} // namespace rmf_fleet_adapter
