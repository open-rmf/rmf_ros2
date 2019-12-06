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

#include "ScheduleManager.hpp"

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
using ScheduleConflict = rmf_traffic_msgs::msg::ScheduleConflict;
class ScheduleManager::ConflictListener : public Listener<ScheduleConflict>
{
public:

  ConflictListener(ScheduleManager* parent)
  : _parent(parent)
  {
    // Do nothing
  }

  void receive(const ScheduleConflict& msg) final
  {
    if (_parent->_waiting_for_schedule)
      return;

    if (msg.version <= _parent->_last_revised_version)
      return;

    const auto& schedule_ids = _parent->_schedule_ids;

    for (const auto id : msg.indices)
    {
      if (std::find(schedule_ids.begin(), schedule_ids.end(), id)
          == schedule_ids.end())
        continue;

      _parent->_have_conflict = true;
      _parent->_last_conflict_version = msg.version;
      return _parent->_revision_callback();
    }
  }

  ScheduleManager* const _parent;
};

//==============================================================================
ScheduleConnections ScheduleConnections::make(rclcpp::Node& node)
{
  ScheduleConnections connections;

  connections.submit_trajectories = node.create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  connections.delay_trajectories = node.create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  connections.replace_trajectories = node.create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  connections.erase_trajectories = node.create_client<EraseTrajectories>(
        rmf_traffic_ros2::EraseTrajectoriesSrvName);

  connections.resolve_conflicts = node.create_client<ResolveConflicts>(
        rmf_traffic_ros2::ResolveConflictsSrvName);

  return connections;
}

//==============================================================================
bool ScheduleConnections::ready() const
{
  bool ready = true;
  ready &= submit_trajectories->service_is_ready();
  ready &= delay_trajectories->service_is_ready();
  ready &= replace_trajectories->service_is_ready();
  ready &= erase_trajectories->service_is_ready();

  return ready;
}

//==============================================================================
ScheduleManager::ScheduleManager(
    rmf_fleet_adapter::ScheduleConnections* connections,
    rmf_traffic_msgs::msg::FleetProperties properties,
    std::function<void()> revision_callback)
: _connections(connections),
  _properties(std::move(properties)),
  _revision_callback(std::move(revision_callback))
{
  // Do nothing
}

namespace {
//==============================================================================
std::vector<rmf_traffic_msgs::msg::Trajectory> convert(
    const std::vector<rmf_traffic::Trajectory>& trajectories)
{
  std::vector<rmf_traffic_msgs::msg::Trajectory> output;
  output.reserve(trajectories.size());
  for (const auto& trajectory : trajectories)
    output.emplace_back(rmf_traffic_ros2::convert(trajectory));

  return output;
}

} // anonymous namespace

//==============================================================================
void ScheduleManager::push_trajectories(
    const std::vector<rmf_traffic::Trajectory>& trajectories,
    std::function<void()> approval_callback)
{
  // TODO(MXG): Be smarter here. If there are no trajectories then erase the
  // current schedule? Or have the robot stand in place?
  if (trajectories.empty())
    approval_callback();

  if (_waiting_for_schedule)
  {
    // If any delays were queued on a previous trajectory, we should throw them
    // all out
    _queued_delays.clear();

    _queued_change =
        [this, approval_cb{std::move(approval_callback)}, trajectories]()
    {
      push_trajectories(trajectories, std::move(approval_cb));
    };

    return;
  }

  _waiting_for_schedule = true;

  if (_have_conflict)
    return resolve_trajectories(trajectories, std::move(approval_callback));

  if (_schedule_ids.empty())
    return submit_trajectories(trajectories, std::move(approval_callback));

  return replace_trajectories(trajectories, std::move(approval_callback));
}

//==============================================================================
void ScheduleManager::push_delay(
    const rmf_traffic::Duration duration,
    const rmf_traffic::Time from_time)
{
  if (_waiting_for_schedule)
  {
    _queued_delays.push_back([=](){ push_delay(duration, from_time); });
    return;
  }

  using DelayTrajectories = rmf_traffic_msgs::srv::DelayTrajectories;

  const auto& delay = _connections->delay_trajectories;
  DelayTrajectories::Request request;

  request.delay_ids = _schedule_ids;
  request.delay = duration.count();
  request.from_time = from_time.time_since_epoch().count();

  _schedule_ids.clear();
  _waiting_for_schedule = true;

  delay->async_send_request(
        std::make_shared<DelayTrajectories::Request>(
          std::move(request)),
        [this](rclcpp::Client<DelayTrajectories>::SharedFuture future)
  {
    const auto response = future.get();

    _waiting_for_schedule = false;

    if (!response->error.empty())
      throw std::runtime_error(response->error);

    for (auto i = response->original_version+1;
         i <= response->current_version; ++i)
    {
      _schedule_ids.push_back(i);
    }

    process_queues();
  });
}

//==============================================================================
bool ScheduleManager::waiting() const
{
  return _waiting_for_schedule;
}

//==============================================================================
const std::vector<rmf_traffic::schedule::Version>& ScheduleManager::ids() const
{
  return _schedule_ids;
}

//==============================================================================
ScheduleManager::~ScheduleManager()
{
  if (!_schedule_ids.empty())
  {
    using EraseTrajectories = rmf_traffic_msgs::srv::EraseTrajectories;

    const auto& erase = _connections->erase_trajectories;
    EraseTrajectories::Request request;
    request.erase_ids = _schedule_ids;

    _schedule_ids.clear();

    erase->async_send_request(
          std::make_shared<EraseTrajectories::Request>(std::move(request)));
  }
}

//==============================================================================
void ScheduleManager::submit_trajectories(
    const TrajectorySet& trajectories,
    std::function<void()> approval_callback)
{
  using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;

  const auto& submit = _connections->submit_trajectories;
  SubmitTrajectories::Request request;

  request.fleet = _properties;
  request.trajectories = convert(trajectories);

  _waiting_for_schedule = true;

  submit->async_send_request(
        std::make_shared<SubmitTrajectories::Request>(
          std::move(request)),
        [this, approval_cb{std::move(approval_callback)}](
        rclcpp::Client<SubmitTrajectories>::SharedFuture future)
  {
    const auto response = future.get();

    _waiting_for_schedule = false;

    if (!response->error.empty())
      throw std::runtime_error(response->error);

    if (response->accepted)
    {
      for (auto i = response->original_version+1;
           i <= response->current_version; ++i)
      {
        _schedule_ids.push_back(i);
      }

      if (process_queues())
        return;

      approval_cb();
      return;
    }

    if (process_queues())
      return;

    _revision_callback();
  });
}

//==============================================================================
void ScheduleManager::replace_trajectories(
    const TrajectorySet& trajectories,
    std::function<void()> approval_callback)
{
  using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;

  const auto& replace = _connections->replace_trajectories;
  ReplaceTrajectories::Request request;

  request.replace_ids = _schedule_ids;
  request.trajectories = convert(trajectories);

  _waiting_for_schedule = true;

  _schedule_ids.clear();
  replace->async_send_request(
        std::make_shared<ReplaceTrajectories::Request>(std::move(request)),
        [this](rclcpp::Client<ReplaceTrajectories>::SharedFuture future)
  {
    const auto response = future.get();

    _waiting_for_schedule = false;

    if (!response->error.empty())
      throw std::runtime_error(response->error);

    for (auto i = response->original_version+1;
         i <= response->latest_trajectory_version; ++i)
    {
      _schedule_ids.push_back(i);
    }

    process_queues();
  });

  // We don't need to wait for approval for plan replacements
  approval_callback();
}

//==============================================================================
void ScheduleManager::resolve_trajectories(
    const TrajectorySet& trajectories,
    std::function<void()> approval_callback)
{
  _last_revised_version = _last_conflict_version;

  using ResolveConflicts = rmf_traffic_msgs::srv::ResolveConflicts;
  ResolveConflicts::Request request;
  request.resolve_ids = _schedule_ids;
  request.trajectories = convert(trajectories);
  request.conflict_version = _last_revised_version;

  const auto& resolve = _connections->resolve_conflicts;

  resolve->async_send_request(
        std::make_shared<ResolveConflicts::Request>(std::move(request)),
        [this, approval_cb{std::move(approval_callback)}](
        rclcpp::Client<ResolveConflicts>::SharedFuture future)
  {
    const auto response = future.get();

    _waiting_for_schedule = false;

    if (!response->error.empty())
      throw std::runtime_error(response->error);

    if (!response->accepted)
    {
      // The conflict was resolved by someone else, so we will quit
      return;
    }

    _schedule_ids.clear();
    for (auto i = response->original_version+1;
         i <= response->latest_trajectory_version; ++i)
    {
      _schedule_ids.push_back(i);
    }

    if (process_queues())
      return;

    approval_cb();
  });
}

//==============================================================================
bool ScheduleManager::process_queues()
{
  if (_queued_change)
  {
    _queued_change();
    _queued_change = nullptr;
    return true;
  }

  if (!_queued_delays.empty())
  {
    const auto queued_delay = _queued_delays.front();
    _queued_delays.erase(_queued_delays.begin());
    queued_delay();
    return true;
  }

  return false;
}

} // namespace rmf_fleet_adapter
