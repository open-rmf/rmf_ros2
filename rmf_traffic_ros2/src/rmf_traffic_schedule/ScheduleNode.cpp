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

#include "ScheduleNode.hpp"

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>

#include <rmf_traffic/Conflict.hpp>

namespace rmf_traffic_schedule {

//==============================================================================
ScheduleNode::ScheduleNode()
  : Node("rmf_traffic_schedule_node")
{
  // TODO(MXG): As soon as possible, all of these services should be made
  // multi-threaded so they can be parallel processed.

  submit_trajectory_service =
      create_service<rmf_traffic_msgs::srv::SubmitTrajectory>(
        "rmf_traffic/submit_trajectory",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const SubmitTrajectory::Request::SharedPtr request,
            const SubmitTrajectory::Response::SharedPtr response)
        { this->submit_trajectory(request_header, request, response); });

  erase_schedule_service =
      create_service<EraseSchedule>(
        "rmf_traffic/erase_schedule",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const EraseSchedule::Request::SharedPtr request,
            const EraseSchedule::Response::SharedPtr response)
        { this->erase_schedule(request_header, request, response); });

  register_query_service =
      create_service<RegisterQuery>(
        "rmf_traffic/register_query",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const RegisterQuery::Request::SharedPtr request,
            const RegisterQuery::Response::SharedPtr response)
        { this->register_query(request_header, request, response); });

  unregister_query_service =
      create_service<UnregisterQuery>(
        "rmf_traffic/unregister_query",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const UnregisterQuery::Request::SharedPtr request,
            const UnregisterQuery::Response::SharedPtr response)
        { this->unregister_query(request_header, request, response); });

  mirror_update_service =
      create_service<MirrorUpdate>(
        "rmf_traffic/mirror_update",
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const MirrorUpdate::Request::SharedPtr request,
            const MirrorUpdate::Response::SharedPtr response)
        { this->mirror_update(request_header, request, response); });
}

//==============================================================================
void insert_conflicts(
    const std::size_t i,
    const std::vector<rmf_traffic::ConflictData>& conflicts,
    rmf_traffic_msgs::srv::SubmitTrajectory::Response& response)
{
  if(!conflicts.empty())
  {
    response.accepted = false;
    for(const auto& c : conflicts)
    {
      rmf_traffic_msgs::msg::ScheduleConflict conflict_msg;
      conflict_msg.index = i;
      conflict_msg.time = c.get_time().time_since_epoch().count();
      response.conflicts.emplace_back(std::move(conflict_msg));
    }
  }
}

//==============================================================================
void ScheduleNode::submit_trajectory(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const SubmitTrajectory::Request::SharedPtr& request,
    const SubmitTrajectory::Response::SharedPtr& response)
{
  response->accepted = true;
  response->current_version = database.latest_version();
  response->original_version = response->current_version;

  std::vector<rmf_traffic::Trajectory> requested_trajectories;
  requested_trajectories.reserve(request->trajectories.size());
  for(std::size_t i=0; i < request->trajectories.size(); ++i)
  {
    const auto& msg = request->trajectories[i];

    bool valid_trajectory = true;
    rmf_traffic::Trajectory requested_trajectory =
        [&]() -> rmf_traffic::Trajectory
    {
      try
      {
        return rmf_traffic_ros2::convert(msg);
      }
      catch(const std::exception& e)
      {
        valid_trajectory = false;
        response->accepted = false;
        response->error = e.what();
      }

      return {""};
    }();

    if(!valid_trajectory)
      return;

    if(requested_trajectory.size() < 2)
    {
      response->error = "Invalid trajectory at index [" + std::to_string(i)
          + "]: Only [" + std::to_string(requested_trajectory.size())
          + "] segments; minimum is 2.";
      return;
    }

    const auto view = database.query(rmf_traffic::schedule::make_query(
                     {requested_trajectory.get_map_name()},
                     requested_trajectory.start_time(),
                     requested_trajectory.finish_time()));

    for(const auto& scheduled_trajectory : view)
    {
      if(*requested_trajectory.finish_time() < *scheduled_trajectory.start_time())
        continue;

      if(*scheduled_trajectory.finish_time() < *requested_trajectory.start_time())
        continue;

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            requested_trajectory, scheduled_trajectory);

      insert_conflicts(i, conflicts, *response);
    }

    requested_trajectories.emplace_back(std::move(requested_trajectory));
  }

  if(!response->accepted)
    return;

  // TODO(MXG): Should there be constraints on trajectory submissions to avoid
  // this kind of check? Like each submission can only refer to one vehicle at
  // a time, and therefore we should never need to test these trajectories for
  // conflicts with each other?
  for(std::size_t i=0; i < requested_trajectories.size()-1; ++i)
  {
    for(std::size_t j=i+1; j < requested_trajectories.size(); ++j)
    {
      const auto conflicts = rmf_traffic::DetectConflict::between(
            requested_trajectories[i],
            requested_trajectories[j]);
      insert_conflicts(i, conflicts, *response);
    }
  }

  if(!response->accepted)
    return;

  for(auto&& request : requested_trajectories)
    database.insert(std::move(request));

  response->current_version = database.latest_version();
}

//==============================================================================
void ScheduleNode::erase_schedule(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const EraseSchedule::Request::SharedPtr& request,
    const EraseSchedule::Response::SharedPtr& response)
{
  for(const uint64_t id : request->erase_ids)
  {
    database.erase(id);
  }

  response->version = database.latest_version();
}

//==============================================================================
void ScheduleNode::register_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response)
{
  uint64_t query_id = last_query_id;
  uint64_t attempts = 0;
  do
  {
    ++query_id;
    ++attempts;
    if(attempts == std::numeric_limits<uint64_t>::max())
    {
      // I suspect a compute would run out of RAM before we reach this point,
      // but there's no harm in double-checking.
      response->error = "No more space for additional queries to be registered";
      return;
    }
  } while(registered_queries.find(query_id) != registered_queries.end());

  last_query_id = query_id;
  registered_queries.insert(
        std::make_pair(query_id, rmf_traffic_ros2::convert(request->query)));
}

//==============================================================================
void ScheduleNode::unregister_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const UnregisterQuery::Request::SharedPtr& request,
    const UnregisterQuery::Response::SharedPtr& response)
{
  const auto it = registered_queries.find(request->query_id);
  if(it == registered_queries.end())
  {
    response->error = "No query found with the id ["
        + std::to_string(request->query_id) + "]";
    response->confirmation = false;
    return;
  }

  registered_queries.erase(it);
  response->confirmation = true;
}

//==============================================================================
void ScheduleNode::mirror_update(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const MirrorUpdate::Request::SharedPtr& request,
    const MirrorUpdate::Response::SharedPtr& response)
{
  const auto query_it = registered_queries.find(request->query_id);
  if(query_it == registered_queries.end())
  {
    response->error = "Unrecognized query_id: "
        + std::to_string(request->query_id);
    return;
  }

  auto query = rmf_traffic::schedule::make_query(request->last_version);
  query.spacetime() = query_it->second;

  response->patch = rmf_traffic_ros2::convert(database.changes(query));
}

} // namespace rmf_traffic_schedule
