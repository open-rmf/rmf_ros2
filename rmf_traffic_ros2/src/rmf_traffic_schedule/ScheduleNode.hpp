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

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP

#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_wakeup.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectories.hpp>
#include <rmf_traffic_msgs/srv/replace_trajectories.hpp>
#include <rmf_traffic_msgs/srv/delay_trajectories.hpp>
#include <rmf_traffic_msgs/srv/erase_trajectories.hpp>

#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/mirror_update.h>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>

#include <unordered_map>

namespace rmf_traffic_schedule {

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:

  ScheduleNode();


private:

  using request_id_ptr = std::shared_ptr<rmw_request_id_t>;

  using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;
  using SubmitTrajectoriesService = rclcpp::Service<SubmitTrajectories>;

  void submit_trajectories(
      const request_id_ptr& request_header,
      const SubmitTrajectories::Request::SharedPtr& request,
      const SubmitTrajectories::Response::SharedPtr& response);

  SubmitTrajectoriesService::SharedPtr submit_trajectories_service;


  using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;
  using ReplaceTrajectoriesService = rclcpp::Service<ReplaceTrajectories>;

  void replace_trajectories(
      const request_id_ptr& request_header,
      const ReplaceTrajectories::Request::SharedPtr& request,
      const ReplaceTrajectories::Response::SharedPtr& response);

  ReplaceTrajectoriesService::SharedPtr replace_trajectories_service;


  using DelayTrajectories = rmf_traffic_msgs::srv::DelayTrajectories;
  using DelayTrajectoriesService = rclcpp::Service<DelayTrajectories>;

  void delay_trajectories(
      const request_id_ptr& request_header,
      const DelayTrajectories::Request::SharedPtr& request,
      const DelayTrajectories::Response::SharedPtr& response);

  DelayTrajectoriesService::SharedPtr delay_trajectories_service;


  using EraseTrajectories = rmf_traffic_msgs::srv::EraseTrajectories;
  using EraseTrajectoriesService = rclcpp::Service<EraseTrajectories>;

  void erase_trajectories(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const EraseTrajectories::Request::SharedPtr& request,
      const EraseTrajectories::Response::SharedPtr& response);

  EraseTrajectoriesService::SharedPtr erase_trajectories_service;


  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  void register_query(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const RegisterQuery::Request::SharedPtr& request,
      const RegisterQuery::Response::SharedPtr& response);

  RegisterQueryService::SharedPtr register_query_service;


  using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
  using UnregisterQueryService = rclcpp::Service<UnregisterQuery>;

  void unregister_query(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const UnregisterQuery::Request::SharedPtr& request,
      const UnregisterQuery::Response::SharedPtr& response);

  UnregisterQueryService::SharedPtr unregister_query_service;


  using MirrorUpdate = rmf_traffic_msgs::srv::MirrorUpdate;
  using MirrorUpdateService = rclcpp::Service<MirrorUpdate>;

  void mirror_update(
      const std::shared_ptr<rmw_request_id_t>& request_header,
      const MirrorUpdate::Request::SharedPtr& request,
      const MirrorUpdate::Response::SharedPtr& response);

  MirrorUpdateService::SharedPtr mirror_update_service;


  using MirrorWakeup = rmf_traffic_msgs::msg::MirrorWakeup;
  using MirrorWakeupPublisher = rclcpp::Publisher<MirrorWakeup>;
  MirrorWakeupPublisher::SharedPtr mirror_wakeup_publisher;

  void wakeup_mirrors() const;

  rmf_traffic::schedule::Database database;

  using QueryMap =
      std::unordered_map<uint64_t, rmf_traffic::schedule::Query::Spacetime>;
  // TODO(MXG): Have a way to make query registrations expire after they have
  // not been used for some set amount of time (e.g. 24 hours? 48 hours?).
  std::size_t last_query_id = 0;
  QueryMap registered_queries;

};

} // namespace rmf_traffic_schedule

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
