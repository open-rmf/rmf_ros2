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

#include <rmf_traffic_ros2/schedule/internal_Node.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

class MissingQueryScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  MissingQueryScheduleNode(
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    QueryMap registered_queries_,
    QuerySubscriberCountMap registered_query_subscriber_counts,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : ScheduleNode(database_, registered_queries_, registered_query_subscriber_counts, options)
  {
    RCLCPP_WARN(get_logger(), "MissingQuery starting 1");
    setup_query_services();
  }

  MissingQueryScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(options)
  {
    RCLCPP_WARN(get_logger(), "MissingQuery starting 2");
    setup_query_services();
  }

  void setup_query_services()
  {
    RCLCPP_WARN(get_logger(), "MissingQuery setup query services");
    register_query_service =
      create_service<RegisterQuery>(
      rmf_traffic_ros2::RegisterQueryServiceName,
      [=](const std::shared_ptr<rmw_request_id_t> request_header,
      const RegisterQuery::Request::SharedPtr request,
      const RegisterQuery::Response::SharedPtr response)
      { this->register_query(request_header, request, response); });

    unregister_query_service =
      create_service<UnregisterQuery>(
      rmf_traffic_ros2::UnregisterQueryServiceName,
      [=](const std::shared_ptr<rmw_request_id_t> request_header,
      const UnregisterQuery::Request::SharedPtr request,
      const UnregisterQuery::Response::SharedPtr response)
      { this->unregister_query(request_header, request, response); });
  }

  void register_query(
    const request_id_ptr& request_header,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response)
  {
    RCLCPP_WARN(get_logger(), "MissingQuery register_query");
    ScheduleNode::register_query(request_header, request, response);
    RCLCPP_WARN(get_logger(), "MissingQuery register_query done");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissingQueryScheduleNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
