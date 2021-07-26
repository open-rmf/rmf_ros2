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

#include <chrono>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/schedule/internal_Node.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class WrongQueryScheduleNode : public rmf_traffic_ros2::schedule::ScheduleNode
{
public:
  WrongQueryScheduleNode(const rclcpp::NodeOptions& options)
    : ScheduleNode(0, options, ScheduleNode::no_automatic_setup)
  {
  }

  void setup_query_services() override
  {
    register_query_service =
      create_service<RegisterQuery>(
      rmf_traffic_ros2::RegisterQueryServiceName,
      [=](const std::shared_ptr<rmw_request_id_t> request_header,
      const RegisterQuery::Request::SharedPtr request,
      const RegisterQuery::Response::SharedPtr response)
      { this->register_query(request_header, request, response); });
  }

  void register_query(
    const request_id_ptr& request_header,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response) override
  {
    ScheduleNode::register_query(request_header, request, response);
    // Change the query that was just stored so it is different from what was
    // registered
    if (is_first_query)
    {
      RCLCPP_WARN(
        get_logger(),
        "Fiddling with first query to cause a registered query mismatch");
      rmf_traffic::schedule::Query query =
        rmf_traffic::schedule::make_query({});

      auto region = rmf_traffic::Region{"L1", {}};
      const auto circle = rmf_traffic::geometry::Circle(1000);
      const auto final_circle =
        rmf_traffic::geometry::make_final_convex(circle);
      Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
      region.push_back(rmf_traffic::geometry::Space{final_circle, tf});
      query.spacetime().regions()->push_back(region);

      // Replace the stored query
      ScheduleNode::register_query(response->query_id, query);

      is_first_query = false;
    }
  }

  bool is_first_query = true;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WrongQueryScheduleNode>(rclcpp::NodeOptions());
  node->setup(rmf_traffic_ros2::schedule::ScheduleNode::QueryMap());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
