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

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include <rmf_fleet_msgs/srv/lift_clearance.hpp>
#include <std_msgs/msg/bool.hpp>

class ExperimentalLiftWatchdogNode : public rclcpp::Node
{
public:

  using Service = rmf_fleet_msgs::srv::LiftClearance;
  using Request = Service::Request;
  using Response = Service::Response;

  ExperimentalLiftWatchdogNode()
  : rclcpp::Node("experimental_lift_watchdog")
  {
    service = create_service<rmf_fleet_msgs::srv::LiftClearance>(
      "experimental_lift_watchdog",
      [=](
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<Request> request,
        const std::shared_ptr<Response> response)
      {
        service_callback(header, request, response);
      });

    permission_subscription = create_subscription<std_msgs::msg::Bool>(
      "experimental_lift_permission", rclcpp::SystemDefaultsQoS(),
      [=](std::shared_ptr<std_msgs::msg::Bool> msg)
      {
        permission_callback(msg);
      });
  }

  void service_callback(
    const std::shared_ptr<rmw_request_id_t>& /*header*/,
    const std::shared_ptr<Request>& request,
    const std::shared_ptr<Response>& response)
  {
    if (permission)
    {
      response->decision =
        rmf_fleet_msgs::srv::LiftClearance::Response::DECISION_CLEAR;

      RCLCPP_INFO(
        get_logger(),
        "Robot [%s] is clear to enter lift [%s]",
        request->robot_name.c_str(), request->lift_name.c_str());
    }
    else
    {
      response->decision =
        rmf_fleet_msgs::srv::LiftClearance::Response::DECISION_CROWDED;

      RCLCPP_INFO(
        get_logger(),
        "Robot [%s] cannot enter lift [%s]",
        request->robot_name.c_str(), request->lift_name.c_str());
    }
  }

  void permission_callback(
    const std::shared_ptr<std_msgs::msg::Bool>& msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "Set Lift watchdog permission to [%s]", msg->data ? "true" : "false");
    permission = msg->data;
  }

  rclcpp::Service<Service>::SharedPtr service;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_subscription;
  bool permission = true;

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentalLiftWatchdogNode>());
  rclcpp::shutdown();
}
