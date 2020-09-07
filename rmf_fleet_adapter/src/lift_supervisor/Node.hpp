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

#ifndef SRC__LIFT_SUPERVISOR__NODE_HPP
#define SRC__LIFT_SUPERVISOR__NODE_HPP

#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

#include <std_msgs/msg/bool.hpp>

#include <rclcpp/node.hpp>

#include <unordered_map>
#include <unordered_set>

namespace rmf_fleet_adapter {
namespace lift_supervisor {

//==============================================================================
class Node : public rclcpp::Node
{
public:

  Node();

private:

  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftRequestPub = rclcpp::Publisher<LiftRequest>;
  LiftRequestPub::SharedPtr _lift_request_pub;

  using LiftRequestSub = rclcpp::Subscription<LiftRequest>;
  LiftRequestSub::SharedPtr _adapter_lift_request_sub;
  void _adapter_lift_request_update(LiftRequest::UniquePtr msg);

  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftStateSub = rclcpp::Subscription<LiftState>;
  LiftStateSub::SharedPtr _lift_state_sub;
  void _lift_state_update(LiftState::UniquePtr msg);

  using EmergencyNotice = std_msgs::msg::Bool;
  using EmergencyNoticePub = rclcpp::Publisher<EmergencyNotice>;
  EmergencyNoticePub::SharedPtr _emergency_notice_pub;

  std::unordered_map<std::string, LiftRequest::UniquePtr> _active_sessions;
};

} // namespace lift_supervisor
} // namespace rmf_fleet_adapter

#endif // SRC__LIFT_SUPERVISOR__NODE_HPP
