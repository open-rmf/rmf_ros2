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

#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_fleet_adapter {
namespace lift_supervisor {

//==============================================================================
Node::Node()
: rclcpp::Node("rmf_lift_supervisor")
{
  const auto default_qos = rclcpp::SystemDefaultsQoS();

  _lift_request_pub = create_publisher<LiftRequest>(
    FinalLiftRequestTopicName, default_qos);

  _adapter_lift_request_sub = create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName, default_qos,
    [&](LiftRequest::UniquePtr msg)
    {
      _adapter_lift_request_update(std::move(msg));
    });

  _lift_state_sub = create_subscription<LiftState>(
    LiftStateTopicName, default_qos,
    [&](LiftState::UniquePtr msg)
    {
      _lift_state_update(std::move(msg));
    });

  _emergency_notice_pub = create_publisher<EmergencyNotice>(
    rmf_traffic_ros2::EmergencyTopicName, default_qos);
}

//==============================================================================
void Node::_adapter_lift_request_update(LiftRequest::UniquePtr msg)
{
  auto& curr_request = _active_sessions.insert(
    std::make_pair(msg->lift_name, nullptr)).first->second;

  if (curr_request)
  {
    if (curr_request->session_id == msg->session_id)
    {
      if (msg->request_type != LiftRequest::REQUEST_END_SESSION)
        curr_request = std::move(msg);
      else
      {
        _lift_request_pub->publish(*msg);
        curr_request = nullptr;
      }
    }
  }
  else
  {
    if (msg->request_type != LiftRequest::REQUEST_END_SESSION)
    {
      curr_request = std::move(msg);
    }
  }

  // TODO(MXG): Make this more intelligent by scheduling the lift
}

//==============================================================================
void Node::_lift_state_update(LiftState::UniquePtr msg)
{
  auto& lift_request = _active_sessions.insert(
    std::make_pair(msg->lift_name, nullptr)).first->second;

  if (lift_request)
  {
    if ((lift_request->destination_floor != msg->current_floor) ||
      (lift_request->door_state != msg->door_state))
      _lift_request_pub->publish(*lift_request);
  }

  // For now, we do not need to publish this.

//  std_msgs::msg::Bool emergency_msg;
//  emergency_msg.data = false;

//  if (LiftState::MODE_FIRE == msg->current_mode
//      || LiftState::MODE_EMERGENCY == msg->current_mode)
//  {
//    emergency_msg.data = true;
//  }

//  _emergency_notice_pub->publish(emergency_msg);
}

} // namespace lift_supervisor
} // namespace rmf_fleet_adapter
