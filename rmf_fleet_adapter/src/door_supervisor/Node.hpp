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

#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

#include <string>
#include <unordered_map>

namespace rmf_fleet_adapter {
namespace door_supervisor {

//==============================================================================
class Node : public rclcpp::Node
{
public:

  Node();

private:

  using DoorMode = rmf_door_msgs::msg::DoorMode;

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
  DoorRequestPub::SharedPtr _door_request_pub;

  using DoorRequestSub = rclcpp::Subscription<DoorRequest>;
  DoorRequestSub::SharedPtr _adapter_door_request_sub;
  void _adapter_door_request_update(DoorRequest::UniquePtr msg);

  void _process_open_request(
    const std::string& door_name,
    const std::string& requester_id,
    const builtin_interfaces::msg::Time& time);

  void _send_open_request(const std::string& door_name);

  void _process_close_request(
    const std::string& door_name,
    const std::string& requester_id,
    const builtin_interfaces::msg::Time& time);

  void _send_close_request(const std::string& door_name);

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateSub = rclcpp::Subscription<DoorState>;
  DoorStateSub::SharedPtr _door_state_sub;
  void _door_state_update(DoorState::UniquePtr msg);

  using Heartbeat = rmf_door_msgs::msg::SupervisorHeartbeat;
  using HeartbeatPub = rclcpp::Publisher<Heartbeat>;
  HeartbeatPub::SharedPtr _door_heartbeat_pub;
  void _publish_heartbeat();

  using OpenRequestLog =
    std::unordered_map<
    std::string,
    std::unordered_map<std::string, rclcpp::Time>>;
  OpenRequestLog _log;
};

} // namespace door_supervisor
} // namesapce rmf_fleet_adapter
