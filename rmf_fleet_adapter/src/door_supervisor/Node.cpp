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

namespace rmf_fleet_adapter {
namespace door_supervisor {

const std::string DoorSupervisorRequesterID = "door_supervisor";

//==============================================================================
Node::Node()
: rclcpp::Node("door_supervisor")
{
  const auto default_qos = rclcpp::SystemDefaultsQoS();

  _door_request_pub = create_publisher<DoorRequest>(
    FinalDoorRequestTopicName, default_qos);

  _adapter_door_request_sub = create_subscription<DoorRequest>(
    AdapterDoorRequestTopicName, default_qos,
    [&](DoorRequest::UniquePtr msg)
    {
      _adapter_door_request_update(std::move(msg));
    });

  _door_state_sub = create_subscription<DoorState>(
    DoorStateTopicName, default_qos,
    [&](DoorState::UniquePtr msg)
    {
      _door_state_update(std::move(msg));
    });

  _door_heartbeat_pub = create_publisher<Heartbeat>(
    DoorSupervisorHeartbeatTopicName, default_qos);
}

//==============================================================================
void Node::_adapter_door_request_update(DoorRequest::UniquePtr msg)
{
  if (DoorMode::MODE_OPEN == msg->requested_mode.value)
    _process_open_request(msg->door_name, msg->requester_id, msg->request_time);

  if (DoorMode::MODE_CLOSED == msg->requested_mode.value)
    _process_close_request(msg->door_name, msg->requester_id,
      msg->request_time);
}

//==============================================================================
void Node::_process_open_request(
  const std::string& door_name,
  const std::string& requester_id,
  const builtin_interfaces::msg::Time& time)
{
  auto& open_requests = _log[door_name];
  auto insertion = open_requests.insert(std::make_pair(requester_id, time));
  if (!insertion.second)
  {
    // Use the latest time in the log
    auto& logged_request_time = insertion.first->second;
    logged_request_time = std::max(logged_request_time, rclcpp::Time(time));
  }

  _send_open_request(door_name);
  _publish_heartbeat();
}

//==============================================================================
void Node::_send_open_request(const std::string& door_name)
{
  DoorRequest request;
  request.door_name = door_name;
  request.request_time = get_clock()->now();
  request.requester_id = DoorSupervisorRequesterID;
  request.requested_mode.value = DoorMode::MODE_OPEN;
  _door_request_pub->publish(request);
}

//==============================================================================
void Node::_process_close_request(
  const std::string& door_name,
  const std::string& requester_id,
  const builtin_interfaces::msg::Time& time)
{
  auto door_it = _log.find(door_name);
  if (door_it == _log.end())
    return _publish_heartbeat();

  auto& door_log = door_it->second;
  auto request_it = door_log.find(requester_id);
  if (request_it == door_log.end())
    return _publish_heartbeat();

  auto& logged_request_time = request_it->second;
  const auto new_request_time = rclcpp::Time(time);
  if (new_request_time < logged_request_time)
    return _publish_heartbeat();

  // We can remove this requester from the log of open requests
  door_log.erase(request_it);

  if (!door_log.empty())
    return _publish_heartbeat();

  // If all the open requests have been erased for this door, then we can
  // safely close it.
  // TODO(MXG): Consider whether the door_it should be erased from _log
  _send_close_request(door_name);
  _publish_heartbeat();
}

//==============================================================================
void Node::_send_close_request(const std::string& door_name)
{
  DoorRequest request;
  request.door_name = door_name;
  request.request_time = get_clock()->now();
  request.requester_id = DoorSupervisorRequesterID;
  request.requested_mode.value = DoorMode::MODE_CLOSED;
  _door_request_pub->publish(request);
}

//==============================================================================
void Node::_door_state_update(DoorState::UniquePtr msg)
{
  const std::string& door_name = msg->door_name;
  const auto& door_it = _log.find(door_name);

  // TODO(MXG): Instead of MOVE_MOVING, we may want to consider having a
  // MODE_OPENING and MODE_CLOSING to be more precise about what the door is
  // doing.
  if (door_it == _log.end() || door_it->second.empty())
  {
    if (DoorMode::MODE_OPEN == msg->current_mode.value
      || DoorMode::MODE_MOVING == msg->current_mode.value)
    {
      // If the door is not closed but it's supposed to be, then send a reminder
      _send_close_request(door_name);
    }
  }
  else
  {
    if (DoorMode::MODE_CLOSED == msg->current_mode.value
      || DoorMode::MODE_MOVING == msg->current_mode.value)
    {
      // If the door is not open but it's supposed to be, then send a reminder
      _send_open_request(door_name);
    }
  }
}

//==============================================================================
void Node::_publish_heartbeat()
{
  rmf_door_msgs::msg::SupervisorHeartbeat msg;
  for (const auto& door : _log)
  {
    rmf_door_msgs::msg::DoorSessions sessions;
    sessions.door_name = door.first;
    for (const auto& session : door.second)
    {
      rmf_door_msgs::msg::Session s;
      s.request_time = session.second;
      s.requester_id = session.first;
      sessions.sessions.emplace_back(std::move(s));
    }

    msg.all_sessions.emplace_back(std::move(sessions));
  }

  _door_heartbeat_pub->publish(msg);
}

} // namespace door_supervisor
} // namespace rmf_fleet_adapter
