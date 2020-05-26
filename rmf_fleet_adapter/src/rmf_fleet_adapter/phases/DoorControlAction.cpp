/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "DoorControlAction.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorControlAction::DoorControlAction(
  std::string door_name,
  uint32_t target_mode,
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _target_mode{target_mode},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)}
{
  auto _start_obs = rxcpp::observable<>::create<bool>([this](const auto& s)
  {
    _do_publish();
    s.on_completed();
  });
  _obs = _start_obs.combine_latest(
    [this](const auto&, const auto& door_state, const auto& heartbeat)
    {
      return _do(door_state, heartbeat);
    }, door_state_obs, supervisor_heartbeat_obs)

    // using lift instead of take_while/take_until because we want the last COMPLETED state to be
    // reported. take_while/take_until may consume the last message.
    .lift<Task::StatusMsg>([this](const auto& s)
    {
      return rxcpp::make_subscriber<Task::StatusMsg>([this, s](const Task::StatusMsg& status)
      {
        s.on_next(status);
        if (_cancelled || status.state == Task::StatusMsg::STATE_COMPLETED)
          s.on_completed();
      });
  });
}

//==============================================================================
Task::StatusMsg DoorControlAction::_do(
  const rmf_door_msgs::msg::DoorState& door_state,
  const rmf_door_msgs::msg::SupervisorHeartbeat& heartbeat)
{
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;

  bool has_session = supervisor_has_session(heartbeat, _session_id, _door_name);
  if (!_supervisor_received_publish)
  {
    _supervisor_received_publish = has_session;
    if (_supervisor_received_publish)
      _timer.reset();
  }
  else if (!_supervisor_finished_request)
    _supervisor_finished_request = !has_session;

  if (_supervisor_finished_request && door_state.current_mode.value == _target_mode)
    status.state = Task::StatusMsg::STATE_COMPLETED;

  return status;
}

//==============================================================================
void DoorControlAction::_do_publish()
{
  auto publisher = _transport->create_publisher<rmf_door_msgs::msg::DoorRequest>(
    AdapterDoorRequestTopicName, 10);

  _session_id = boost::uuids::to_string(boost::uuids::random_generator{}());
  rmf_door_msgs::msg::DoorRequest msg{};
  msg.door_name = _door_name;
  msg.request_time = _transport->now();
  msg.requested_mode.value = _target_mode;
  msg.requester_id = _session_id;

  publisher->publish(msg);
  _retry_publish_door_request(publisher, msg);
}

//==============================================================================
void DoorControlAction::_retry_publish_door_request(
  const rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr& publisher,
  const rmf_door_msgs::msg::DoorRequest& msg)
{
  if (_supervisor_received_publish)
    return;
  publisher->publish(msg);
  _timer = _transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this, publisher, msg]()
    {
      _retry_publish_door_request(publisher, msg);
    });
}

} // namespace phases
} // namespace rmf_fleet_adapter