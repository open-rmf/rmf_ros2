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

#include "DoorClose.hpp"
#include "RxOperators.hpp"
#include "SupervisorHasSession.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorClose::ActivePhase::ActivePhase(
  std::string door_name,
  std::string request_id,
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs,
  rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr door_request_pub)
  : _door_name{std::move(door_name)},
    _request_id{std::move(request_id)},
    _transport{transport},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)},
    _door_req_pub(std::move(door_request_pub))
{
  _description = "Closing door \"" + _door_name + "\"";

  using rmf_door_msgs::msg::DoorRequest;
  using rmf_door_msgs::msg::SupervisorHeartbeat;
  _obs = _supervisor_heartbeat_obs
    .lift<SupervisorHeartbeat::SharedPtr>(on_subscribe([this]()
    {
      _status.state = Task::StatusMsg::STATE_ACTIVE;
      _status.status = "waiting for door supervisor to receive request";
      _publish_close_door();
    }))
    .map([this](const auto& heartbeat)
    {
      _update_status(heartbeat);
      return _status;
    })
    .lift<Task::StatusMsg>(grab_while_active())
    .finally([this]()
    {
      if (_timer)
        _timer.reset();
    });
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& DoorClose::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DoorClose::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorClose::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorClose::ActivePhase::cancel()
{
  // Don't actually cancel anything here, we don't want to leave hanging opened doors.
  // no op
}

//==============================================================================
const std::string& DoorClose::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
void DoorClose::ActivePhase::_publish_close_door()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  rmf_door_msgs::msg::DoorRequest msg{};
  msg.door_name = _door_name;
  msg.request_time = transport->now();
  msg.requested_mode.value = rmf_door_msgs::msg::DoorMode::MODE_CLOSED;
  msg.requester_id = _request_id;

  _door_req_pub->publish(msg);
  _timer = transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]()
    {
      _publish_close_door();
    });
}

//==============================================================================
void DoorClose::ActivePhase::_update_status(
  const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat)
{
  if (!supervisor_has_session(*heartbeat, _request_id, _door_name))
  {
    _status.status = "success";
    _status.state = Task::StatusMsg::STATE_COMPLETED;
  }
}

//==============================================================================
DoorClose::PendingPhase::PendingPhase(
  std::string door_name,
  std::string request_id,
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs,
  rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr door_request_pub)
  : _door_name{std::move(door_name)},
    _request_id{std::move(request_id)},
    _transport{std::move(transport)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)},
    _door_request_pub{std::move(door_request_pub)}
{
  _description = "Close door \"" + _door_name + "\"";
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DoorClose::PendingPhase::begin()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  return std::make_shared<DoorClose::ActivePhase>(
    _door_name,
    _request_id,
    transport,
    _supervisor_heartbeat_obs,
    _door_request_pub);
}

//==============================================================================
rmf_traffic::Duration DoorClose::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorClose::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
