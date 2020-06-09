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

#include "DoorOpen.hpp"
#include "RxOperators.hpp"
#include "SupervisorHasSession.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <utility>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorOpen::ActivePhase::ActivePhase(
  std::string door_name,
  std::string request_id,
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState::SharedPtr> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs,
  rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr door_request_pub)
  : _door_name{std::move(door_name)},
    _request_id{std::move(request_id)},
    _transport{transport},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)},
    _door_req_pub(std::move(door_request_pub)),
    _door_close_phase{
      _door_name,
      _request_id,
      transport,
      _supervisor_heartbeat_obs,
      _door_req_pub
    }
{
  _description = "Opening door \"" + _door_name + "\"";

  using rmf_door_msgs::msg::DoorRequest;
  using rmf_door_msgs::msg::DoorState;
  using rmf_door_msgs::msg::SupervisorHeartbeat;
  using CombinedType = std::tuple<DoorState::SharedPtr, SupervisorHeartbeat::SharedPtr>;
  _obs = _door_state_obs.combine_latest(_supervisor_heartbeat_obs)
    .lift<CombinedType>(on_subscribe([this]()
    {
      _status.state = Task::StatusMsg::STATE_ACTIVE;
      _status.status = "waiting for door supervisor to receive request";
      _publish_open_door();
    }))
    .map([this](const auto& v)
    {
      _update_status(std::get<0>(v), std::get<1>(v));
      return _status;
    })
    .lift<Task::StatusMsg>(grab_while_active())
    .finally([this]()
    {
      if (_timer)
        _timer.reset();
    })
    // When the phase is cancelled, queue a door close phase to make sure that there is no hanging
    // open doors
    .take_until(_cancelled.get_observable().filter([](auto b) { return b; }))
    .concat(rxcpp::observable<>::create<Task::StatusMsg>(
      [this](const auto& s)
      {
        // FIXME: is this thread-safe?
        if (!_cancelled.get_value())
          s.on_completed();
        else
          _door_close_phase.observe().subscribe(s);
      }));
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& DoorOpen::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DoorOpen::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DoorOpen::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void DoorOpen::ActivePhase::cancel()
{
  _cancelled.get_subscriber().on_next(true);
}

//==============================================================================
const std::string& DoorOpen::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
void DoorOpen::ActivePhase::_publish_open_door()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  rmf_door_msgs::msg::DoorRequest msg{};
  msg.door_name = _door_name;
  msg.request_time = transport->now();
  msg.requested_mode.value = rmf_door_msgs::msg::DoorMode::MODE_OPEN;
  msg.requester_id = _request_id;

  _door_req_pub->publish(msg);
  _timer = transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]()
    {
      _publish_open_door();
    });
}

//==============================================================================
void DoorOpen::ActivePhase::_update_status(
  const rmf_door_msgs::msg::DoorState::SharedPtr& door_state,
  const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat)
{
  using rmf_door_msgs::msg::DoorMode;
  if (door_state->door_name == _door_name &&
    door_state->current_mode.value == DoorMode::MODE_OPEN
    && supervisor_has_session(*heartbeat, _request_id, _door_name))
  {
    _status.status = "success";
    _status.state = Task::StatusMsg::STATE_COMPLETED;
  }
}

//==============================================================================
DoorOpen::PendingPhase::PendingPhase(
  std::string  door_name,
  std::string request_id,
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState::SharedPtr> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs,
  rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr door_request_pub)
  : _door_name{std::move(door_name)},
    _request_id{std::move(request_id)},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)},
    _door_request_pub{std::move(door_request_pub)}
{
  _description = "Open door \"" + _door_name + "\"";
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DoorOpen::PendingPhase::begin()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  return std::make_shared<DoorOpen::ActivePhase>(
    _door_name,
    _request_id,
    transport,
    _door_state_obs,
    _supervisor_heartbeat_obs,
    _door_request_pub);
}

//==============================================================================
rmf_traffic::Duration DoorOpen::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DoorOpen::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
