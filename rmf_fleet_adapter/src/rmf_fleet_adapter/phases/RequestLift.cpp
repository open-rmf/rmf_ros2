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

#include "RequestLift.hpp"
#include "RxOperators.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
RequestLift::Action::Action(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs)
  : _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)}
{
  using rmf_lift_msgs::msg::LiftState;

  auto transport_ = _transport.lock();
  if (!transport_)
    throw std::runtime_error("invalid transport state");
  // TODO: multiplex publisher?
  _publisher = transport_->create_publisher<rmf_lift_msgs::msg::LiftRequest>(
    AdapterLiftRequestTopicName, 10);

  _session_id = boost::uuids::to_string(boost::uuids::random_generator{}());

  _obs = _lift_state_obs
    .lift<LiftState>(on_subscribe([this]() { _do_publish(); }))
    .map([this](const auto& v)
    {
      return _check_status(v);
    })
    .lift<Task::StatusMsg>(grab_while([this](const Task::StatusMsg& status)
    {
      if (
        status.state == Task::StatusMsg::STATE_COMPLETED ||
        status.state == Task::StatusMsg::STATE_FAILED)
      {
        _timer.reset();
        return false;
      }
      return true;
    }));
}

//==============================================================================
Task::StatusMsg RequestLift::Action::_check_status(const rmf_lift_msgs::msg::LiftState& lift_state)
{
  using rmf_lift_msgs::msg::LiftState;
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;
  if (lift_state.current_floor == _destination && lift_state.door_state == LiftState::DOOR_OPEN)
  {
    status.state = Task::StatusMsg::STATE_COMPLETED;
    _timer.reset();
  }
  return status;
}

//==============================================================================
void RequestLift::Action::_do_publish()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  rmf_lift_msgs::msg::LiftRequest msg{};
  msg.lift_name = _lift_name;
  msg.destination_floor = _destination;
  msg.session_id = _session_id;
  msg.request_time = transport->now();
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_AGV_MODE;
  msg.door_state = rmf_lift_msgs::msg::LiftRequest::DOOR_OPEN;

  _publisher->publish(msg);
  _timer = transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]()
    {
      _do_publish();
    });
}

//==============================================================================
RequestLift::ActivePhase::ActivePhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs)
  : _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)},
    _action{_transport, _lift_name, _destination, _lift_state_obs}
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& RequestLift::ActivePhase::observe() const
{
  return _action.get_observable();
}

//==============================================================================
rmf_traffic::Duration RequestLift::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void RequestLift::ActivePhase::emergency_alarm(bool on)
{
  // TODO: implement
}

//==============================================================================
void RequestLift::ActivePhase::cancel()
{
  // TODO: implement
}

//==============================================================================
const std::string& RequestLift::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
RequestLift::PendingPhase::PendingPhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs)
  : _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)}
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> RequestLift::PendingPhase::begin()
{
  return std::make_shared<RequestLift::ActivePhase>(
    _transport, _lift_name, _destination, _lift_state_obs);
}

//==============================================================================
rmf_traffic::Duration RequestLift::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& RequestLift::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter