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

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<RequestLift::Action> RequestLift::Action::make(
  std::string requester_id,
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState::SharedPtr> lift_state_obs,
  rclcpp::Publisher<rmf_lift_msgs::msg::LiftRequest>::SharedPtr lift_request_pub)
{
  auto inst = std::shared_ptr<Action>(new Action(
    std::move(requester_id),
    transport,
    std::move(lift_name),
    std::move(destination),
    std::move(lift_state_obs),
    std::move(lift_request_pub)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
RequestLift::Action::Action(
  std::string requester_id,
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState::SharedPtr> lift_state_obs,
  rclcpp::Publisher<rmf_lift_msgs::msg::LiftRequest>::SharedPtr lift_request_pub)
  : _transport{transport},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)},
    _publisher{std::move(lift_request_pub)},
    _session_id{std::move(requester_id)}
{
  // no op
}

//==============================================================================
void RequestLift::Action::_init_obs()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport");

  using rmf_lift_msgs::msg::LiftState;

  _obs = _lift_state_obs
    .lift<LiftState::SharedPtr>(on_subscribe([weak = weak_from_this(), transport]()
    {
      auto me = weak.lock();
      if (!me)
        return;

      me->_do_publish(transport);
      me->_timer = transport->create_wall_timer(
        std::chrono::milliseconds(1000),
        [weak, transport]()
        {
          auto me = weak.lock();
          if (!me)
            return;

          me->_do_publish(transport);
        });
    }))
    .map([weak = weak_from_this()](const auto& v)
    {
      auto me = weak.lock();
      if (!me)
        throw std::runtime_error("invalid state");

      return me->_get_status(v);
    })
    .lift<Task::StatusMsg>(grab_while([weak = weak_from_this()](const Task::StatusMsg& status)
    {
      auto me = weak.lock();
      if (!me)
        return false;

      if (
        status.state == Task::StatusMsg::STATE_COMPLETED ||
        status.state == Task::StatusMsg::STATE_FAILED)
      {
        me->_timer.reset();
        return false;
      }
      return true;
    }));
}

//==============================================================================
Task::StatusMsg RequestLift::Action::_get_status(
  const rmf_lift_msgs::msg::LiftState::SharedPtr& lift_state)
{
  using rmf_lift_msgs::msg::LiftState;
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;
  if (lift_state->current_floor == _destination && lift_state->door_state == LiftState::DOOR_OPEN)
  {
    status.state = Task::StatusMsg::STATE_COMPLETED;
    status.status = "success";
    _timer.reset();
  }
  return status;
}

//==============================================================================
void RequestLift::Action::_do_publish(const rclcpp::Node::SharedPtr& node)
{
  rmf_lift_msgs::msg::LiftRequest msg{};
  msg.lift_name = _lift_name;
  msg.destination_floor = _destination;
  msg.session_id = _session_id;
  msg.request_time = node->now();
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_AGV_MODE;
  msg.door_state = rmf_lift_msgs::msg::LiftRequest::DOOR_OPEN;
  _publisher->publish(msg);
}

//==============================================================================
RequestLift::ActivePhase::ActivePhase(
  std::string requester_id,
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState::SharedPtr> lift_state_obs,
  rclcpp::Publisher<rmf_lift_msgs::msg::LiftRequest>::SharedPtr lift_request_pub)
  : _transport{transport},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)},
    _action{RequestLift::Action::make(
      std::move(requester_id),
      transport,
      _lift_name,
      _destination,
      _lift_state_obs,
      std::move(lift_request_pub))
    }
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& RequestLift::ActivePhase::observe() const
{
  return _action->get_observable();
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
  std::string requester_id,
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string lift_name,
  std::string destination,
  rxcpp::observable<rmf_lift_msgs::msg::LiftState::SharedPtr> lift_state_obs,
  rclcpp::Publisher<rmf_lift_msgs::msg::LiftRequest>::SharedPtr lift_request_pub)
  : _requester_id{std::move(requester_id)},
    _transport{std::move(transport)},
    _lift_name{std::move(lift_name)},
    _destination{std::move(destination)},
    _lift_state_obs{std::move(lift_state_obs)},
    _lift_request_pub{std::move(lift_request_pub)}
{
  std::ostringstream oss;
  oss << "Requesting lift \"" << lift_name << "\" to \"" << destination << "\"";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> RequestLift::PendingPhase::begin()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  return std::make_shared<RequestLift::ActivePhase>(
    _requester_id,
    transport,
    _lift_name,
    _destination,
    _lift_state_obs,
    _lift_request_pub);
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
