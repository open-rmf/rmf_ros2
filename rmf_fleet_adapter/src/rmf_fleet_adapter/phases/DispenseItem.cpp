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

#include "DispenseItem.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DispenseItem::Action::Action(
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult::SharedPtr> result_obs,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserState::SharedPtr> state_obs,
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr request_pub)
  : _transport{transport},
    _request_guid{std::move(request_guid)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(items)},
    _result_obs{std::move(result_obs)},
    _state_obs{std::move(state_obs)},
    _request_pub{std::move(request_pub)}
{
  using rmf_dispenser_msgs::msg::DispenserResult;
  using rmf_dispenser_msgs::msg::DispenserState;
  using CombinedType = std::tuple<DispenserResult::SharedPtr, DispenserState::SharedPtr>;
  _obs = _result_obs
    .combine_latest(
      rxcpp::observe_on_event_loop(),
      _state_obs.start_with(std::make_shared<DispenserState>()))
    .lift<CombinedType>(on_subscribe([this, transport]()
    {
      _do_publish();
      _timer = transport->create_wall_timer(std::chrono::milliseconds(1000), [this]()
      {
        _do_publish();
      });
    }))
    .map([this](const auto& v)
    {
      return _get_status(std::get<0>(v), std::get<1>(v));
    })
    .lift<Task::StatusMsg>(grab_while_active())
    .finally([this]()
    {
      if (_timer)
        _timer.reset();
    });
}

//==============================================================================
Task::StatusMsg DispenseItem::Action::_get_status(
  const rmf_dispenser_msgs::msg::DispenserResult::SharedPtr& dispenser_result,
  const rmf_dispenser_msgs::msg::DispenserState::SharedPtr& dispenser_state)
{
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;

  using rmf_dispenser_msgs::msg::DispenserResult;
  if (dispenser_result->request_guid == _request_guid)
  {
    switch (dispenser_result->status)
    {
      case DispenserResult::ACKNOWLEDGED:
        _request_acknowledged = true;
        break;
      case DispenserResult::SUCCESS:
        status.state = Task::StatusMsg::STATE_COMPLETED;
        break;
      case DispenserResult::FAILED:
        status.state = Task::StatusMsg::STATE_FAILED;
        break;
    }
  }

  if (!_request_acknowledged)
  {
    _request_acknowledged = std::find(
          dispenser_state->request_guid_queue.begin(),
          dispenser_state->request_guid_queue.end(),
          _request_guid) != dispenser_state->request_guid_queue.end();
  }
  else if (
    dispenser_state->guid == _target &&
    std::find(
      dispenser_state->request_guid_queue.begin(),
      dispenser_state->request_guid_queue.end(),
      _request_guid
    ) == dispenser_state->request_guid_queue.end())
  {
    // The request has been received, so if it's no longer in the queue,
    // then we'll assume it's finished.
    status.state = Task::StatusMsg::STATE_COMPLETED;
  }

  return status;
}

//==============================================================================
void DispenseItem::Action::_do_publish()
{
  rmf_dispenser_msgs::msg::DispenserRequest msg{};
  msg.request_guid = _request_guid;
  msg.target_guid = _target;
  msg.transporter_type = _transporter_type;
  msg.items = _items;
  _request_pub->publish(msg);
}

//==============================================================================
DispenseItem::ActivePhase::ActivePhase(
  const std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> dispenser_items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult::SharedPtr> result_obs,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserState::SharedPtr> state_obs,
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr request_pub)
  : _transport{transport},
    _request_guid{std::move(request_guid)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(dispenser_items)},
    _result_obs{std::move(result_obs)},
    _state_obs{std::move(state_obs)},
    _action{
      transport,
      _request_guid,
      _target,
      _transporter_type,
      _items,
      _result_obs,
      _state_obs,
      std::move(request_pub)}
{
  std::ostringstream oss;
  oss << "Dispensing items (";
  for (size_t i = 0; i < _items.size() - 1; i++)
    oss << _items[i].type_guid << ", ";
  oss << _items[_items.size() - 1].type_guid << ")";

  _description = oss.str();
}

//==============================================================================
const rxcpp::observable<Task::StatusMsg>& DispenseItem::ActivePhase::observe() const
{
  return _action.get_observable();
}

//==============================================================================
rmf_traffic::Duration DispenseItem::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DispenseItem::ActivePhase::emergency_alarm(bool on)
{
  // TODO: implement
}

//==============================================================================
void DispenseItem::ActivePhase::cancel()
{
  // no op
}

//==============================================================================
const std::string& DispenseItem::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
DispenseItem::PendingPhase::PendingPhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult::SharedPtr> result_obs,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserState::SharedPtr> state_obs,
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr request_pub)
  : _transport{std::move(transport)},
    _request_guid{std::move(request_guid)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(items)},
    _result_obs{std::move(result_obs)},
    _state_obs{std::move(state_obs)},
    _request_pub{std::move(request_pub)}
{
  std::ostringstream oss;
  oss << "Dispense items (";
  for (size_t i = 0; i < _items.size() - 1; i++)
    oss << _items[i].type_guid << ", ";
  oss << _items[_items.size() - 1].type_guid << ")";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DispenseItem::PendingPhase::begin()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  return std::make_shared<DispenseItem::ActivePhase>(
    transport,
    _request_guid,
    _target,
    _transporter_type,
    _items,
    _result_obs,
    _state_obs,
    _request_pub);
}

//==============================================================================
rmf_traffic::Duration DispenseItem::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& DispenseItem::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
