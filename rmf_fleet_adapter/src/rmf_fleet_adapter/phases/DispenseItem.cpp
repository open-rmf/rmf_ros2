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

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DispenseItem::Action::Action(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs)
  : _transport{std::move(transport)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(items)},
    _result_obs{std::move(result_obs)}
{
  auto transport_ = _transport.lock();
  if (!transport_)
    throw std::runtime_error("invalid transport state");
  // TODO: multiplex publisher?
  _publisher = transport_->create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
    AdapterLiftRequestTopicName, 10);

  _request_guid = boost::uuids::to_string(boost::uuids::random_generator{}());

  using rmf_dispenser_msgs::msg::DispenserResult;
  _obs = _result_obs
    .lift<DispenserResult>(on_subscribe([this]() { _do_publish(); }))
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
Task::StatusMsg DispenseItem::Action::_check_status(
  const rmf_dispenser_msgs::msg::DispenserResult& dispenser_result)
{
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;

  using rmf_dispenser_msgs::msg::DispenserResult;
  if (dispenser_result.request_guid == _request_guid)
  {
    switch (dispenser_result.status)
    {
      case DispenserResult::SUCCESS:
        status.state = Task::StatusMsg::STATE_COMPLETED;
        break;
      case DispenserResult::FAILED:
        status.state = Task::StatusMsg::STATE_FAILED;
        break;
    }
  }
  return status;
}

//==============================================================================
void DispenseItem::Action::_do_publish()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  rmf_dispenser_msgs::msg::DispenserRequest msg{};
  msg.request_guid = _request_guid;
  msg.target_guid = _target;
  msg.transporter_type = _transporter_type;
  msg.items = _items;

  _publisher->publish(msg);
  _timer = transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]()
    {
      _do_publish();
    });
}

//==============================================================================
DispenseItem::ActivePhase::ActivePhase(
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> dispenser_items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs)
  : _transport{std::move(transport)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(dispenser_items)},
    _result_obs{std::move(result_obs)},
    _action{
      _transport,
      _target,
      _transporter_type,
      _items,
      _result_obs}
{
  std::ostringstream oss;
  oss << "Dispensing items (";
  for (size_t i = 0; i < _items.size() - 1; i++)
    oss << _items[i].type_guid << ", ";
  oss << _items[_items.size() - 2].type_guid << ")";

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
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs)
  : _transport{std::move(transport)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(items)},
    _result_obs{std::move(result_obs)}
{
  std::ostringstream oss;
  oss << "Dispense items (";
  for (size_t i = 0; i < _items.size() - 1; i++)
    oss << _items[i].type_guid << ", ";
  oss << _items[_items.size() - 2].type_guid << ")";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> DispenseItem::PendingPhase::begin()
{
  return std::make_shared<DispenseItem::ActivePhase>(
    _transport, _target, _transporter_type, _items, _result_obs);
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