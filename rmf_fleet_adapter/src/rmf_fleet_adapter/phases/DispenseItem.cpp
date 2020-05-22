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
DispenseItem::ActivePhase::ActivePhase(
  std::shared_ptr<rmf_rxcpp::Transport> transport,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> dispenser_items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs)
  : _transport{std::move(transport)},
    _target{std::move(target)},
    _transporter_type{std::move(transporter_type)},
    _items{std::move(dispenser_items)},
    _result_obs{std::move(result_obs)}
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
  return _job;
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
  // TODO: implement
}

//==============================================================================
const std::string& DispenseItem::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
DispenseItem::PendingPhase::PendingPhase(
  std::shared_ptr<rmf_rxcpp::Transport> transport,
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

//==============================================================================
DispenseItem::Action::Action(
  std::shared_ptr<rmf_rxcpp::Transport>& transport,
  std::string& target,
  std::string& transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem>& items,
  rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult>& result_obs)
  : _transport{transport},
    _target{target},
    _transporter_type{transporter_type},
    _items{items},
    _result_obs{result_obs}
{
  // no op
}

} // namespace phases
} // namespace rmf_fleet_adapter