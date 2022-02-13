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
#include "Utils.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<DispenseItem::ActivePhase> DispenseItem::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context),
      std::move(request_guid),
      std::move(target),
      std::move(transporter_type),
      std::move(items)
  ));
  inst->_init_obs();
  return inst;
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
DispenseItem::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration DispenseItem::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void DispenseItem::ActivePhase::emergency_alarm(bool)
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
DispenseItem::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items)
: _context(std::move(context)),
  _request_guid(std::move(request_guid)),
  _target(std::move(target)),
  _transporter_type(std::move(transporter_type)),
  _items(std::move(items))
{
  std::ostringstream oss;
  oss << "Receive items (";
  for (size_t i = 0; i < _items.size(); i++)
  {
    oss << _items[i].type_guid;
    if (i < _items.size()-1)
      oss << ", ";
  }
  oss << ")";

  _description = oss.str();
}

//==============================================================================
void DispenseItem::ActivePhase::_init_obs()
{
  using rmf_dispenser_msgs::msg::DispenserResult;
  using rmf_dispenser_msgs::msg::DispenserState;
  using CombinedType = std::tuple<DispenserResult::SharedPtr,
      DispenserState::SharedPtr>;

  const auto& node = _context->node();
  _obs = node->dispenser_result()
    .start_with(std::shared_ptr<DispenserResult>(nullptr))
    .combine_latest(
    rxcpp::observe_on_event_loop(),
    node->dispenser_state().start_with(
      std::shared_ptr<DispenserState>(nullptr)))
    .lift<CombinedType>(on_subscribe([weak = weak_from_this(), &node]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        me->_do_publish();
        me->_timer =
        node->try_create_wall_timer(std::chrono::milliseconds(1000), [weak]()
        {
          auto me = weak.lock();
          if (!me)
            return;

          me->_do_publish();
        });
      }))
    .map([weak = weak_from_this()](const auto& v)
      {
        auto me = weak.lock();
        if (!me)
          return LegacyTask::StatusMsg();

        return me->_get_status(std::get<0>(v), std::get<1>(v));
      })
    .lift<LegacyTask::StatusMsg>(grab_while_active())
    .finally([weak = weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
          return;

        if (me->_timer)
          me->_timer.reset();
      });
}

//==============================================================================
LegacyTask::StatusMsg DispenseItem::ActivePhase::_get_status(
  const rmf_dispenser_msgs::msg::DispenserResult::SharedPtr& dispenser_result,
  const rmf_dispenser_msgs::msg::DispenserState::SharedPtr& dispenser_state)
{
  LegacyTask::StatusMsg status{};
  status.state = LegacyTask::StatusMsg::STATE_ACTIVE;

  using rmf_dispenser_msgs::msg::DispenserResult;
  if (dispenser_result
    && dispenser_result->request_guid == _request_guid
    && is_newer(dispenser_result->time, _last_msg))
  {
    _last_msg = dispenser_result->time;
    switch (dispenser_result->status)
    {
      case DispenserResult::ACKNOWLEDGED:
        _request_acknowledged = true;
        break;
      case DispenserResult::SUCCESS:
        status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
        break;
      case DispenserResult::FAILED:
        status.state = LegacyTask::StatusMsg::STATE_FAILED;
        break;
    }
  }

  if (dispenser_state
    && dispenser_state->guid == _target
    && is_newer(dispenser_state->time, _last_msg))
  {
    _last_msg = dispenser_state->time;
    if (!_request_acknowledged)
    {
      _request_acknowledged = std::find(
        dispenser_state->request_guid_queue.begin(),
        dispenser_state->request_guid_queue.end(),
        _request_guid) != dispenser_state->request_guid_queue.end();
    }
    else if (
      std::find(
        dispenser_state->request_guid_queue.begin(),
        dispenser_state->request_guid_queue.end(),
        _request_guid
      ) == dispenser_state->request_guid_queue.end())
    {
      // The request has been received, so if it's no longer in the queue,
      // then we'll assume it's finished.
      status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
    }
  }

  return status;
}

//==============================================================================
void DispenseItem::ActivePhase::_do_publish()
{
  rmf_dispenser_msgs::msg::DispenserRequest msg{};
  msg.request_guid = _request_guid;
  msg.target_guid = _target;
  msg.transporter_type = _transporter_type;
  msg.items = _items;
  _context->node()->dispenser_request()->publish(msg);
}

//==============================================================================
DispenseItem::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items)
: _context(std::move(context)),
  _request_guid(std::move(request_guid)),
  _target(std::move(target)),
  _transporter_type(std::move(transporter_type)),
  _items(std::move(items))
{
  std::ostringstream oss;
  oss << "Dispense items (";
  for (size_t i = 0; i < _items.size(); i++)
  {
    oss << _items[i].type_guid;
    if (i < _items.size()-1)
      oss << ", ";
  }
  oss << ")";

  _description = oss.str();
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> DispenseItem::PendingPhase::begin()
{
  return DispenseItem::ActivePhase::make(
    _context,
    _request_guid,
    _target,
    _transporter_type,
    _items);
}

//==============================================================================
rmf_traffic::Duration
DispenseItem::PendingPhase::estimate_phase_duration() const
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
