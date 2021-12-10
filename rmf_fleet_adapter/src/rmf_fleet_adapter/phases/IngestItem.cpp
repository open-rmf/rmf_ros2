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

#include "IngestItem.hpp"
#include "Utils.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
std::shared_ptr<IngestItem::ActivePhase> IngestItem::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items)
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
IngestItem::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration IngestItem::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void IngestItem::ActivePhase::emergency_alarm(bool)
{
  // TODO: implement
}

//==============================================================================
void IngestItem::ActivePhase::cancel()
{
  // no op
}

//==============================================================================
const std::string& IngestItem::ActivePhase::description() const
{
  return _description;
}

//==============================================================================
IngestItem::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items)
: _context(std::move(context)),
  _request_guid(std::move(request_guid)),
  _target(std::move(target)),
  _transporter_type(std::move(transporter_type)),
  _items(std::move(items))
{
  std::ostringstream oss;
  oss << "Ingest items (";
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
void IngestItem::ActivePhase::_init_obs()
{
  using rmf_ingestor_msgs::msg::IngestorResult;
  using rmf_ingestor_msgs::msg::IngestorState;
  using CombinedType = std::tuple<IngestorResult::SharedPtr,
      IngestorState::SharedPtr>;

  const auto& node = _context->node();
  _obs = node->ingestor_result()
    .start_with(std::shared_ptr<IngestorResult>(nullptr))
    .combine_latest(
    rxcpp::observe_on_event_loop(),
    node->ingestor_state().start_with(std::shared_ptr<IngestorState>(nullptr)))
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
LegacyTask::StatusMsg IngestItem::ActivePhase::_get_status(
  const rmf_ingestor_msgs::msg::IngestorResult::SharedPtr& ingestor_result,
  const rmf_ingestor_msgs::msg::IngestorState::SharedPtr& ingestor_state)
{
  LegacyTask::StatusMsg status{};
  status.state = LegacyTask::StatusMsg::STATE_ACTIVE;

  using rmf_ingestor_msgs::msg::IngestorResult;
  if (ingestor_result
    && ingestor_result->request_guid == _request_guid
    && is_newer(ingestor_result->time, _last_msg))
  {
    _last_msg = ingestor_result->time;
    switch (ingestor_result->status)
    {
      case IngestorResult::ACKNOWLEDGED:
        _request_acknowledged = true;
        break;
      case IngestorResult::SUCCESS:
        status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
        break;
      case IngestorResult::FAILED:
        status.state = LegacyTask::StatusMsg::STATE_FAILED;
        break;
    }
  }

  if (ingestor_state
    && ingestor_state->guid == _target
    && is_newer(ingestor_state->time, _last_msg))
  {
    _last_msg = ingestor_state->time;
    if (!_request_acknowledged)
    {
      _request_acknowledged = std::find(
        ingestor_state->request_guid_queue.begin(),
        ingestor_state->request_guid_queue.end(),
        _request_guid) != ingestor_state->request_guid_queue.end();
    }
    else if (
      std::find(
        ingestor_state->request_guid_queue.begin(),
        ingestor_state->request_guid_queue.end(),
        _request_guid
      ) == ingestor_state->request_guid_queue.end())
    {
      // The request has been received, so if it's no longer in the queue,
      // then we'll assume it's finished.
      status.state = LegacyTask::StatusMsg::STATE_COMPLETED;
    }
  }

  return status;
}

//==============================================================================
void IngestItem::ActivePhase::_do_publish()
{
  rmf_ingestor_msgs::msg::IngestorRequest msg{};
  msg.request_guid = _request_guid;
  msg.target_guid = _target;
  msg.transporter_type = _transporter_type;
  msg.items = _items;
  _context->node()->ingestor_request()->publish(msg);
}

//==============================================================================
IngestItem::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string request_guid,
  std::string target,
  std::string transporter_type,
  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> items)
: _context(std::move(context)),
  _request_guid(std::move(request_guid)),
  _target(std::move(target)),
  _transporter_type(std::move(transporter_type)),
  _items(std::move(items))
{
  std::ostringstream oss;
  oss << "Ingest items (";
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
std::shared_ptr<LegacyTask::ActivePhase> IngestItem::PendingPhase::begin()
{
  return IngestItem::ActivePhase::make(
    _context,
    _request_guid,
    _target,
    _transporter_type,
    _items);
}

//==============================================================================
rmf_traffic::Duration IngestItem::PendingPhase::estimate_phase_duration() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& IngestItem::PendingPhase::description() const
{
  return _description;
}

} // namespace phases
} // namespace rmf_fleet_adapter
