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

#include "ReadyToCharge.hpp"
#include "RxOperators.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto ReadyToCharge::Active::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration ReadyToCharge::Active::estimate_remaining_time() const
{
  // TODO 
  return rmf_traffic::time::from_seconds(5);
}

//==============================================================================
void ReadyToCharge::Active::emergency_alarm(const bool value)
{
  // Assume charging station is a holding point
}

//==============================================================================
void ReadyToCharge::Active::cancel()
{
  // TODO
}

//==============================================================================
const std::string& ReadyToCharge::Active::description() const
{
  return _description;
}

//==============================================================================
ReadyToCharge::Active::Active(
  agv::RobotContextPtr context,
  const std::string request_id,
  const std::string location)
: _context(std::move(context)), _id(request_id)
{
  _description = "Charger Negotiation";

  //Place holder value for testing
  _location = location;

  //INIT
  _current_state = State::AWAITING_RESPONSE;
}

//==============================================================================
void ReadyToCharge::Active::_init_obs()
{
  using rmf_charger_msgs::msg::ChargerState;
  using rmf_charger_msgs::msg::ChargerRequest;
  _status_obs = _context->node()->charger_state()
    .lift<ChargerState::SharedPtr>(
    on_subscribe(
      [weak = weak_from_this()]()
      {
        auto me = weak.lock();
        if (!me)
        {
          return;
        }
        me->_timer = me->_context->node()->try_create_wall_timer(
          std::chrono::milliseconds(100),
          [me]()
          {
            std::cout << "TXing" <<std::endl;
            ChargerRequest request;
            request.charger_name = me->_location;
            request.fleet_name = me->_context->description().owner();
            request.robot_name = me->_context->description().name();
            request.start_timeout = rclcpp::Duration {10,0};
            request.request_id = me->_id;
            me->_context->node()->charger_request()->publish(request);
          }
        );
      }
    )
    )
    .filter([weak = weak_from_this()](const auto& status_msg)
    {
      auto me = weak.lock();
      if(!me)
        return false;
      return status_msg->charger_name == me->_location;
    })
    .map([weak = weak_from_this()](const auto& status_msg)
    {
      std::cout << "Attempting acknowledgement " <<std::endl;
      auto me = weak.lock();
      if (!me)
      {
        std::cout << "Failed to get lock" <<std::endl;
        return Task::StatusMsg();
      }
      std::cout << "status received" <<std::endl;
      if (me->_current_state == State::AWAITING_RESPONSE)
      {
        if((status_msg->state == ChargerState::CHARGER_ASSIGNED
          || status_msg->state == ChargerState::CHARGER_CHARGING)
          && status_msg->robot_name == me->_context->description().name()
          && status_msg->robot_fleet == me->_context->description().owner()
          && status_msg->request_id == me->_id)
        {
          me->_timer->cancel();
          Task::StatusMsg status;
          status.state = Task::StatusMsg::STATE_COMPLETED;
          status.status = "Negotiation Success";
          return status;
        }
        else if(status_msg->state == ChargerState::CHARGER_ERROR
        && status_msg->request_id == me->_id)
        {
          me->_timer->cancel();
          Task::StatusMsg status;
          status.state = Task::StatusMsg::STATE_FAILED;
          status.status = status_msg->error_message;
          return status;
        }
      }
      Task::StatusMsg status;
      status.state = Task::StatusMsg::STATE_ACTIVE;
      status.status = "Negotiation In progress";
      return status;
    });
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> ReadyToCharge::Pending::begin()
{
  auto active =
      std::shared_ptr<Active>(new Active(_context, _id, _location));
  active->_init_obs();
  return active;
}

//==============================================================================
rmf_traffic::Duration ReadyToCharge::Pending::estimate_phase_duration() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& ReadyToCharge::Pending::description() const
{
  return _description;
}

//==============================================================================
ReadyToCharge::Pending::Pending(
  agv::RobotContextPtr context,
  const std::string id,
  const std::string location)
: _context(std::move(context)), _id(id), _location(location)
{
  _description =
    "Performing Charger Negotiation";
}

//==============================================================================
auto ReadyToCharge::make(
    agv::RobotContextPtr context,
    const std::string id,
    const std::string location) -> std::unique_ptr<Pending>
{
  return std::unique_ptr<Pending>(
        new Pending(context, id, location));
}

}
}
