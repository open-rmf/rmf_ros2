/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "ReservationManager.hpp"
#include "RobotContext.hpp"

using namespace rmf_fleet_adapter::agv;

//==============================================================================
void ReservationManager::replace_ticket(
  const rmf_reservation_msgs::msg::ReservationAllocation new_allocation)
{
  auto context = _context.lock();
  if (!context)
  {
    return;
  }
  if (has_ticket())
  {
    if (new_allocation.ticket.ticket_id != _allocation->ticket.ticket_id)
    {
      RCLCPP_INFO(
        context->node()->get_logger(),
        "Releasing waypoint for ticket %lu as new ticket has become available",
        _allocation->ticket.ticket_id);
      rmf_reservation_msgs::msg::ReleaseRequest msg;
      msg.ticket = _allocation->ticket;
      context->node()->release_location()->publish(msg);
    }
  }
  _allocation = new_allocation;
}

//==============================================================================
std::string ReservationManager::get_reserved_location() const
{
  if (has_ticket())
    return _allocation->resource;

  return "";
}

//==============================================================================
bool ReservationManager::has_ticket() const
{
  return _allocation.has_value();
}
