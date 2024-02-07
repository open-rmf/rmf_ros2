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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__RESERVATION_MANAGER_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__RESERVATION_MANAGER_HPP
#include <deque>
#include <optional>
#include <rmf_chope_msgs/msg/reservation_allocation.hpp>

namespace rmf_fleet_adapter {
namespace agv {

enum class ReservationState
{
  AWAITING_REQUEST,
  AWAITING_ALLOCATION,
  AWAITING_TURN
};


class ReservationManager
{
public:
  void add_ticket(const rmf_chope_msgs::msg::ReservationAllocation alloc)
  {
    allocations.push_front(alloc);
  }

  std::optional<rmf_chope_msgs::msg::ReservationAllocation> release_ticket()
  {
    if (allocations.size() <= 1)
    {
      // For safety every robot must have at least one reservation at any point in time.
      return std::nullopt;
    }
    auto temp = allocations.back();
    allocations.pop_back();
    return temp;
  }

  bool has_ticket() const
  {
    return allocations.size() != 0;
  }

  std::deque<rmf_chope_msgs::msg::ReservationAllocation> allocations;
};
}
}
#endif