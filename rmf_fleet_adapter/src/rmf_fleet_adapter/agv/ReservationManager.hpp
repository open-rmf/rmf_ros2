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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__RESERVATION_MANAGER_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__RESERVATION_MANAGER_HPP
#include <deque>
#include <optional>
#include <rmf_reservation_msgs/msg/reservation_allocation.hpp>

namespace rmf_fleet_adapter {
namespace agv {

class RobotContext;

class ReservationManager
{
public:
  /// Adds a ticket and releases the previous ticket if the ticket id is different
  void replace_ticket(
    const rmf_reservation_msgs::msg::ReservationAllocation new_allocation);

  /// Retrieves the location name of the current reservation. Returns empty string if
  /// no location is found.
  std::string get_reserved_location() const;

  /// Checks if a ticket exists
  bool has_ticket() const;
private:
  std::optional<rmf_reservation_msgs::msg::ReservationAllocation> _allocation;
  std::weak_ptr<agv::RobotContext> _context;

  friend RobotContext;
};
}
}
#endif
