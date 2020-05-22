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

#ifndef SRC_RMF_FLEET_ADAPTER_PHASES_DOORCONTROLACTION__HPP
#define SRC_RMF_FLEET_ADAPTER_PHASES_DOORCONTROLACTION__HPP

#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct DoorControlAction
{
  const std::string& door_name;
  uint32_t target_mode;
  rmf_rxcpp::Transport& transport;
  rxcpp::observable<rmf_door_msgs::msg::DoorState>& door_state_obs;

  DoorControlAction(
    const std::string& door_name,
    uint32_t target_mode,
    rmf_rxcpp::Transport& transport,
    rxcpp::observable<rmf_door_msgs::msg::DoorState>& door_state_obs)
    : door_name{door_name},
      target_mode{target_mode},
      transport{transport},
      door_state_obs{door_state_obs}
  {
    // no op
  }

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    // TODO: multiplex publisher?
      auto publisher = transport.create_publisher<rmf_door_msgs::msg::DoorRequest>(
        AdapterDoorRequestTopicName, 10);

      rmf_door_msgs::msg::DoorRequest msg{};
      msg.door_name = door_name;
      msg.request_time = transport.now();
      msg.requested_mode.value = target_mode;
      msg.requester_id = transport.get_name();

      publisher->publish(msg);

      door_state_obs.subscribe([this, s](const rmf_door_msgs::msg::DoorState& door_state)
      {
        if (door_state.door_name == door_name &&
          door_state.current_mode.value == target_mode)
        {
          s.on_completed();
        }
      });
  }
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif //SRC_RMF_FLEET_ADAPTER_PHASES_DOORCONTROLACTION__HPP
