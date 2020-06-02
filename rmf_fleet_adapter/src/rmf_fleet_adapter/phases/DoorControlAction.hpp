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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DOORCONTROLACTION_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DOORCONTROLACTION_HPP

#include "../Task.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

#include <algorithm>

namespace rmf_fleet_adapter {
namespace phases {

class DoorControlAction
{
public:

  DoorControlAction(
    std::string door_name,
    uint32_t target_mode,
    std::weak_ptr<rmf_rxcpp::Transport> transport,
    rxcpp::observable<rmf_door_msgs::msg::DoorState::SharedPtr> door_state_obs,
    rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs);

  inline const rxcpp::observable<Task::StatusMsg>& get_observable() const
  {
    return _obs;
  }

  inline void cancel()
  {
    _timer.reset();
    _cancelled.get_subscriber().on_next(true);
  }

private:

  std::string _door_name;
  uint32_t _target_mode;
  std::weak_ptr<rmf_rxcpp::Transport> _transport;
  rxcpp::observable<rmf_door_msgs::msg::DoorState::SharedPtr> _door_state_obs;
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> _supervisor_heartbeat_obs;
  rxcpp::observable<Task::StatusMsg> _obs;
  Task::StatusMsg _status;
  rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  std::string _session_id;
  bool _supervisor_received_publish = false;
  bool _supervisor_finished_request = false;
  rxcpp::subjects::behavior<bool> _cancelled{false};

  void _update_status(
    const rmf_door_msgs::msg::DoorState::SharedPtr& door_state,
    const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat);

  void _do_publish();
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOORCONTROLACTION_HPP
