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

#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <algorithm>

namespace rmf_fleet_adapter {
namespace phases {

class DoorControlAction
{
public:

  DoorControlAction(
    const std::string& door_name,
    uint32_t target_mode,
    std::shared_ptr<rmf_rxcpp::Transport>& transport,
    rxcpp::observable<rmf_door_msgs::msg::DoorState>& door_state_obs,
    rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat>& supervisor_heartbeat_obs)
    : _door_name{door_name},
      _target_mode{target_mode},
      _transport{transport},
      _door_state_obs{door_state_obs},
      _supervisor_heartbeat_obs{supervisor_heartbeat_obs}
  {
    // no op
  }

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    using rmf_door_msgs::msg::SupervisorHeartbeat;
    using rmf_door_msgs::msg::DoorSessions;
    using rmf_door_msgs::msg::Session;

    // TODO: multiplex publisher?
    auto publisher = _transport->create_publisher<rmf_door_msgs::msg::DoorRequest>(
      AdapterDoorRequestTopicName, 10);

    rmf_door_msgs::msg::DoorRequest msg{};
    msg.door_name = _door_name;
    msg.request_time = _transport->now();
    msg.requested_mode.value = _target_mode;
    msg.requester_id = boost::uuids::to_string(boost::uuids::random_generator{}());

    _supervisor_heartbeat_obs.take_while([this, msg](const SupervisorHeartbeat& heartbeat)
    {
      auto it = std::find_if(
        heartbeat.all_sessions.begin(),
        heartbeat.all_sessions.end(),
        [msg](const DoorSessions& door_sessions)
        {
          return door_sessions.door_name == msg.door_name;
        });

      if (it == heartbeat.all_sessions.end())
        return true;

      const DoorSessions& door_sessions = *it;
      auto it2 = std::find_if(
        door_sessions.sessions.begin(),
        door_sessions.sessions.end(),
        [msg](const Session& session)
        {
          return session.requester_id == msg.requester_id;
        });

      if (it2 == door_sessions.sessions.end())
        return true;

      _supervisor_received_publish = true;
      _timer.reset();
      return false;
    });

    publisher->publish(msg);
    _publish_door_request(publisher, msg);

    _door_state_obs.take_while([this, s](const rmf_door_msgs::msg::DoorState& door_state)
    {
      if (_supervisor_received_publish &&
        door_state.door_name == _door_name &&
        door_state.current_mode.value == _target_mode)
      {
        s.on_completed();
        return false;
      }
      return true;
    }).subscribe();
  }

private:

  const std::string& _door_name;
  uint32_t _target_mode;
  std::shared_ptr<rmf_rxcpp::Transport>& _transport;
  rxcpp::observable<rmf_door_msgs::msg::DoorState>& _door_state_obs;
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat>& _supervisor_heartbeat_obs;
  rclcpp::TimerBase::SharedPtr _timer;
  // FIXME?: This may be read from different threads, do we need a lock or std::atomic<bool>?
  bool _supervisor_received_publish = false;

  void _publish_door_request(
    const rclcpp::Publisher<rmf_door_msgs::msg::DoorRequest>::SharedPtr& publisher,
    const rmf_door_msgs::msg::DoorRequest& msg)
  {
    if (_supervisor_received_publish)
      return;
    publisher->publish(msg);
    _timer = _transport->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, publisher, msg]()
      {
        _publish_door_request(publisher, msg);
      });
  }
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOORCONTROLACTION_HPP
