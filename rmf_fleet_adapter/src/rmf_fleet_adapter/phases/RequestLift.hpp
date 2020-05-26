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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__REQUESTLIFT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__REQUESTLIFT_HPP

#include "../Task.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct RequestLift
{
  class ActivePhase : public Task::ActivePhase
  {
  public:

    ActivePhase(
      std::weak_ptr<rmf_rxcpp::Transport> transport,
      std::string lift_name,
      std::string destination,
      rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs);

    const rxcpp::observable<Task::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    std::weak_ptr<rmf_rxcpp::Transport> _transport;
    std::string _lift_name;
    std::string _destination;
    rxcpp::observable<rmf_lift_msgs::msg::LiftState> _lift_state_obs;
    std::string _description;
    rxcpp::observable<Task::StatusMsg> _job;
  };

  class PendingPhase : public Task::PendingPhase
  {
  public:

    PendingPhase(
      std::weak_ptr<rmf_rxcpp::Transport> transport,
      std::string lift_name,
      std::string destination,
      rxcpp::observable<rmf_lift_msgs::msg::LiftState> lift_state_obs);

    std::shared_ptr<Task::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    std::weak_ptr<rmf_rxcpp::Transport> _transport;
    std::string _lift_name;
    std::string _destination;
    rxcpp::observable<rmf_lift_msgs::msg::LiftState> _lift_state_obs;
    std::string _description;
  };

  class Action
  {
  public:

    Action(
      std::weak_ptr<rmf_rxcpp::Transport>& transport,
      std::string& lift_name,
      std::string& destination,
      rxcpp::observable<rmf_lift_msgs::msg::LiftState>& lift_state_obs);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:

    std::weak_ptr<rmf_rxcpp::Transport>& _transport;
    std::string& _lift_name;
    std::string& _destination;
    rxcpp::observable<rmf_lift_msgs::msg::LiftState>& _lift_state_obs;
  };
};

template<typename Subscriber>
void RequestLift::Action::operator()(const Subscriber& s)
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  // TODO: multiplex publisher?
  auto publisher = transport->create_publisher<rmf_lift_msgs::msg::LiftRequest>(
    AdapterLiftRequestTopicName, 10);

  rmf_lift_msgs::msg::LiftRequest msg{};
  msg.lift_name = _lift_name;
  msg.destination_floor = _destination;
  msg.session_id = boost::uuids::to_string(boost::uuids::random_generator{}());
  msg.request_time = transport->now();
  msg.request_type = rmf_lift_msgs::msg::LiftRequest::REQUEST_AGV_MODE;
  msg.door_state = rmf_lift_msgs::msg::LiftRequest::DOOR_OPEN;

  publisher->publish(msg);
  auto timer = transport->create_wall_timer(std::chrono::milliseconds(1000), [publisher, msg]()
  {
    publisher->publish(msg);
  });

  _lift_state_obs.take_while(
    [this, s, timer](const rmf_lift_msgs::msg::LiftState& lift_state)
    {
      if (lift_state.current_floor == _destination)
      {
        s.on_completed();
        return false;
      }
      return true;
    }).subscribe();
}

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__REQUESTLIFT_HPP
