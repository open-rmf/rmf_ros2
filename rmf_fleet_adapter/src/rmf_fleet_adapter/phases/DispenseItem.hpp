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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DISPENSEITEM_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DISPENSEITEM_HPP

#include "../Task.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct DispenseItem
{
  class ActivePhase : public Task::ActivePhase
  {
  public:

    ActivePhase(
      std::shared_ptr<rmf_rxcpp::Transport> transport,
      std::string target,
      std::string transporter_type,
      std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
      rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs);

    const rxcpp::observable<Task::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    std::shared_ptr<rmf_rxcpp::Transport> _transport;
    std::string _target;
    std::string _transporter_type;
    std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> _items;
    rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> _result_obs;
    std::string _description;
    rxcpp::observable<Task::StatusMsg> _job;
  };

  class PendingPhase : public Task::PendingPhase
  {
  public:

    PendingPhase(
      std::shared_ptr<rmf_rxcpp::Transport> transport,
      std::string target,
      std::string transporter_type,
      std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> items,
      rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> result_obs);

    std::shared_ptr<Task::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    std::shared_ptr<rmf_rxcpp::Transport> _transport;
    std::string _target;
    std::string _transporter_type;
    std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem> _items;
    rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult> _result_obs;
    std::string _description;
  };

  class Action
  {
  public:

    Action(
      std::shared_ptr<rmf_rxcpp::Transport>& transport,
      std::string& target,
      std::string& transporter_type,
      std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem>& items,
      rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult>& result_obs);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:

    std::shared_ptr<rmf_rxcpp::Transport>& _transport;
    std::string& _target;
    std::string& _transporter_type;
    std::vector<rmf_dispenser_msgs::msg::DispenserRequestItem>& _items;
    rxcpp::observable<rmf_dispenser_msgs::msg::DispenserResult>& _result_obs;
  };
};

template<typename Subscriber>
void DispenseItem::Action::operator()(const Subscriber& s)
{
  // TODO: multiplex publisher?
  auto publisher = _transport->create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
    DispenserRequestTopicName, 10);

  rmf_dispenser_msgs::msg::DispenserRequest msg{};
  msg.request_guid = "";
  msg.target_guid = _target;
  msg.transporter_type = _transporter_type;
  msg.items = _items;

  publisher->publish(msg);
  auto timer = _transport->create_wall_timer(std::chrono::milliseconds(1000), [publisher, msg]()
  {
    publisher->publish(msg);
  });

  _result_obs.take_while(
    [this, s, msg, timer](const rmf_dispenser_msgs::msg::DispenserResult& dispenser_result)
    {
      if (dispenser_result.request_guid == msg.request_guid &&
        (dispenser_result.status == rmf_dispenser_msgs::msg::DispenserResult::SUCCESS ||
        dispenser_result.status == rmf_dispenser_msgs::msg::DispenserResult::FAILED))
      {
        Task::StatusMsg status;
        if (dispenser_result.status == rmf_dispenser_msgs::msg::DispenserResult::SUCCESS)
          status.state = Task::StatusMsg::STATE_COMPLETED;
        else
          status.state = Task::StatusMsg::STATE_FAILED;
        s.on_next(status);
        s.on_completed();
        return false;
      }
      return true;
    }).subscribe();
}

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DISPENSEITEM_HPP
