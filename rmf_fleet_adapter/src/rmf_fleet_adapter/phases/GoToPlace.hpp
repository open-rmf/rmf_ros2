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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__GOTOPLACE_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__GOTOPLACE_HPP

#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

#include "../services/FindPath.hpp"
#include "../services/FindEmergencyPullover.hpp"
#include "../services/Negotiate.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class GoToPlace
{
public:

  using StatusMsg = Task::StatusMsg;
  class Pending;

  class Active
    : public Task::ActivePhase,
    public rmf_traffic::schedule::Negotiator,
    public std::enable_shared_from_this<Active>
  {
  public:

    // Documentation inherited from ActivePhase
    const rxcpp::observable<StatusMsg>& observe() const final;

    // Documentation inherited from ActivePhase
    rmf_traffic::Duration estimate_remaining_time() const final;

    // Documentation inherited from ActivePhase
    void emergency_alarm(bool on) final;

    // Documentation inherited from ActivePhase
    void cancel() final;

    // Documentation inherited from ActivePhase
    const std::string& description() const final;

    // Documentation inherited from Negotiator
    void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

  private:
    friend class Pending;
    Active(
      agv::RobotContextPtr context,
      rmf_traffic::agv::Plan::Goal goal,
      double original_time_estimate,
      std::optional<rmf_traffic::Duration> tail_period);

    void find_plan();

    void find_emergency_plan();

    void execute_plan(rmf_traffic::agv::Plan new_plan);

    agv::RobotContextPtr _context;
    rmf_traffic::agv::Plan::Goal _goal;
    double _latest_time_estimate;
    std::string _description;
    rmf_utils::optional<rmf_traffic::agv::Plan> _plan;
    std::shared_ptr<Task> _subtasks;
    bool _emergency_active = false;
    bool _performing_emergency_task = false;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    rxcpp::observable<StatusMsg> _status_obs;
    rmf_rxcpp::subscription_guard _status_subscription;
    rmf_rxcpp::subscription_guard _plan_subscription;
    std::shared_ptr<services::FindPath> _find_path_service;
    rclcpp::TimerBase::SharedPtr _find_path_timer;
    std::shared_ptr<services::FindEmergencyPullover> _pullover_service;
    rclcpp::TimerBase::SharedPtr _find_pullover_timer;
    std::optional<rmf_traffic::Duration> _tail_period;

    rmf_rxcpp::subscription_guard _interrupt_subscription;

    struct NegotiateManagers
    {
      rmf_rxcpp::subscription_guard subscription;
      rclcpp::TimerBase::SharedPtr timer;
    };
    using NegotiatePtr = std::shared_ptr<services::Negotiate>;
    using NegotiateServiceMap =
      std::unordered_map<NegotiatePtr, NegotiateManagers>;
    NegotiateServiceMap _negotiate_services;

    std::shared_ptr<void> _negotiator_license;
  };

  class Pending : public Task::PendingPhase
  {
  public:

    // Documentation inherited
    std::shared_ptr<Task::ActivePhase> begin() final;

    // Documentation inherited
    rmf_traffic::Duration estimate_phase_duration() const final;

    // Documentation inherited
    const std::string& description() const final;

  private:
    friend class GoToPlace;
    Pending(
      agv::RobotContextPtr context,
      rmf_traffic::agv::Plan::Goal goal,
      double time_estimate,
      std::optional<rmf_traffic::Duration> tail_period);

    agv::RobotContextPtr _context;
    rmf_traffic::agv::Plan::Goal _goal;
    double _time_estimate;
    std::optional<rmf_traffic::Duration> _tail_period;
    std::string _description;
  };

  /// Make a Task Phase for going to a place
  static std::unique_ptr<Pending> make(
    agv::RobotContextPtr context,
    rmf_traffic::agv::Plan::Start start_estimate,
    rmf_traffic::agv::Plan::Goal goal,
    std::optional<rmf_traffic::Duration> tail_period = std::nullopt);

};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__GOTOPLACE_HPP
