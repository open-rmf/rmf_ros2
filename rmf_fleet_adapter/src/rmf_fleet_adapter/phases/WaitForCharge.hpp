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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__WAITFORCHARGE_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__WAITFORCHARGE_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_battery/agv/BatterySystem.hpp>

#include <rmf_traffic/Time.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class WaitForCharge
{
public:

  using StatusMsg = LegacyTask::StatusMsg;
  class Pending;

  class Active
    : public LegacyTask::ActivePhase,
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

  private:
    friend class Pending;
    Active(
      agv::RobotContextPtr context,
      rmf_battery::agv::BatterySystem battery_system,
      double charge_to_soc,
      rmf_traffic::Time start_time);

    agv::RobotContextPtr _context;
    rmf_battery::agv::BatterySystem _battery_system;
    double _charge_to_soc;
    std::string _description;
    rxcpp::observable<StatusMsg> _status_obs;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    rmf_rxcpp::subscription_guard _battery_soc_subscription;
    rmf_traffic::Time _start_time;
    rmf_traffic::Time _last_update_time;
    double _initial_battery_soc;
    double _expected_charging_rate; // % per hour

  };

  class Pending : public LegacyTask::PendingPhase
  {
  public:
    // Documentation inherited
    std::shared_ptr<LegacyTask::ActivePhase> begin() final;

    // Documentation inherited
    rmf_traffic::Duration estimate_phase_duration() const final;

    // Documentation inherited
    const std::string& description() const final;

  private:
    friend class WaitForCharge;
    Pending(
      agv::RobotContextPtr context,
      rmf_battery::agv::BatterySystem battery_system,
      double charge_to_soc,
      double time_estimate);

    agv::RobotContextPtr _context;
    rmf_battery::agv::BatterySystem _battery_system;
    double _charge_to_soc;
    std::string _description;
    double _time_estimate;
  };

  static std::unique_ptr<Pending> make(
    agv::RobotContextPtr context,
    rmf_battery::agv::BatterySystem battery_system,
    double charge_to_soc);

};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__WAITFORCHARGE_HPP
