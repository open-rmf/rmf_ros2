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

#include "WaitForCharge.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto WaitForCharge::Active::observe() const
-> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration WaitForCharge::Active::estimate_remaining_time() const
{
  const double capacity = _battery_system.capacity();
  const double charging_current = _battery_system.charging_current();
  const double time_estimate =
    3600.0 * capacity * (_charge_to_soc - _context->current_battery_soc()) /
    charging_current;

  return rmf_traffic::time::from_seconds(time_estimate);
}

//==============================================================================
void WaitForCharge::Active::emergency_alarm(const bool)
{
  // Assume charging station is a holding point
}

//==============================================================================
void WaitForCharge::Active::cancel()
{
  // TODO
}

//==============================================================================
const std::string& WaitForCharge::Active::description() const
{
  return _description;
}

//==============================================================================
WaitForCharge::Active::Active(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc,
  rmf_traffic::Time start_time)
: _context(std::move(context)),
  _battery_system(battery_system),
  _charge_to_soc(charge_to_soc),
  _start_time(start_time)
{
  _last_update_time = start_time;
  _initial_battery_soc = _context->current_battery_soc();
  _expected_charging_rate = (100.0 / (
      _battery_system.capacity() /
      _battery_system.charging_current()));

  _description = "Charging [" + _context->requester_id() + "] to ["
    + std::to_string(100.0 * _charge_to_soc) + "]";

  StatusMsg initial_msg;
  initial_msg.status = _description;
  _status_publisher.get_subscriber().on_next(initial_msg);
  const auto now = _context->node()->now();
  initial_msg.start_time = now;
  initial_msg.end_time = now + estimate_remaining_time();

  _status_obs = _status_publisher
    .get_observable()
    .start_with(initial_msg);

  _context->current_mode(rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING);
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> WaitForCharge::Pending::begin()
{
  const auto& now = std::chrono::steady_clock::now();

  auto active =
    std::shared_ptr<Active>(new Active(
        _context, _battery_system, _charge_to_soc, now));

  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Robot [%s] has begun waiting for its battery to charge to %.1f%%. "
    "Please ensure that the robot is charging.",
    _context->name().c_str(),
    _charge_to_soc * 100.0);

  active->_battery_soc_subscription = _context->observe_battery_soc()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [a = active->weak_from_this()](const double battery_soc)
    {
      const auto active = a.lock();

      if (!active)
        return;

      if (active->_charge_to_soc <= battery_soc)
      {
        active->_status_publisher.get_subscriber().on_completed();
      }

      const auto& now = std::chrono::steady_clock::now();
      if (std::chrono::seconds(60) <= now - active->_last_update_time)
      {
        const double delta_soc = battery_soc - active->_initial_battery_soc;
        const double elapsed_seconds =
        (now - active->_start_time).count() / 1e9;
        const double average_charging_rate =
        100.0 * delta_soc / (elapsed_seconds / 3600.0);

        RCLCPP_INFO(
          active->_context->node()->get_logger(),
          "Robot [%s] is still waiting for its battery to charge to %.1f%%. "
          "The current battery percentage is %.1f%%. The robot is charging "
          "at an average rate of %.1f %%/hour. The expected charging rate "
          "is %.1f %%/hour. If the battery percentage has not been rising, "
          "please check that the robot is connected to its charger.",
          active->_context->name().c_str(),
          active->_charge_to_soc * 100.0,
          battery_soc * 100,
          average_charging_rate,
          active->_expected_charging_rate);

        active->_last_update_time = now;
      }

    });

  return active;
}

//==============================================================================
rmf_traffic::Duration WaitForCharge::Pending::estimate_phase_duration() const
{
  return rmf_traffic::time::from_seconds(_time_estimate);
}

//==============================================================================
const std::string& WaitForCharge::Pending::description() const
{
  return _description;
}

//==============================================================================
WaitForCharge::Pending::Pending(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc,
  double time_estimate)
: _context(std::move(context)),
  _battery_system(battery_system),
  _charge_to_soc(charge_to_soc),
  _time_estimate(time_estimate)
{
  _description =
    "Charging robot to [" + std::to_string(100.0 * charge_to_soc) + "%]";
}

//==============================================================================
auto WaitForCharge::make(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc) -> std::unique_ptr<Pending>
{

  const double capacity = battery_system.capacity();
  const double charging_current = battery_system.charging_current();
  const double time_estimate =
    3600.0 * capacity * (charge_to_soc - context->current_battery_soc()) /
    charging_current;

  return std::unique_ptr<Pending>(
    new Pending(context, battery_system, charge_to_soc, time_estimate));
}

} // namespace phases
} // namespace rmf_fleet_adapter
