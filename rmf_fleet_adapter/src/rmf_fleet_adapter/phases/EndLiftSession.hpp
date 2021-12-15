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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__ENDLIFTSESSION_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__ENDLIFTSESSION_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

namespace rmf_fleet_adapter {
namespace phases {

struct EndLiftSession
{
  class Active : public LegacyTask::ActivePhase,
    public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Active> make(
      agv::RobotContextPtr context,
      std::string lift_name,
      std::string destination);

    Active(
      agv::RobotContextPtr context,
      std::string lift_name,
      std::string destination);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _lift_name;
    std::string _destination;
    std::string _description;
    rxcpp::observable<LegacyTask::StatusMsg> _obs;
    rclcpp::TimerBase::SharedPtr _timer;

    void _init_obs();
    void _publish_session_end();
  };

  class Pending : public LegacyTask::PendingPhase
  {
  public:

    Pending(
      agv::RobotContextPtr context,
      std::string lift_name,
      std::string destination);

    std::shared_ptr<LegacyTask::ActivePhase> begin() final;

    rmf_traffic::Duration estimate_phase_duration() const final;

    const std::string& description() const override;

    const std::string& lift_name() const
    {
      return _lift_name;
    }

    const std::string& destination() const
    {
      return _destination;
    }

  private:
    agv::RobotContextPtr _context;
    std::string _lift_name;
    std::string _destination;
    std::string _description;
  };
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__ENDLIFTSESSION_HPP
