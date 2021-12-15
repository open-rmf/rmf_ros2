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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DOOROPEN_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DOOROPEN_HPP

#include "DoorClose.hpp"
#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct DoorOpen
{
  /**
   * The phase should do the following
   * 1. Send out a MODE_OPEN door request
   * 3. Periodically resend the open request while the supervisor state does not contain the requester_id
   * 2. It is completed when the supervisor state contains the requester_id and the door has an OPEN mode
   * 4. If cancelled, should start a door close phase
   */
  class ActivePhase : public LegacyTask::ActivePhase,
    public std::enable_shared_from_this<ActivePhase>
  {
  public:

    static std::shared_ptr<ActivePhase> make(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id,
      rmf_traffic::Time expected_finish);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _door_name;
    std::string _request_id;
    rmf_traffic::Time _expected_finish;
    rxcpp::subjects::behavior<bool> _cancelled =
      rxcpp::subjects::behavior<bool>(false);
    rxcpp::observable<LegacyTask::StatusMsg> _obs;
    std::string _description;
    rclcpp::TimerBase::SharedPtr _timer;
    LegacyTask::StatusMsg _status;
    std::shared_ptr<DoorClose::ActivePhase> _door_close_phase;

    ActivePhase(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id,
      rmf_traffic::Time expected_finish);

    void _init_obs();

    void _publish_open_door();

    void _update_status(
      const rmf_door_msgs::msg::DoorState::SharedPtr& door_state,
      const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat);
  };

  class PendingPhase : public LegacyTask::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id,
      rmf_traffic::Time expected_finish);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

    const std::string& door_name() const
    {
      return _door_name;
    }

  private:

    agv::RobotContextPtr _context;
    std::string _door_name;
    std::string _request_id;
    rmf_traffic::Time _expected_finish;
    std::string _description;
  };
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOOROPEN_HPP
