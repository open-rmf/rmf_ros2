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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DOORCLOSE_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DOORCLOSE_HPP

#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_rxcpp/Transport.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

namespace rmf_fleet_adapter {
namespace phases {

struct DoorClose
{
  /**
   * The phase should do the following
   * 1. Send out a MODE_CLOSED door request
   * 2. Periodically resend the close request while the supervisor state contains the requester_id
   * 3. It is completed when the supervisor state does NOT contains the requester_id, regardless of the door state
   * 4. Cancellation requests are ignored
   */
  class ActivePhase : public Task::ActivePhase,
    public std::enable_shared_from_this<ActivePhase>
  {
  public:

    static std::shared_ptr<ActivePhase> make(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id);

    const rxcpp::observable<Task::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _door_name;
    std::string _request_id;
    rxcpp::observable<Task::StatusMsg> _obs;
    std::string _description;
    rclcpp::TimerBase::SharedPtr _timer;
    Task::StatusMsg _status;

    ActivePhase(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id);

    void _init_obs();

    void _publish_close_door();

    void _update_status(
      const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat);
  };

  class PendingPhase : public Task::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::string door_name,
      std::string request_id);

    std::shared_ptr<Task::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _door_name;
    std::string _request_id;
    std::string _description;
  };
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOORCLOSE_HPP
