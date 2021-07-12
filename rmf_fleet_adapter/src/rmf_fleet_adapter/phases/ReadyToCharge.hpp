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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__READYTOCHARGE_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__READYTOCHARGE_HPP


#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class ReadyToCharge
{
public:
  using StatusMsg = Task::StatusMsg;
  class Pending;

  class Active
    : public Task::ActivePhase,
      public std::enable_shared_from_this<Active>
  {
  public:
    Active(agv::RobotContextPtr context,
      std::string request_id,
      std::string location);

    // Documentation inherited from ActivePhase
    const rxcpp::observable<StatusMsg>& observe() const final;

    // Documentation inherited from ActivePhase
    rmf_traffic::Duration estimate_remaining_time() const final;

    // Documentation inherited from ActivePhase
    void emergency_alarm(bool on) final;

    // Documentation inherited from ActivePhase
    void cancel() final;

    // Documentation inherited from ActivePhase
    const std::string & description() const final;

  private:
    enum State
    {
      AWAITING_RESPONSE = 0,
      REQUEST_SUCCESS
    };

    friend class Pending;
    rxcpp::observable<StatusMsg> _status_obs;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    agv::RobotContextPtr _context;
    State _current_state;
    rclcpp::TimerBase::SharedPtr _timer;
    std::string _id;
    std::string _description;
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
    friend class ReadyToCharge;
    Pending(
      agv::RobotContextPtr context,
      std::string id,
      std::string location);

    agv::RobotContextPtr _context;
    std::string _description;
    std::string _id;
    std::string _location;
  };

  static std::unique_ptr<Pending> make(
    agv::RobotContextPtr context,
    const std::string id,
    const std::string location);
  };
}
}
#endif
