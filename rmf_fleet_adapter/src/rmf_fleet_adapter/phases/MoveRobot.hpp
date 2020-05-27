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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP

#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

namespace rmf_fleet_adapter {
namespace phases {

struct MoveRobot
{
  class ActivePhase : public Task::ActivePhase
  {
  public:

    ActivePhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints);

    const rxcpp::observable<Task::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    std::string _description;
    rxcpp::observable<Task::StatusMsg> _job;
  };

  class PendingPhase : public Task::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints);

    std::shared_ptr<Task::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    std::string _description;
  };

  class Action
  {
  public:

    Action(
      agv::RobotContextPtr& context,
      std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:

    agv::RobotContextPtr& _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint>& _waypoints;
  };
};

template<typename Subscriber>
void MoveRobot::Action::operator()(const Subscriber& s)
{
  _context->command()->follow_new_path(_waypoints, [s]()
  {
    s.on_completed();
  });
}

} // namespace phases
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
