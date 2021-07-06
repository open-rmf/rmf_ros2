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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP

#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
struct DockRobot
{
  class Action;

  class ActivePhase : public Task::ActivePhase
  {
  public:

    ActivePhase(
      agv::RobotContextPtr context,
      std::string dock_name);

    const rxcpp::observable<Task::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:
    friend class Action;

    agv::RobotContextPtr _context;
    std::string _dock_name;
    std::string _description;
    std::shared_ptr<Action> _action;
    rxcpp::observable<Task::StatusMsg> _obs;
  };

  class PendingPhase : public Task::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::string dock_name);

    std::shared_ptr<Task::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _dock_name;
    std::string _description;
  };

  class Action
  {
  public:

    Action(ActivePhase* phase);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:
    ActivePhase* _phase;
  };
};

//==============================================================================
template<typename Subscriber>
void DockRobot::Action::operator()(const Subscriber& s)
{
  Task::StatusMsg status;
  status.state = Task::StatusMsg::STATE_ACTIVE;
  status.status = "Docking [" + _phase->_context->requester_id() +
    "] into dock ["
    + _phase->_dock_name + "]";

  s.on_next(status);
  _phase->_context->command()->dock(
    _phase->_dock_name,
    [s, dock_name = _phase->_dock_name, context = _phase->_context]()
    {
      Task::StatusMsg status;
      status.status = "Finished docking [" + context->requester_id()
      + "] into dock [" + dock_name + "]";
      status.state = Task::StatusMsg::STATE_COMPLETED;
      s.on_next(status);
      s.on_completed();
    });
}

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__DOCKROBOT_HPP
