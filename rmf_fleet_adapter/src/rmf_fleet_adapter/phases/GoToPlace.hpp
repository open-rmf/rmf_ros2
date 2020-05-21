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

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class GoToPlace
{
public:

  using StatusMsg = Task::StatusMsg;

  class Active : public Task::ActivePhase
  {
  public:

    // Documentation inherited
    rxcpp::observable<StatusMsg>& observe() final;

    // Documentation inherited
    rmf_traffic::Duration estimate_remaining_time() const final;

    // Documentation inherited
    void emergency_alarm(bool on) final;

    // Documentation inherited
    void cancel() final;

    // Documentation inherited
    const std::string & description() const final;

  private:



  };

  class Pending : public Task::PendingPhase
  {
  public:

    // Documentation inherited
    std::shared_ptr<Task::ActivePhase> begin() final;

    // Documentation inherited
    rmf_traffic::Duration estimate_phase_duration() const final;

    // Documentation inherited
    const std::string & description() const final;

  private:


  };

  std::unique_ptr<Pending> make();

};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__GOTOPLACE_HPP
