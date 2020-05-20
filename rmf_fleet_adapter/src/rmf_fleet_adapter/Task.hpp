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

#ifndef SRC__RMF_FLEET_ADAPTER__TASK_HPP
#define SRC__RMF_FLEET_ADAPTER__TASK_HPP

#include <string>
#include <memory>

#include <rmf_traffic/schedule/Negotiator.hpp>

#include <rmf_task_msgs/msg/task_summary.hpp>

#include <rmf_rxcpp/RxJobs.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
class Task : public rmf_traffic::schedule::Negotiator
{
public:

  /// This class represents the active phase of a Task. It provides an
  /// observable that the Task can track to stay up-to-date on the status and to
  /// know when to begin the next phase.
  ///
  /// The ActivePhase class must be a schedule Negotiator so that it can
  /// negotiate its way out of conflicts with other schedule participants to
  /// complete its work.
  class ActivePhase
      : public std::enable_shared_from_this<ActivePhase>,
        public rmf_traffic::schedule::Negotiator
  {
  public:

    /// Get a reference to an observable for the status of this ActivePhase.
    /// When this phase is complete, it will trigger on_completed()
    virtual rxcpp::observable<rmf_task_msgs::msg::TaskSummary>& observe() = 0;

    /// Estimate how much time remains in this phase.
    virtual rmf_traffic::Duration estimate_remaining_time() const = 0;

    // Virtual destructor
    virtual ~ActivePhase() = default;
  };

  class PendingPhase : std::enable_shared_from_this<PendingPhase>
  {
  public:

    /// Begin this phase.
    virtual std::shared_ptr<ActivePhase> begin() = 0;

    /// Estimate how much time this phase will require.
    virtual rmf_traffic::Duration estimate_phase_time() const = 0;

    // Virtual destructor
    virtual ~PendingPhase() = default;
  };

  /// Construct a Task
  Task(std::vector<std::unique_ptr<PendingPhase>> phases);

  /// Get the current phase of the task
  const std::shared_ptr<ActivePhase>& current_phase();

  /// const-qualified current_phase()
  const std::shared_ptr<const ActivePhase>& current_phase() const;

  /// Get the phases of the task that are pending
  const std::vector<std::unique_ptr<PendingPhase>>& pending_phases() const;
};

}

#endif // SRC__RMF_FLEET_ADAPTER__TASK_HPP
