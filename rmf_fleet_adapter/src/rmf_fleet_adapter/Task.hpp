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
#include <rmf_rxcpp/Publisher.hpp>

namespace rmf_fleet_adapter {

//==============================================================================
class Task
{
public:

  using StatusMsg = rmf_task_msgs::msg::TaskSummary;

  /// This class represents the active phase of a Task. It provides an
  /// observable that the Task can track to stay up-to-date on the status and to
  /// know when to begin the next phase.
  ///
  /// The ActivePhase class must be a schedule Negotiator so that it can
  /// negotiate its way out of conflicts with other schedule participants to
  /// complete its work.
  class ActivePhase : public std::enable_shared_from_this<ActivePhase>
  {
  public:

    /// Get a reference to an observable for the status of this ActivePhase.
    /// When this phase is complete, it will trigger on_completed()
    virtual const rxcpp::observable<StatusMsg>& observe() const = 0;

    /// Estimate how much time remains in this phase.
    virtual rmf_traffic::Duration estimate_remaining_time() const = 0;

    /// Activate or deactivate the emergency alarm behavior.
    virtual void emergency_alarm(bool on) = 0;

    /// Tell this phase to cancel
    virtual void cancel() = 0;

    /// Human-readable description of the phase
    virtual const std::string& description() const = 0;

    // Virtual destructor
    virtual ~ActivePhase() = default;
  };

  class PendingPhase
  {
  public:

    /// Begin this phase.
    virtual std::shared_ptr<ActivePhase> begin() = 0;

    /// Estimate how much time this phase will require.
    virtual rmf_traffic::Duration estimate_phase_duration() const = 0;

    /// Human-readable description of the phase
    virtual const std::string& description() const = 0;

    // Virtual destructor
    virtual ~PendingPhase() = default;
  };

  using StatusCallback =
    std::function<void(const StatusMsg&)>;

  /// Construct a Task
  Task(std::vector<std::unique_ptr<PendingPhase>> phases);

  /// Get a reference to an observable for the status of this Task
  const rxcpp::observable<StatusMsg>& observe() const;

  /// Get the current phase of the task
  const std::shared_ptr<ActivePhase>& current_phase();

  /// const-qualified current_phase()
  std::shared_ptr<const ActivePhase> current_phase() const;

  using PendingPhases = std::vector<std::unique_ptr<PendingPhase>>;

  /// Get the phases of the task that are pending
  const PendingPhases& pending_phases() const;

  /// Cancel this task
  void cancel();

private:

  rmf_rxcpp::Publisher<StatusMsg> _status_publisher;

  std::shared_ptr<ActivePhase> _active_phase;
  rxcpp::subscription _active_phase_subscription;

  // NOTE(MXG): Pending phases are stored in reverse order so we can simply
  // pop_back() to snatch the next phase.
  std::vector<std::unique_ptr<PendingPhase>> _pending_phases;

  rmf_utils::optional<builtin_interfaces::msg::Time> _initial_time;

  void _start_next_phase();

  StatusMsg _process_summary(const StatusMsg& input_msg);
};

}

#endif // SRC__RMF_FLEET_ADAPTER__TASK_HPP
