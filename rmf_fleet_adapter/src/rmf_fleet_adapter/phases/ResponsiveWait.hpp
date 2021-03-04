/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__RESPONSIVEWAIT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__RESPONSIVEWAIT_HPP

#include "GoToPlace.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
class ResponsiveWait
{
private:
  struct PhaseInfo
  {
    agv::RobotContextPtr context;
    std::size_t waiting_point;
    std::optional<rmf_traffic::Time> finish_time;
    std::optional<rmf_traffic::Duration> period;
    std::string description;
  };

public:

  using StatusMsg = Task::StatusMsg;
  class Pending;

  class Active
      : public Task::ActivePhase,
        public std::enable_shared_from_this<Active>
  {
  public:

    // Documentation inherited
    const rxcpp::observable<StatusMsg>& observe() const final;

    // Documentation inherited
    rmf_traffic::Duration estimate_remaining_time() const final;

    // Documentation inherited
    void emergency_alarm(bool on) final;

    // Documentation inherited
    void cancel() final;

    // Documentation inherited
    const std::string& description() const final;

  private:
    friend class Pending;
    Active(PhaseInfo info);

    void _begin_movement();

    PhaseInfo _info;
    rxcpp::observable<StatusMsg> _status_obs;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    rmf_rxcpp::subscription_guard _movement_subscription;
    std::shared_ptr<Task::ActivePhase> _movement;
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
    friend class ResponsiveWait;
    Pending(PhaseInfo info);
    PhaseInfo _info;
  };

  /// Make a ResponsiveWait phase that has an explicit deadline. When the ROS time
  /// reaches or passes finish_time (and the robot is on its goal waypoint),
  /// the phase will successfully finish.
  ///
  /// \param[in] context
  ///   The context of the robot that needs to wait
  ///
  /// \param[in] waiting_point
  ///   The graph waypoint index that the robot should wait on
  ///
  /// \param[in] finish_time
  ///   The time that the robot can stop waiting
  static std::unique_ptr<Pending> make_until(
    agv::RobotContextPtr context,
    std::size_t waiting_point,
    rmf_traffic::Time finish_time);

  /// Make an ActiveWait phase that has no deadline. The phase will continue
  /// until someone explicitly calls cancel() on it.
  ///
  /// To avoid slamming the schedule with an unreasonably long waiting
  /// prediction, the phase will only schedule a wait for the duration of
  /// the update_period parameter. When that period has passed, the phase will
  /// restart its scheduling for another duration equal to update_period.
  ///
  /// The value of update_period may have a noticeable impact on how other
  /// traffic participants schedule around or through this robot. A small
  /// update_period might make other participants more likely to try to schedule
  /// their paths through this robot. A longer update_period might make that
  /// less likely.
  ///
  /// \param[in] context
  ///   The context of the robot that needs to wait
  ///
  /// \param[in] waiting_point
  ///   The graph waypoint index that the robot should wait on
  ///
  /// \param[in] update_period
  ///   The scheduling period for the waiting
  static std::unique_ptr<Pending> make_indefinite(
    agv::RobotContextPtr context,
    std::size_t waiting_point,
    rmf_traffic::Duration update_period = std::chrono::seconds(30));
};

} // namespace phases
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__PHASES__RESPONSIVEWAIT_HPP
