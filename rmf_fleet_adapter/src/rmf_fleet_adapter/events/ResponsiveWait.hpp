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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__RESPONSIVEWAIT_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__RESPONSIVEWAIT_HPP

#include "../agv/RobotContext.hpp"

#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class ResponsiveWait : public rmf_task_sequence::Event
{
public:

  static void add(rmf_task_sequence::Event::Initializer& initializer);

  static rmf_task::Task::ActivePtr start(
    const std::string& task_id,
    agv::RobotContextPtr& context,
    std::size_t waiting_point,
    std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
    std::function<void()> finished);

  class Description : public rmf_task_sequence::events::Placeholder::Description
  {
  public:

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
    static ConstDescriptionPtr make_indefinite(
      std::size_t waiting_point,
      rmf_traffic::Duration update_period = std::chrono::seconds(30));

    // TODO(MXG): Consider bringing back make_until(...)

    std::size_t waiting_point;
    rmf_traffic::Duration period;

    Description(
      std::size_t waiting_point_,
      rmf_traffic::Duration period_);
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const ResponsiveWait::Description& description,
      std::function<void()> update);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:

    Standby(const Description& description);

    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    Description _description;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state;
    ActivePtr _active = nullptr;

  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Active> make(
      const AssignIDPtr& id,
      agv::RobotContextPtr context,
      Description description,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> update,
      std::function<void()> finished);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:
    Active(Description description);

    void _next_cycle();

    void _begin_movement();

    Description _description;
    std::shared_ptr<rmf_task_sequence::Event::Active> _go_to_place;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    std::function<void()> _update;
    std::function<void()> _finished;
    rmf_task::events::SimpleEventStatePtr _state;
    bool _interrupted = false;
    bool _cancelled = false;
    std::function<void()> _waiting_for_interruption;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__RESPONSIVEWAIT_HPP
