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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__ERROR_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__ERROR_HPP

#include <rmf_task_sequence/Event.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class Error : public rmf_task_sequence::Event
{
public:

  enum class Behavior
  {
    /// This Error event will block until an operator explicitly cancels,
    /// kills, skips, or rewinds the task. This is the recommended behavior
    /// because an event error usually means something in the task process went
    /// wrong and needs to be retried.
    Block = 0,

    /// This Error event will report that the error occurred but then finish as
    /// soon as it is activated.
    Continue
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      rmf_task::events::SimpleEventStatePtr state,
      Behavior behavior = Behavior::Block);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    rmf_task::events::SimpleEventStatePtr _state;
    Behavior _behavior;
    ActivePtr _active;
  };

  class Active : public rmf_task_sequence::Event::Active
  {
  public:

    static std::shared_ptr<Active> make(
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> finished,
      Behavior behavior = Behavior::Block);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _finished;
  };

};

} // namespace task
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__ERROR_HPP
