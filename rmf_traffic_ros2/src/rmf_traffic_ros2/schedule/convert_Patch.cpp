/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

using Change = rmf_traffic::schedule::Database::Change;
using Time = rmf_traffic::Time;
using Duration = rmf_traffic::Duration;

namespace rmf_traffic_ros2 {

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeInsert convert_insert(
    const Change& change)
{
  const Change::Insert& insert = *change.insert();

  rmf_traffic_msgs::msg::ScheduleChangeInsert msg;
  msg.version = change.id();
  msg.trajectory = convert(*insert.trajectory());

  return msg;
}

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeInterrupt convert_interrupt(
    const Change& change)
{
  const Change::Interrupt& interrupt = *change.interrupt();

  rmf_traffic_msgs::msg::ScheduleChangeInterrupt msg;
  msg.version = change.id();
  msg.interruption_trajectory = convert(*interrupt.interruption());
  msg.delay = interrupt.delay().count();
  msg.parent_id = interrupt.original_id();

  return msg;
}

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeDelay convert_delay(
    const Change& change)
{
  const Change::Delay& delay = *change.delay();

  rmf_traffic_msgs::msg::ScheduleChangeDelay msg;
  msg.version = change.id();
  msg.from_time = delay.from().time_since_epoch().count();
  msg.delay = delay.duration().count();
  msg.parent_id = delay.original_id();

  return msg;
}

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeReplace convert_replace(
    const Change& change)
{
  const Change::Replace& replace = *change.replace();

  rmf_traffic_msgs::msg::ScheduleChangeReplace msg;
  msg.version = change.id();
  msg.trajectory = convert(*replace.trajectory());
  msg.parent_id = replace.original_id();

  return msg;
}

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeErase convert_erase(
    const Change& change)
{
  const Change::Erase& erase = *change.erase();

  rmf_traffic_msgs::msg::ScheduleChangeErase msg;
  msg.version = change.id();
  msg.parent_id = erase.original_id();

  return msg;
}

//==============================================================================
static rmf_traffic_msgs::msg::ScheduleChangeCull convert_cull(
    const Change& change)
{
  const Change::Cull& cull = *change.cull();

  rmf_traffic_msgs::msg::ScheduleChangeCull msg;
  msg.version = change.id();
  msg.time = cull.time().time_since_epoch().count();

  return msg;
}

//==============================================================================
class ChangeDispatcher
{
public:

  using PatchMsg = rmf_traffic_msgs::msg::SchedulePatch;

  ChangeDispatcher()
  {
    dispatcher.resize(static_cast<std::size_t>(Change::Mode::NUM),
                      [](PatchMsg&, const Change& change)
    {
      throw std::runtime_error(
            "Unsupported Database::Change::Mode ["
            + std::to_string(static_cast<uint16_t>(change.get_mode())) + "]");
    });

#define MAKE_DISPATCH(x) dispatcher[static_cast<uint16_t>(Change::Mode:: x)] = \
  [](PatchMsg& patch, const Change& change)

    MAKE_DISPATCH(Insert) { patch.insertions.emplace_back(convert_insert(change)); };
    MAKE_DISPATCH(Interrupt) { patch.interruptions.emplace_back(convert_interrupt(change)); };
    MAKE_DISPATCH(Delay) { patch.delays.emplace_back(convert_delay(change)); };
    MAKE_DISPATCH(Replace) { patch.replacements.emplace_back(convert_replace(change)); };
    MAKE_DISPATCH(Erase) { patch.erasures.emplace_back(convert_erase(change)); };
    MAKE_DISPATCH(Cull) { patch.culls.emplace_back(convert_cull(change)); };
  }

  void dispatch(PatchMsg& patch, const Change& change)
  {
    dispatcher.at(static_cast<std::size_t>(change.get_mode()))(patch, change);
  }

private:

  std::vector<std::function<void(PatchMsg&, const Change&)>> dispatcher;

};

//==============================================================================
rmf_traffic_msgs::msg::SchedulePatch convert(
    const rmf_traffic::schedule::Database::Patch& patch)
{
  static ChangeDispatcher dispatcher;

  rmf_traffic_msgs::msg::SchedulePatch msg;
  for(const auto& change : patch)
    dispatcher.dispatch(msg, change);

  msg.latest_version = patch.latest_version();

  return msg;
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeInsert& ins)
{
  return Change::make_insert(convert(ins.trajectory), ins.version);
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeInterrupt& intr)
{
  return Change::make_interrupt(
        intr.parent_id, convert(intr.interruption_trajectory),
        Duration(intr.delay), intr.version);
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeDelay& delay)
{
  return Change::make_delay(delay.parent_id, Time(Duration(delay.from_time)),
                            Duration(delay.delay), delay.version);
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeReplace& rep)
{
  return Change::make_replace(rep.parent_id, convert(rep.trajectory),
                              rep.version);
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeErase& er)
{
  return Change::make_erase(er.parent_id, er.version);
}

//==============================================================================
static Change convert(
    const rmf_traffic_msgs::msg::ScheduleChangeCull& cull)
{
  return Change::make_cull(Time(Duration(cull.time)), cull.version);
}

//==============================================================================
rmf_traffic::schedule::Database::Patch convert(
    const rmf_traffic_msgs::msg::SchedulePatch& patch)
{
  std::vector<Change> changes;

  for(const auto& insert : patch.insertions)
    changes.emplace_back(convert(insert));

  for(const auto& interrupt : patch.interruptions)
    changes.emplace_back(convert(interrupt));

  for(const auto& delay : patch.delays)
    changes.emplace_back(convert(delay));

  for(const auto& replace : patch.replacements)
    changes.emplace_back(convert(replace));

  for(const auto& erase : patch.erasures)
    changes.emplace_back(convert(erase));

  for(const auto& cull : patch.culls)
    changes.emplace_back(convert(cull));

  return rmf_traffic::schedule::Database::Patch(
        std::move(changes), patch.latest_version);
}


} // namespace rmf_traffic_ros2
