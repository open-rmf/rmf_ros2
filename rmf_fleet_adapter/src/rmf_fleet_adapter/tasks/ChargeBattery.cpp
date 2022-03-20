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

#include "../phases/WaitForCharge.hpp"

#include "../events/GoToPlace.hpp"
#include "../events/LegacyPhaseShim.hpp"

#include "ChargeBattery.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

#include <rmf_task_sequence/Task.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
struct GoToChargerDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  GoToChargerDescription()
  : rmf_task_sequence::events::Placeholder::Description("Go to charger", "")
  {
    // Do nothing
  }

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const GoToChargerDescription&,
    std::function<void()> update)
  {
    using GoToPlace = rmf_task_sequence::events::GoToPlace::Description;
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    return events::GoToPlace::Standby::make(
      id, get_state, parameters,
      *GoToPlace::make(context->dedicated_charger_wp()),
      std::move(update));
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<GoToChargerDescription>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const GoToChargerDescription& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const GoToChargerDescription& description,
        const nlohmann::json&,
        std::function<void()> update,
        std::function<void()> checkpoint,
        std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update))
        ->begin(std::move(checkpoint), std::move(finished));
      });
  }
};

//==============================================================================
struct WaitForChargeDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  WaitForChargeDescription()
  : rmf_task_sequence::events::Placeholder::Description(
      "Wait for charging", "")
  {
    // Do nothing
  }

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const WaitForChargeDescription&,
    std::function<void()> update)
  {
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    auto legacy = phases::WaitForCharge::make(
      context,
      parameters->battery_system(),
      context->task_planner()->configuration().constraints().recharge_soc());

    return events::LegacyPhaseShim::Standby::make(
      std::move(legacy), context->worker(), context->clock(), id,
      std::move(update));
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<WaitForChargeDescription>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const WaitForChargeDescription& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const WaitForChargeDescription& description,
        const nlohmann::json&,
        std::function<void()> update,
        std::function<void()> checkpoint,
        std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update))
        ->begin(std::move(checkpoint), std::move(finished));
      });
  }
};

//==============================================================================
// TODO(MXG): Consider making the ChargeBatteryEvent description public so it
// can be incorporated into other task types
struct ChargeBatteryEventDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  ChargeBatteryEventDescription()
  : rmf_task_sequence::events::Placeholder::Description(
      "Charge Battery", "")
  {
    // Do nothing
  }
};

//==============================================================================
void add_charge_battery(
  rmf_task::Activator& task_activator,
  const rmf_task_sequence::Phase::ConstActivatorPtr& phase_activator,
  rmf_task_sequence::Event::Initializer& event_initializer,
  std::function<rmf_traffic::Time()> clock)
{
  using Bundle = rmf_task_sequence::events::Bundle;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using ChargeBattery = rmf_task::requests::ChargeBattery;

  auto private_initializer =
    std::make_shared<rmf_task_sequence::Event::Initializer>();

  WaitForChargeDescription::add(*private_initializer);
  GoToChargerDescription::add(*private_initializer);

  auto charge_battery_event_unfolder =
    [](const ChargeBatteryEventDescription&)
    {
      return Bundle::Description({
            std::make_shared<GoToChargerDescription>(),
            std::make_shared<WaitForChargeDescription>()
          }, Bundle::Sequence, "Charge Battery");
    };

  Bundle::unfold<ChargeBatteryEventDescription>(
    std::move(charge_battery_event_unfolder),
    event_initializer, private_initializer);

  auto charge_battery_task_unfolder =
    [](const rmf_task::requests::ChargeBattery::Description&)
    {
      rmf_task_sequence::Task::Builder builder;
      builder
      .add_phase(
        Phase::Description::make(
          std::make_shared<ChargeBatteryEventDescription>(),
          "Charge Battery", ""), {});

      return *builder.build("Charge Battery", "");
    };

  rmf_task_sequence::Task::unfold<ChargeBattery::Description>(
    std::move(charge_battery_task_unfolder),
    task_activator,
    phase_activator,
    std::move(clock));
}

} // namespace task
} // namespace rmf_fleet_adapter
