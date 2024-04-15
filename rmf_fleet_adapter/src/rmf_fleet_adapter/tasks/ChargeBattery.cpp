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
#include "../events/WaitForCancel.hpp"
#include "../events/GoToPlace.hpp"
#include "../events/LegacyPhaseShim.hpp"

#include "ChargeBattery.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

#include <rmf_task_sequence/Task.hpp>

#include <rmf_fleet_adapter/tasks/ParkRobotIndefinitely.hpp>

#include <random>
namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
rmf_traffic::Duration estimate_charge_time(
  const double initial_soc,
  const double recharged_soc,
  const rmf_battery::agv::BatterySystem& battery_system)
{
  if (initial_soc < recharged_soc)
  {
    const double delta_soc = recharged_soc - initial_soc;
    const double dt = (3600 * delta_soc * battery_system.capacity()) /
      battery_system.charging_current();
    return rmf_traffic::time::from_seconds(dt);
  }

  return rmf_traffic::Duration(0);
}

//==============================================================================
// TODO(MXG): Consider making the ChargeBatteryEvent public so it
// can be incorporated into other task types
class ChargeBatteryEvent : public rmf_task_sequence::Event
{
public:

  class Model : public rmf_task_sequence::Activity::Model
  {
  public:
    static rmf_task_sequence::Activity::ConstModelPtr make(
      rmf_task::State invariant_initial_state,
      const rmf_task::Parameters& parameters)
    {
      const auto model = std::shared_ptr<Model>(
        new Model(invariant_initial_state, parameters));
      model->_invariant_initial_state = invariant_initial_state;

      const auto wp_opt = invariant_initial_state.dedicated_charging_waypoint();
      if (wp_opt.has_value())
      {
        const auto go_to_place_desc =
          rmf_task_sequence::events::GoToPlace::Description::make(*wp_opt);
        model->_go_to_place = go_to_place_desc->make_model(
          invariant_initial_state, parameters);
      }

      model->_parameters = parameters;
      return model;
    }

    std::optional<rmf_task::Estimate> estimate_finish(
      rmf_task::State state,
      rmf_traffic::Time earliest_arrival_time,
      const rmf_task::Constraints& constraints,
      const rmf_task::TravelEstimator& travel_estimator) const final
    {
      rmf_traffic::Time wait_until = earliest_arrival_time;
      if (_go_to_place)
      {
        const auto estimate_go_to = _go_to_place->estimate_finish(
          std::move(state), earliest_arrival_time, constraints,
          travel_estimator);
        if (!estimate_go_to.has_value())
          return std::nullopt;

        wait_until = estimate_go_to->wait_until();
        state = estimate_go_to->finish_state();
      }

      const auto recharged_soc = constraints.recharge_soc();
      rmf_traffic::Duration dt = rmf_traffic::Duration(0);
      const auto initial_soc_opt = state.battery_soc();
      if (!initial_soc_opt.has_value())
      {
        // Assume no charging is needed if the "robot" does not have a battery
        return rmf_task::Estimate(state, wait_until);
      }

      const double initial_soc = *initial_soc_opt;
      if (initial_soc < recharged_soc)
      {
        state.battery_soc(recharged_soc);
      }
      wait_until += estimate_charge_time(
        initial_soc,
        recharged_soc,
        _parameters.battery_system());

      return rmf_task::Estimate(state, wait_until);
    }

    rmf_traffic::Duration invariant_duration() const final
    {
      return rmf_traffic::Duration(0);
    }

    rmf_task::State invariant_finish_state() const final
    {
      return _invariant_initial_state;
    }

  private:
    Model(
      rmf_task::State invariant_initial_state,
      rmf_task::Parameters parameters)
    : _invariant_initial_state(std::move(invariant_initial_state)),
      _parameters(std::move(parameters))
    {
      // Do nothing
    }
    rmf_task::State _invariant_initial_state;
    rmf_task_sequence::Activity::ConstModelPtr _go_to_place;
    rmf_task::Parameters _parameters;
  };

  class Description : public rmf_task_sequence::Activity::Description
  {
  public:
    Description(
      std::optional<std::size_t> specific_location_,
      bool indefinite_,
      bool park_)
    : specific_location(specific_location_),
      indefinite(indefinite_),
      park(park_)
    {
      // Do nothing
    }

    std::optional<std::size_t> specific_location;
    bool indefinite;
    bool park;

    rmf_task_sequence::Activity::ConstModelPtr make_model(
      rmf_task::State invariant_initial_state,
      const rmf_task::Parameters& parameters) const final
    {
      return Model::make(std::move(invariant_initial_state), parameters);
    }

    rmf_task::Header generate_header(
      const rmf_task::State& initial_state,
      const rmf_task::Parameters& parameters) const final
    {
      rmf_traffic::Duration duration_estimate = rmf_traffic::Duration(0);
      double recharged_soc = 1.0;
      if (const auto c = initial_state.get<agv::GetContext>())
      {
        if (const auto context = c->value)
        {
          recharged_soc = context->task_planner()
            ->configuration().constraints().recharge_soc();

          const auto header_go_to =
            rmf_task_sequence::events::GoToPlace::Description::make(
            context->dedicated_charging_wp())->generate_header(
            initial_state, parameters);
          duration_estimate += header_go_to.original_duration_estimate();
        }
      }

      double initial_soc = initial_state.battery_soc().value_or(0.0);
      duration_estimate += estimate_charge_time(
        initial_soc, recharged_soc, parameters.battery_system());

      std::string name = park ? "Park" : "Charge Battery";

      return rmf_task::Header(name, "", duration_estimate);
    }
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:
    static std::shared_ptr<Standby> make(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update)
    {
      const auto state = get_state();
      const auto context = state.get<agv::GetContext>()->value;
      const auto header = description.generate_header(state, *parameters);

      auto standby = std::shared_ptr<Standby>(new Standby(description));
      standby->_assign_id = id;
      standby->_context = context;
      standby->_time_estimate = header.original_duration_estimate();
      standby->_update = std::move(update);
      standby->_state = rmf_task::events::SimpleEventState::make(
        id->assign(),
        header.category(),
        header.detail(),
        rmf_task::Event::Status::Standby,
        {},
        context->clock());

      return standby;
    }

    ConstStatePtr state() const final
    {
      return _state;
    }

    rmf_traffic::Duration duration_estimate() const final
    {
      return _time_estimate;
    }

    ActivePtr begin(
      std::function<void()>,
      std::function<void()> finished) final
    {
      if (!_active)
      {
        const std::string& task = _desc.park ? "parking" : "charging";

        RCLCPP_INFO(
          _context->node()->get_logger(),
          "Beginning a new %s task for robot [%s]",
          task.c_str(),
          _context->requester_id().c_str());

        _active = Active::make(
          _desc,
          _assign_id,
          _context,
          _state,
          _update,
          std::move(finished));
      }

      return _active;
    }

  private:
    Standby(Description desc)
    : _desc(desc)
    {
      // Do nothing
    }
    Description _desc;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    rmf_traffic::Duration _time_estimate;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state;
    ActivePtr _active = nullptr;
  };

  class Active : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:
    static std::shared_ptr<Active> make(
      Description desc,
      AssignIDPtr assign_id,
      agv::RobotContextPtr context,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> update,
      std::function<void()> finished)
    {
      auto active = std::shared_ptr<Active>(new Active(desc));
      active->_assign_id = std::move(assign_id);
      active->_context = std::move(context);
      active->_state = std::move(state);
      active->_update = std::move(update);
      active->_finished = std::move(finished);

      active->_charging_update_subscription = active
        ->_context
        ->observe_charging_change()
        .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
        .subscribe(
        [w = active->weak_from_this()](const auto&)
        {
          const auto self = w.lock();
          if (!self)
            return;

          self->_consider_restart();
        });

      active->_consider_restart();
      return active;
    }

    rmf_task::Event::ConstStatePtr state() const final
    {
      return _state;
    }

    rmf_traffic::Duration remaining_time_estimate() const final
    {
      if (_sequence)
        return _sequence->remaining_time_estimate();

      return rmf_traffic::Duration(0);
    }

    Backup backup() const final
    {
      return Backup::make(0, nlohmann::json());
    }

    Resume interrupt(std::function<void()> task_is_interrupted) final
    {
      if (_sequence)
        _sequence->interrupt(std::move(task_is_interrupted));

      _current_target_wp = std::nullopt;
      _current_waiting_for_charger = std::nullopt;
      return Resume::make(
        [w = weak_from_this()]()
        {
          if (const auto self = w.lock())
          {
            self->_consider_restart();
          }
        });
    }

    void cancel() final
    {
      if (_sequence)
        _sequence->cancel();
    }

    void kill() final
    {
      if (_sequence)
        _sequence->kill();
    }

  private:

    Active(Description desc)
    : _desc(desc)
    {
      // Do nothing
    }
    void _consider_restart()
    {
      std::size_t target_wp = _context->dedicated_charging_wp();
      if (!_desc.specific_location.has_value())
      {
        bool location_changed = true;
        if (_current_target_wp.has_value())
        {
          if (target_wp == *_current_target_wp)
          {
            location_changed = false;
          }
        }

        bool waiting_changed = true;
        if (_current_waiting_for_charger.has_value())
        {
          if (_context->waiting_for_charger() == *_current_waiting_for_charger)
          {
            waiting_changed = false;
          }
        }


        if (!location_changed && !waiting_changed)
        {
          // No need to do anything, the charging location has not changed
          // nor has the mode changed.
          return;
        }
      }
      else
      {
        target_wp = _desc.specific_location.value();
        if (_current_target_wp == target_wp)
          return;
      }

      _current_target_wp = target_wp;
      _current_waiting_for_charger = _context->waiting_for_charger();

      using UpdateFn = std::function<void()>;
      using MakeStandby = std::function<StandbyPtr(UpdateFn)>;
      std::vector<MakeStandby> standbys;

      using GoToPlaceDesc = rmf_task_sequence::events::GoToPlace::Description;
      standbys.push_back(
        [
          target_wp,
          assign_id = _assign_id,
          context = _context
        ](UpdateFn update) -> StandbyPtr
        {
          return events::GoToPlace::Standby::make(
            assign_id,
            context->make_get_state(),
            context->task_parameters(),
            *GoToPlaceDesc::make(target_wp),
            update);
        });

      if (_desc.park)
      {
        standbys.push_back(
          [
            assign_id = _assign_id,
            context = _context
          ](UpdateFn update) -> StandbyPtr
          {
            return events::WaitForCancel::Standby::make(context, assign_id);
          });
      }
      else
      {
        standbys.push_back(
          [
            assign_id = _assign_id,
            context = _context,
            indefinite = _desc.indefinite
          ](UpdateFn update) -> StandbyPtr
          {
            std::optional<double> recharged_soc;
            if (!indefinite)
            {
              recharged_soc = context->task_planner()
              ->configuration().constraints().recharge_soc();
            }

            auto legacy = phases::WaitForCharge::make(
              context,
              context->task_parameters()->battery_system(),
              recharged_soc);

            return events::LegacyPhaseShim::Standby::make(
              std::move(legacy), context->worker(), context->clock(), assign_id,
              std::move(update));
          });
      }

      _sequence = rmf_task_sequence::events::Bundle::standby(
        rmf_task_sequence::events::Bundle::Type::Sequence,
        standbys, _state, _update)->begin([]() {}, _finished);
    }

    Description _desc;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _update;
    std::function<void()> _finished;
    rmf_rxcpp::subscription_guard _charging_update_subscription;
    std::optional<std::size_t> _current_target_wp;
    std::optional<bool> _current_waiting_for_charger;
    rmf_task_sequence::Event::ActivePtr _sequence;
  };
};

//==============================================================================
struct GoToChargerDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  GoToChargerDescription()
  : rmf_task_sequence::events::Placeholder::Description("Go to place", "")
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
      *GoToPlace::make(context->dedicated_charging_wp()),
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
  WaitForChargeDescription(bool indefinite_)
  : rmf_task_sequence::events::Placeholder::Description(
      "Wait for charging", ""),
    indefinite(indefinite_)
  {
    // Do nothing
  }

  bool indefinite;

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const WaitForChargeDescription& desc,
    std::function<void()> update)
  {
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    std::optional<double> recharged_soc;
    if (!desc.indefinite)
    {
      recharged_soc = context->task_planner()
        ->configuration().constraints().recharge_soc();
    }

    auto legacy = phases::WaitForCharge::make(
      context,
      parameters->battery_system(),
      recharged_soc);

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
void add_charge_battery(
  rmf_task::Activator& task_activator,
  const rmf_task_sequence::Phase::ConstActivatorPtr& phase_activator,
  rmf_task_sequence::Event::Initializer& event_initializer,
  std::function<rmf_traffic::Time()> clock)
{
  using Bundle = rmf_task_sequence::events::Bundle;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using ChargeBatteryTask = rmf_task::requests::ChargeBattery;

  event_initializer.add<ChargeBatteryEvent::Description>(
    [](
      const rmf_task::Event::AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const ChargeBatteryEvent::Description& description,
      std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
    {
      return ChargeBatteryEvent::Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [](
      const rmf_task::Event::AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const ChargeBatteryEvent::Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
    {
      return ChargeBatteryEvent::Standby::make(
        id, get_state, parameters, description, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });

  auto charge_battery_task_unfolder =
    [](const rmf_task::requests::ChargeBattery::Description& desc)
    {
      rmf_task_sequence::Task::Builder builder;
      builder
      .add_phase(
        Phase::Description::make(
          std::make_shared<ChargeBatteryEvent::Description>(
            std::nullopt, desc.indefinite(), false),
          "Charge Battery", ""), {});

      return *builder.build("Charge Battery", "");
    };

  rmf_task_sequence::Task::unfold<ChargeBatteryTask::Description>(
    std::move(charge_battery_task_unfolder),
    task_activator,
    phase_activator,
    std::move(clock));
}

//==============================================================================
namespace {
std::string generate_uuid(const std::size_t length = 3)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

} // anonymous namespace

//==============================================================================
class ParkRobotIndefinitely::Implementation
{
public:
  std::string requester;
  std::function<rmf_traffic::Time()> time_now_cb;
  std::optional<std::size_t> parking_waypoint;
};

//==============================================================================
ParkRobotIndefinitely::ParkRobotIndefinitely(
  const std::string& requester,
  std::function<rmf_traffic::Time()> time_now_cb,
  std::optional<std::size_t> parking_waypoint)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation {
      requester,
      time_now_cb,
      parking_waypoint
    }))
{
  // Do nothing
}

//==============================================================================
rmf_task::ConstRequestPtr ParkRobotIndefinitely::make_request(
  const rmf_task::State& state) const
{
  std::string id = "ParkRobot-" + generate_uuid();
  auto phase_desc = std::make_shared<ChargeBatteryEvent::Description>(
    _pimpl->parking_waypoint, true, true);

  auto desc = rmf_task_sequence::Task::Builder()
    .add_phase(rmf_task_sequence::phases::SimplePhase::Description::make(
        phase_desc), {})
    .build("Park", "");

  auto now = _pimpl->time_now_cb ?
    _pimpl->time_now_cb() : state.time().value_or(
    rmf_traffic::Time(std::chrono::system_clock::now().time_since_epoch()));
  rmf_task::Task::ConstBookingPtr booking =
    std::make_shared<rmf_task::Task::Booking>(
    std::move(id),
    now,
    nullptr,
    _pimpl->requester,
    now,
    true);

  return std::make_shared<rmf_task::Request>(
    std::move(booking),
    std::move(desc));
}

} // namespace task
} // namespace rmf_fleet_adapter
