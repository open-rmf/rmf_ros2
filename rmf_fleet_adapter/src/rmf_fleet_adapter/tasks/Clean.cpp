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

#include "Clean.hpp"

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>
#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>

#include "../events/Error.hpp"
#include "../events/GoToPlace.hpp"

#include <rmf_fleet_adapter/schemas/event_description__clean.hpp>
#include <rmf_fleet_adapter/schemas/task_description__clean.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
// TODO(MXG): This implementation that uses the Dock command is crude and
// should be replaced with an explicit cleaning activation command.
struct CleanEvent : public rmf_task_sequence::events::Placeholder::Description
{
  CleanEvent(const rmf_task::requests::Clean::Description& clean)
  : rmf_task_sequence::events::Placeholder::Description("Clean", ""),
    start_waypoint(clean.start_waypoint()),
    end_waypoint(clean.end_waypoint())
  {
    // Do nothing
  }

  struct DockChecker : public rmf_traffic::agv::Graph::Lane::Executor
  {
    void execute(const Dock& dock) final { found_dock = dock.dock_name(); }
    void execute(const Wait&) final {}
    void execute(const DoorOpen&) final {}
    void execute(const LiftMove&) final {}
    void execute(const DoorClose&) final {}
    void execute(const LiftDoorOpen&) final {}
    void execute(const LiftSessionEnd&) final {}
    void execute(const LiftSessionBegin&) final {}

    std::optional<std::string> found_dock = std::nullopt;
  };

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const CleanEvent& description,
    std::function<void()> update)
  {
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    // Check if going to the start waypoint from the robot's current location
    // will successfully pass through the dock event
    const auto result = context->planner()->plan(
      context->location(),
      description.start_waypoint, {nullptr});

    // TODO(MXG): Make this name more detailed
    const std::string name = "Clean";

    if (!result.success())
    {
      const auto error_state = rmf_task::events::SimpleEventState::make(
        id->assign(), name, "", rmf_task::Event::Status::Error, {},
        context->clock());

      error_state->update_log().error(
        "Could not find a path to the cleaning zone from the robot's current "
        "location");

      return events::Error::Standby::make(std::move(error_state));
    }

    bool will_pass_through_dock = false;
    const auto& graph = context->planner()->get_configuration().graph();
    for (const auto& wp : result->get_waypoints())
    {
      if (!wp.graph_index().has_value())
        continue;

      if (*wp.graph_index() == description.start_waypoint)
      {
        for (const auto& l : wp.approach_lanes())
        {
          DockChecker check;
          const auto& lane = graph.get_lane(l);
          if (lane.entry().event())
          {
            if (lane.entry().event()->execute(check).found_dock.has_value())
            {
              will_pass_through_dock = true;
              break;
            }
          }

          if (lane.exit().event())
          {
            if (lane.exit().event()->execute(check).found_dock.has_value())
            {
              will_pass_through_dock = true;
              break;
            }
          }
        }
      }
    }

    using StandbyPtr = rmf_task_sequence::Event::StandbyPtr;
    using UpdateFn = std::function<void()>;
    using MakeStandby = std::function<StandbyPtr(UpdateFn)>;
    using GoToPlaceDesc = rmf_task_sequence::events::GoToPlace::Description;

    auto go_to_place =
      [id, get_state, parameters](std::size_t wp)
      {
        return [id, get_state, parameters, wp](UpdateFn update)
          {
            return events::GoToPlace::Standby::make(
              id, get_state, parameters,
              *GoToPlaceDesc::make(wp), std::move(update));
          };
      };

    std::vector<MakeStandby> events;
    if (!will_pass_through_dock)
    {
      // If the robot does not pass through the dock on its way to the start
      // location then it is likely already sitting on the start location. We
      // should order the robot to the exit first and then back to the start.
      //
      // This strategy still has flaws because we can't rule out that by the
      // time the robot is generating its plan to reach the exit it might
      // accidentally trigger the docking action. This is one of the reasons we
      // should fix the overall implementation.
      events.push_back(go_to_place(description.end_waypoint));
    }

    events.push_back(go_to_place(description.start_waypoint));
    events.push_back(go_to_place(description.end_waypoint));

    auto sequence_state = rmf_task::events::SimpleEventState::make(
      id->assign(), name, "", rmf_task::Event::Status::Standby, {},
      context->clock());

    return rmf_task_sequence::events::Bundle::standby(
      rmf_task_sequence::events::Bundle::Type::Sequence,
      events, std::move(sequence_state), std::move(update));
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<CleanEvent>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const CleanEvent& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const CleanEvent& description,
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

  std::size_t start_waypoint;
  std::size_t end_waypoint;
};

//==============================================================================
void add_clean(
  const agv::FleetUpdateHandle::Implementation::ConstDockParamsPtr& dock_params,
  const rmf_traffic::agv::VehicleTraits& traits,
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock)
{
  using Clean = rmf_task::requests::Clean;
  using Phase = rmf_task_sequence::phases::SimplePhase;

  auto validate_clean_event =
    deserialization.make_validator_shared(
    schemas::event_description__clean);
  deserialization.add_schema(schemas::event_description__clean);

  auto validate_clean_task =
    deserialization.make_validator_shared(
    schemas::task_description__clean);
  deserialization.add_schema(schemas::task_description__clean);

  deserialization.consider_clean =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();

  auto deserialize_clean =
    [
    dock_params,
    traits,
    place_deser = deserialization.place,
    consider = deserialization.consider_clean
    ](const nlohmann::json& msg) -> agv::DeserializedTask
    {
      if (!consider || !(*consider))
      {
        /* *INDENT-OFF* */
        return {
          nullptr,
          {"Not accepting cleaning tasks"}
        };
        /* *INDENT-ON* */
      }

      const auto zone = msg.at("zone").get<std::string>();
      const auto clean_it = dock_params->find(zone);
      if (clean_it == dock_params->end())
      {
        /* *INDENT-OFF* */
        return {
          nullptr,
          {"No cleaning zone named [" + zone + "] for this fleet adapter"}
        };
        /* *INDENT-ON* */
      }

      const auto& clean_info = clean_it->second;
      auto start_place = place_deser(clean_info.start);
      auto exit_place = place_deser(clean_info.finish);
      if (!start_place.description.has_value()
        || !exit_place.description.has_value())
      {
        auto errors = std::move(start_place.errors);
        errors.insert(
          errors.end(), exit_place.errors.begin(), exit_place.errors.end());
        return {nullptr, std::move(errors)};
      }

      std::vector<Eigen::Vector3d> positions;
      for (const auto& p : clean_info.path)
        positions.push_back({p.x, p.y, p.yaw});

      rmf_traffic::Trajectory clean_path =
        rmf_traffic::agv::Interpolate::positions(
        traits,
        rmf_traffic::Time(rmf_traffic::Duration(0)),
        positions);

      if (clean_path.size() < 2)
      {
        /* *INDENT-OFF* */
        return {
          nullptr,
          {"Invalid cleaning path for zone named [" + zone
            + "]: Too few waypoints [" + std::to_string(clean_path.size())
            + "]"}
        };
        /* *INDENT-ON* */
      }

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      if (!confirm.is_accepted())
        return {nullptr, confirm.errors()};

      // TODO(MXG): Validate the type of cleaning (vacuum, mopping, etc)
      /* *INDENT-OFF* */
      return {
        rmf_task::requests::Clean::Description::make(
          start_place.description->waypoint(),
          exit_place.description->waypoint(),
          std::move(clean_path)),
        confirm.errors()
      };
      /* *INDENT-ON* */
    };
  deserialization.task->add("clean", validate_clean_task, deserialize_clean);

  auto deserialize_clean_event =
    [deserialize_clean](const nlohmann::json& msg) -> agv::DeserializedEvent
    {
      auto clean_task = deserialize_clean(msg);
      if (!clean_task.description)
        return {nullptr, std::move(clean_task.errors)};

      /* *INDENT-OFF* */
      return {
        std::make_shared<CleanEvent>(
          static_cast<const Clean::Description&>(*clean_task.description)),
          std::move(clean_task.errors)
      };
      /* *INDENT-ON* */
    };
  deserialization.event->add(
    "clean", validate_clean_event, deserialize_clean_event);

  CleanEvent::add(*activation.event);

  auto clean_unfolder =
    [](const Clean::Description& clean)
    {
      rmf_task_sequence::Task::Builder builder;
      builder.add_phase(
        Phase::Description::make(std::make_shared<CleanEvent>(clean)), {});

      // TODO(MXG): Make the name and detail more detailed
      return *builder.build("Clean", "");
    };

  rmf_task_sequence::Task::unfold<rmf_task::requests::Clean::Description>(
    std::move(clean_unfolder), *activation.task,
    activation.phase, std::move(clock));
}

} // namespace task
} // namespace rmf_fleet_adapter
