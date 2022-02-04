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

#include "Loop.hpp"

#include "../phases/GoToPlace.hpp"

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>

#include <rmf_fleet_adapter/schemas/event_description_GoToPlace.hpp>
#include <rmf_fleet_adapter/schemas/task_description_Patrol.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<LegacyTask> make_loop(
  const rmf_task::ConstRequestPtr request,
  const agv::RobotContextPtr& context,
  const rmf_traffic::agv::Plan::Start start,
  const rmf_traffic::Time deployment_time,
  const rmf_task::State finish_state)
{
  std::shared_ptr<const rmf_task::requests::Loop::Description> description =
    std::dynamic_pointer_cast<
    const rmf_task::requests::Loop::Description>(request->description());

  if (description == nullptr)
    return nullptr;

  const auto start_waypoint = description->start_waypoint();
  const auto finish_waypoint = description->finish_waypoint();

  const auto loop_start = [&]() -> rmf_traffic::agv::Planner::Start
    {
      if (start.waypoint() == start_waypoint)
        return start;

      rmf_traffic::agv::Planner::Goal goal{start_waypoint};

      const auto result = context->planner()->plan(start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
      finish_time,
      start_waypoint,
      orientation};
    } ();

  const auto loop_end = [&]() -> rmf_traffic::agv::Planner::Start
    {
      if (loop_start.waypoint() == finish_waypoint)
        return loop_start;

      rmf_traffic::agv::Planner::Goal goal{finish_waypoint};

      const auto result = context->planner()->plan(loop_start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
      finish_time,
      finish_waypoint,
      orientation};
    } ();

  LegacyTask::PendingPhases phases;
  phases.push_back(
    phases::GoToPlace::make(
      context, std::move(start), start_waypoint));

  phases.push_back(
    phases::GoToPlace::make(
      context, loop_start, finish_waypoint));

  for (std::size_t i = 1; i < description->num_loops(); ++i)
  {
    phases.push_back(
      phases::GoToPlace::make(
        context, loop_end, start_waypoint));

    phases.push_back(
      phases::GoToPlace::make(
        context, loop_start, finish_waypoint));
  }

  return LegacyTask::make(
    request->booking()->id(),
    std::move(phases),
    context->worker(),
    deployment_time,
    finish_state,
    request);
}

//==============================================================================
void add_loop(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock)
{
  using Loop = rmf_task::requests::Loop;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using GoToPlace = rmf_task_sequence::events::GoToPlace;

  deserialization.add_schema(schemas::task_description_Patrol);
  deserialization.add_schema(schemas::event_description_GoToPlace);
  auto validate_go_to_place =
    deserialization.make_validator_shared(schemas::event_description_GoToPlace);

  auto deserialize_go_to_place =
    [place_deser = deserialization.place](const nlohmann::json& msg)
      -> agv::DeserializedEvent
    {
      auto place = place_deser(msg);
      if (!place.description.has_value())
        return {nullptr, std::move(place.errors)};

      return {
        GoToPlace::Description::make(std::move(*place.description)),
        std::move(place.errors)
      };
    };

  deserialization.event->add(
    "go_to_place", validate_go_to_place, deserialize_go_to_place);

  auto validate_patrol =
    deserialization.make_validator_shared(schemas::task_description_Patrol);

  deserialization.consider_patrol =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();

  // Accept patrol tasks by default
  *deserialization.consider_patrol = [](const auto&, auto& confirm)
    {
      confirm.accept();
    };

  auto deserialize_patrol =
    [
      place_deser = deserialization.place,
      consider = deserialization.consider_patrol
    ](const nlohmann::json& msg) -> agv::DeserializedTask
    {
      if (!(*consider))
        return {nullptr, {"Not accepting patrol requests"}};

      const auto& places_json = msg.at("places");
      std::vector<rmf_traffic::agv::Plan::Goal> places;
      std::vector<std::string> errors;
      bool any_failure = false;
      for (const auto& place_json : places_json)
      {
        const auto place = place_deser(place_json);
        if (!place.description.has_value())
          any_failure = true;
        else
          places.push_back(*place.description);

        errors.insert(errors.begin(), place.errors.begin(), place.errors.end());
      }

      if (any_failure)
        return {nullptr, std::move(errors)};

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      errors.insert(
        errors.end(), confirm.errors().begin(), confirm.errors().end());

      if (!confirm.is_accepted())
        return {nullptr, errors};

      std::size_t rounds = 1;
      const auto& rounds_json_it = msg.find("rounds");
      if (rounds_json_it != msg.end())
        rounds = rounds_json_it->get<std::size_t>();

      rmf_task_sequence::Task::Builder builder;
      for (std::size_t i=0; i < rounds; ++i)
      {
        for (const auto& place : places)
        {
          builder.add_phase(
            Phase::Description::make(GoToPlace::Description::make(place)), {});
        }
      }

      return {builder.build("Patrol", ""), std::move(errors)};
    };

  deserialization.task->add("patrol", validate_patrol, deserialize_patrol);

  auto loop_unfolder =
    [](const Loop::Description& loop)
    {
      rmf_task_sequence::Task::Builder builder;
      for (std::size_t i=0; i < loop.num_loops(); ++i)
      {
        builder
          .add_phase(
            Phase::Description::make(
              GoToPlace::Description::make(loop.start_waypoint())), {})
          .add_phase(
            Phase::Description::make(
              GoToPlace::Description::make(loop.finish_waypoint())), {});
      }

      // TODO(MXG): Consider making the category and detail more details
      return *builder.build("Loop", "");
    };

  rmf_task_sequence::Task::unfold<rmf_task::requests::Loop::Description>(
    std::move(loop_unfolder), *activation.task,
    activation.phase, std::move(clock));
}

} // namespace tasks
} // namespace rmf_fleet_adapter
