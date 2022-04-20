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

#include "Patrol.hpp"

#include <rmf_task/requests/Loop.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>

#include <rmf_fleet_adapter/schemas/event_description__go_to_place.hpp>
#include <rmf_fleet_adapter/schemas/task_description__patrol.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_patrol(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock)
{
  using Loop = rmf_task::requests::Loop;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using GoToPlace = rmf_task_sequence::events::GoToPlace;

  deserialization.add_schema(schemas::task_description__patrol);
  deserialization.add_schema(schemas::event_description__go_to_place);
  auto validate_go_to_place =
    deserialization.make_validator_shared(
    schemas::event_description__go_to_place);

  auto deserialize_go_to_place =
    [place_deser = deserialization.place](const nlohmann::json& msg)
    -> agv::DeserializedEvent
    {
      nlohmann::json place_msg;
      const auto place_it = msg.find("place");
      if (place_it == msg.end())
        place_msg = msg;
      else
        place_msg = place_it.value();

      auto place = place_deser(place_msg);
      if (!place.description.has_value())
        return {nullptr, std::move(place.errors)};

      const auto desc =
        GoToPlace::Description::make(std::move(*place.description));

      const auto followed_by_it = msg.find("followed_by");
      if (followed_by_it != msg.end())
      {
        std::vector<rmf_traffic::agv::Plan::Goal> followed_by;
        for (const auto& f_msg : followed_by_it.value())
        {
          auto f = place_deser(f_msg);
          place.errors.insert(
            place.errors.end(), f.errors.begin(), f.errors.end());

          if (!f.description.has_value())
            return {nullptr, place.errors};

          followed_by.push_back(*f.description);
        }
        desc->expected_next_destinations(std::move(followed_by));
      }

      /* *INDENT-OFF* */
      return {desc, std::move(place.errors)};
      /* *INDENT-ON* */
    };

  deserialization.event->add(
    "go_to_place", validate_go_to_place, deserialize_go_to_place);

  auto validate_patrol =
    deserialization.make_validator_shared(schemas::task_description__patrol);

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
      for (std::size_t i = 0; i < rounds; ++i)
      {
        for (std::size_t j = 0; j < places.size(); ++j)
        {
          const auto& place = places[j];
          auto go_to_place = GoToPlace::Description::make(place);
          auto next = places;
          next.erase(next.begin(), next.begin() + j + 1);
          go_to_place->expected_next_destinations(std::move(next));
          builder.add_phase(Phase::Description::make(go_to_place), {});
        }
      }

      return {builder.build("Patrol", ""), std::move(errors)};
    };

  deserialization.task->add("patrol", validate_patrol, deserialize_patrol);

  auto loop_unfolder =
    [](const Loop::Description& loop)
    {
      rmf_task_sequence::Task::Builder builder;
      for (std::size_t i = 0; i < loop.num_loops(); ++i)
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
