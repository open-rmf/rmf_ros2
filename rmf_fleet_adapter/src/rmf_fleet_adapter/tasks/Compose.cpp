/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "Compose.hpp"

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Bundle.hpp>

#include <rmf_fleet_adapter/schemas/task_description__compose.hpp>
#include <rmf_fleet_adapter/schemas/event_description__sequence.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_compose(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock)
{
  deserialization.add_schema(
    rmf_fleet_adapter::schemas::task_description__compose);

  deserialization.add_schema(
    rmf_fleet_adapter::schemas::event_description__sequence);

  auto validate_compose_task =
    deserialization.make_validator_shared(schemas::task_description__compose);

  auto deserialize_activity =
    [
    deser_phase = deserialization.phase,
    deser_event = deserialization.event
    ](const nlohmann::json& msg) -> agv::DeserializedPhase
    {
      const auto& category = msg.at("category").get<std::string>();
      const auto& description_json = msg.at("description");

      const auto p_it = deser_phase->handlers.find(category);
      if (p_it != deser_phase->handlers.end())
      {
        // There is a specialized phase for this category, so we will
        // deserialize and use that.
        auto phase = p_it->second.deserializer(description_json);
        if (!phase.description)
          return {nullptr, std::move(phase.errors)};

        return phase;
      }

      // There is no specialized phase for this category, so we will check for
      // an event that has the same category name.
      const auto e_it = deser_event->handlers.find(category);
      if (e_it == deser_event->handlers.end())
      {
        // There is no specialized phase or event for this category, so we
        // cannot support this task.
        return {nullptr, {"No support for [" + category + "] activities"}};
      }

      auto event = e_it->second.deserializer(description_json);
      if (!event.description)
        return {nullptr, std::move(event.errors)};

      using rmf_task_sequence::phases::SimplePhase;
      /* *INDENT-OFF* */
      return {
        SimplePhase::Description::make(event.description),
        std::move(event.errors)
      };
      /* *INDENT-ON* */
    };

  deserialization.consider_composed =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();

  // Accept composed tasks by default
  *deserialization.consider_composed = [](const auto&, auto& confirm)
    {
      confirm.accept();
    };

  auto deserialize_compose_task =
    [
    deser_activity = deserialize_activity,
    consider = deserialization.consider_composed
    ](const nlohmann::json& msg)
    -> agv::DeserializedTask
    {
      if (!(*consider))
        return {nullptr, {"Not accepting composed requests"}};

      rmf_task_sequence::Task::Builder builder;
      std::string category = "Composed Task";
      const auto category_it = msg.find("category");
      if (category_it != msg.end() && category_it->is_string())
        category = category_it->get<std::string>();

      std::string detail = "";
      const auto detail_it = msg.find("detail");
      if (detail_it != msg.end() && detail_it->is_string())
        detail = detail_it->get<std::string>();

      std::vector<std::string> errors;
      const auto& phases_json = msg.at("phases");
      for (const auto& phase_json : phases_json)
      {
        auto phase_activity = deser_activity(phase_json.at("activity"));
        if (!phase_activity.description)
          return {nullptr, std::move(phase_activity.errors)};

        errors.insert(
          errors.end(),
          phase_activity.errors.begin(), phase_activity.errors.end());

        std::vector<rmf_task_sequence::Phase::ConstDescriptionPtr> cancel = {};
        const auto cancel_it = phase_json.find("on_cancel");
        if (cancel_it != phase_json.end())
        {
          if (cancel_it.value().is_array())
          {
            const auto& on_cancel_json = cancel_it.value();
            cancel.reserve(on_cancel_json.size());
            for (const auto& cancellation_activity_json : on_cancel_json)
            {
              auto cancellation_activity =
                deser_activity(cancellation_activity_json);

              if (!cancellation_activity.description)
                return {nullptr, std::move(cancellation_activity.errors)};

              cancel.push_back(cancellation_activity.description);
              errors.insert(
                errors.end(),
                cancellation_activity.errors.begin(),
                cancellation_activity.errors.end());
            }
          }
          else
          {
            errors.push_back("The on_cancel property for one of the phases is "
              "not an array. The intended activities will not be performed.");
          }
        }

        builder.add_phase(phase_activity.description, cancel);
      }

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      if (!confirm.is_accepted())
        return {nullptr, confirm.errors()};

      /* *INDENT-OFF* */
      return {
        builder.build(category, detail),
        std::move(errors)
      };
      /* *INDENT-ON* */
    };

  using rmf_task_sequence::events::Bundle;
  using DeserializedDependencies =
    agv::DeserializedDescription<
    std::optional<Bundle::Description::Dependencies>>;

  // Deserialize composed tasks
  deserialization.task->add(
    "compose", validate_compose_task, deserialize_compose_task);

  // Activate composed tasks
  rmf_task_sequence::Task::add(*activation.task, activation.phase, clock);

  auto validate_event_sequence =
    deserialization.make_validator_shared(schemas::event_description__sequence);

  auto deserialize_event_dependencies =
    [deser_event = deserialization.event](const nlohmann::json& msg)
    -> DeserializedDependencies
    {
      std::vector<std::string> errors;
      Bundle::Description::Dependencies deps;
      deps.reserve(msg.size());
      for (const auto& activity_json : msg)
      {
        const auto& category = activity_json.at("category").get<std::string>();
        const auto& description_json = activity_json.at("description");
        const auto e_it = deser_event->handlers.find(category);
        if (e_it == deser_event->handlers.end())
        {
          errors.push_back("No support for [" + category + "] activities");
          return {std::nullopt, std::move(errors)};
        }

        auto event = e_it->second.deserializer(description_json);
        errors.insert(errors.end(), event.errors.begin(), event.errors.end());
        if (!event.description)
          return {std::nullopt, std::move(errors)};

        deps.push_back(event.description);
      }

      return {std::move(deps), std::move(errors)};
    };

  auto deserialize_event_sequence =
    [deserialize_event_dependencies](const nlohmann::json& msg)
    -> agv::DeserializedEvent
    {
      DeserializedDependencies dependencies;
      std::optional<std::string> category;
      std::optional<std::string> detail;
      if (msg.is_array())
      {
        dependencies = deserialize_event_dependencies(msg);
      }
      else
      {
        dependencies = deserialize_event_dependencies(msg.at("activities"));

        const auto category_it = msg.find("category");
        if (category_it != msg.end() && category_it->is_string())
          category = category_it->get<std::string>();

        const auto detail_it = msg.find("detail");
        if (detail_it != msg.end() && detail_it->is_string())
          detail = detail_it->get<std::string>();
      }

      if (!dependencies.description.has_value())
        return {nullptr, std::move(dependencies.errors)};

      /* *INDENT-OFF* */
      return {
        std::make_shared<Bundle::Description>(
          *dependencies.description,
          Bundle::Type::Sequence,
          std::move(category),
          std::move(detail)),
        std::move(dependencies.errors)
      };
      /* *INDENT-ON* */
    };

  // Deserialize activity sequences
  deserialization.event->add(
    "sequence", validate_event_sequence, deserialize_event_sequence);

  // Activate activity sequences
  rmf_task_sequence::events::Bundle::add(activation.event);
}

} // namespace tasks
} // namespace rmf_fleet_adapter
