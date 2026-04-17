#include "Zone.hpp"

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/GoToZone.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>

#include <rmf_fleet_adapter/schemas/event_description__zone.hpp>
#include <rmf_fleet_adapter/schemas/task_description__zone.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
using DeserializedGTZEvent =
  DeserializedDescription<
  std::shared_ptr<const rmf_task_sequence::events::GoToZone::Description>
  >;

void add_zone(
  agv::TaskDeserialization& deserialization)
{
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using GoToZone = rmf_task_sequence::events::GoToZone;
  using GoToPlace = rmf_task_sequence::events::GoToPlace;

  deserialization.add_schema(schemas::task_description__zone);
  deserialization.add_schema(schemas::event_description__zone);

  auto validate_zone_event =
    deserialization.make_validator_shared(schemas::event_description__zone);
  auto validate_zone_task =
    deserialization.make_validator_shared(schemas::task_description__zone);

  deserialization.consider_zone =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();

  // Accept zone tasks/event by default
  *deserialization.consider_zone = [](const auto&, auto& confirm)
    {
      confirm.accept();
    };

  auto deserialize_zone =
    [
      zone_deser = deserialization.zone,
      zone_wp_deser = deserialization.zone_waypoint,
      consider = deserialization.consider_zone
    ](const nlohmann::json& msg) -> DeserializedGTZEvent
    {
      std::vector<std::string> errors;

      if (!(*consider))
      {
        errors.push_back("Not accepting zone requests");
        return {nullptr, errors};
      }

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      errors.insert(
        errors.end(), confirm.errors().begin(), confirm.errors().end());
      if (!confirm.is_accepted())
        return {nullptr, errors};

      // 1. Parse zone_name (required)
      auto valid_zone = zone_deser(msg.at("zone_name"));
      errors.insert(
        errors.end(), valid_zone.errors.begin(), valid_zone.errors.end());
      if (!valid_zone.description.has_value())
        return {nullptr, errors};

      // 2. Parse modifiers (optional)
      std::optional<GoToZone::Description::Modifiers> modifiers;
      if (const auto mod_it = msg.find("modifiers"); mod_it != msg.end())
      {
        GoToZone::Description::Modifiers m;
        // waypoint_preference
        if (const auto wp_it = mod_it->find("waypoint_preference");
            wp_it != mod_it->end())
        {
          if (wp_it->contains("group"))
            m.group_hint = wp_it->at("group").get<std::string>();
          if (wp_it->contains("orientation"))
            m.orientation_hint = wp_it->at("orientation").get<double>();
          if (const auto pw_it = wp_it->find("preferred_waypoints");
              pw_it != wp_it->end())
          {
            bool any_failure = false;
            for (const auto& wp : *pw_it)
            {
              auto vp = zone_wp_deser(*valid_zone.description, wp);
              errors.insert(
                errors.end(), vp.errors.begin(), vp.errors.end());
              if (!vp.description.has_value())
              {
                any_failure = true;
                continue;
              }
              m.preferred_waypoints.push_back(*vp.description);
            }
            if (any_failure)
              return {nullptr, errors};
          }
        }
        // extend with more modifiers here in future
        modifiers = std::move(m);
      }

      // 3. Create description
      auto desc = GoToZone::Description::make(
        *valid_zone.description, std::move(modifiers));
      return {desc, std::move(errors)};
    };

  // zone task
  auto deserialize_zone_task =
    [
      desc = deserialize_zone
    ](const nlohmann::json& msg) -> DeserializedTask
    {
      auto zone_task = desc(msg);

      if (zone_task.description == nullptr)
        return {nullptr, zone_task.errors};

      rmf_task_sequence::Task::Builder builder;
      builder.add_phase(Phase::Description::make(zone_task.description), {});
      return {builder.build("Zone", ""), std::move(zone_task.errors)};
    };
  deserialization.task->add("zone", validate_zone_task, deserialize_zone_task);

  // zone event
  auto deserialize_zone_event =
    [
      desc = deserialize_zone
    ](const nlohmann::json& msg) -> DeserializedEvent
    {
      auto zone_event = desc(msg);

      if (zone_event.description == nullptr)
        return {nullptr, zone_event.errors};

      return {zone_event.description, std::move(zone_event.errors)};
    };
  deserialization.event->add(
    "go_to_zone", validate_zone_event, deserialize_zone_event);
}

} // namespace tasks
} // namespace rmf_fleet_adapter
