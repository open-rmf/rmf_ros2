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
      zone_group_deser = deserialization.zone_group,
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

      // 2. Parse hints (optional), most preferred first
      std::vector<GoToZone::Description::Hint> hints;
      if (const auto hints_it = msg.find("hints"); hints_it != msg.end())
      {
        for (const auto& h : *hints_it)
        {
          GoToZone::Description::Hint hint;
          if (h.contains("group"))
          {
            auto vg = zone_group_deser(*valid_zone.description, h.at("group"));
            errors.insert(errors.end(), vg.errors.begin(), vg.errors.end());
            if (!vg.description.has_value())
              return {nullptr, errors};
            hint.set_group(*vg.description);
          }

          if (h.contains("waypoint"))
          {
            auto vp = zone_wp_deser(*valid_zone.description, h.at("waypoint"));
            errors.insert(errors.end(), vp.errors.begin(), vp.errors.end());
            if (!vp.description.has_value())
              return {nullptr, errors};
            hint.set_waypoint(*vp.description);
          }

          if (h.contains("orientations"))
          {
            hint.set_orientations(
              h.at("orientations").get<std::vector<double>>());
          }

          hints.push_back(std::move(hint));
        }
      }

      // 3. Create description
      auto desc = GoToZone::Description::make(
        *valid_zone.description, std::move(hints));
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
