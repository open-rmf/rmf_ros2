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

#include "../phases/DispenseItem.hpp"
#include "../phases/IngestItem.hpp"
#include "../phases/GoToPlace.hpp"

#include "../events/LegacyPhaseShim.hpp"
#include "../events/Error.hpp"
#include "../events/GoToPlace.hpp"

#include "Delivery.hpp"

#include <rmf_ingestor_msgs/msg/ingestor_request_item.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request_item.hpp>

#include <rmf_task_sequence/events/PickUp.hpp>
#include <rmf_task_sequence/events/DropOff.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>
#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>

#include <rmf_fleet_adapter/schemas/event_description_PayloadTransfer.hpp>
#include <rmf_fleet_adapter/schemas/event_description_PickUp.hpp>
#include <rmf_fleet_adapter/schemas/event_description_DropOff.hpp>
#include <rmf_fleet_adapter/schemas/task_description_Delivery.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
std::shared_ptr<LegacyTask> make_delivery(
  const rmf_task::ConstRequestPtr request,
  const agv::RobotContextPtr& context,
  const rmf_traffic::agv::Plan::Start pickup_start,
  const rmf_traffic::Time deployment_time,
  const rmf_task::State finish_state,
  const rmf_task_msgs::msg::Delivery delivery_profile)
{

  std::shared_ptr<const rmf_task::requests::Delivery::Description> description =
    std::dynamic_pointer_cast<
    const rmf_task::requests::Delivery::Description>(request->description());

  if (description == nullptr)
    return nullptr;

  const auto pickup_waypoint = description->pickup_waypoint();
  const auto dropoff_waypoint = description->dropoff_waypoint();

  LegacyTask::PendingPhases phases;
  phases.push_back(
    phases::GoToPlace::make(
      context,
      std::move(pickup_start),
      pickup_waypoint));

  phases.push_back(
    std::make_unique<phases::DispenseItem::PendingPhase>(
      context,
      request->booking()->id(),
      delivery_profile.pickup_dispenser,
      context->itinerary().description().owner(),
      delivery_profile.items));

  const auto dropoff_start = [&]() -> rmf_traffic::agv::Planner::Start
    {

      if (pickup_start.waypoint() == pickup_waypoint)
        return pickup_start;

      rmf_traffic::agv::Planner::Goal goal{pickup_waypoint};

      const auto result = context->planner()->plan(pickup_start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double orientation = trajectory.back().position()[2];

      return rmf_traffic::agv::Planner::Start{
      finish_time,
      pickup_waypoint,
      orientation};
    } ();

  phases.push_back(
    phases::GoToPlace::make(
      context,
      std::move(dropoff_start),
      dropoff_waypoint));

  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> ingestor_items;
  ingestor_items.reserve(delivery_profile.items.size());
  for (const auto& dispenser_item : delivery_profile.items)
  {
    rmf_ingestor_msgs::msg::IngestorRequestItem item{};
    item.type_guid = dispenser_item.type_guid;
    item.quantity = dispenser_item.quantity;
    item.compartment_name = dispenser_item.compartment_name;
    ingestor_items.push_back(std::move(item));
  }

  phases.push_back(
    std::make_unique<phases::IngestItem::PendingPhase>(
      context,
      request->booking()->id(),
      delivery_profile.dropoff_ingestor,
      context->itinerary().description().owner(),
      ingestor_items));

  return LegacyTask::make(
    request->booking()->id(),
    std::move(phases),
    context->worker(),
    deployment_time,
    finish_state,
    request);
}

//==============================================================================
struct TransferItems : public rmf_task_sequence::events::Placeholder::Description
{
  enum class Dir
  {
    Load,
    Unload
  };

  TransferItems(const rmf_task_sequence::events::PickUp::Description& pickup)
  : rmf_task_sequence::events::Placeholder::Description("Load items", ""),
    direction(Dir::Load),
    target(pickup.from_dispenser()),
    payload(pickup.payload())
  {
    // Do nothing
  }

  TransferItems(const rmf_task_sequence::events::DropOff::Description& dropoff)
  : rmf_task_sequence::events::Placeholder::Description("Unload items", ""),
    direction(Dir::Unload),
    target(dropoff.into_ingestor()),
    payload(dropoff.payload())
  {
    // Do nothing
  }

  template<template<class> class Build, typename T>
  std::vector<T> collect_items() const
  {
    std::vector<T> items;
    for (const auto& c : payload.components())
    {
      items.push_back(
        Build<T>()
        .type_guid(c.sku())
        .quantity(c.quantity())
        .compartment_name(c.compartment()));
    }
  }

  Dir direction;
  std::string target;
  rmf_task::Payload payload;

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr&,
    const TransferItems& description,
    std::function<void()> update)
  {
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;
    const auto* task_id = context->current_task_id();

    std::string name;
    if (description.direction == Dir::Unload)
      name = "Unload";
    else
      name = "Load";

    if (description.payload.components().size() == 1)
      name += "item";
    else
      name += "items";

    if (!task_id)
    {
      const auto error_state = rmf_task::events::SimpleEventState::make(
        id->assign(), name, "", rmf_task::Event::Status::Error, {},
        context->clock());

      error_state->update_log().error(
        "Task ID is null while trying to perform a delivery. This indicates a "
        "serious software error. Please try to reset the task, and contact the "
        "system integrator to inform them of this issue.");

      return events::Error::Standby::make(std::move(error_state));
    }

    std::shared_ptr<LegacyTask::PendingPhase> legacy;
    if (description.direction == Dir::Unload)
    {
      using IngestorItem = rmf_ingestor_msgs::msg::IngestorRequestItem;
      std::vector<IngestorItem> items;
      for (const auto& c : description.payload.components())
      {
        items.push_back(
          rmf_ingestor_msgs::build<IngestorItem>()
          .type_guid(c.sku())
          .quantity(c.quantity())
          .compartment_name(c.compartment()));
      }

      legacy = std::make_shared<phases::IngestItem::PendingPhase>(
        context, *task_id, description.target,
        context->itinerary().description().owner(),
        std::move(items));
    }
    else
    {
      using DispenserItem = rmf_dispenser_msgs::msg::DispenserRequestItem;
      std::vector<DispenserItem> items;
      for (const auto& c : description.payload.components())
      {
        items.push_back(
          rmf_dispenser_msgs::build<DispenserItem>()
          .type_guid(c.sku())
          .quantity(c.quantity())
          .compartment_name(c.compartment()));
      }

      legacy = std::make_shared<phases::DispenseItem::PendingPhase>(
        context, *task_id, description.target,
        context->itinerary().description().owner(),
        std::move(items));
    }

    return events::LegacyPhaseShim::Standby::make(
      legacy, context->worker(), context->clock(), id, std::move(update), name);
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<TransferItems>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const TransferItems& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const TransferItems& description,
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
using DeserializedItemTransfer = agv::FleetUpdateHandle::DeserializedEvent;
template<typename T>
std::function<DeserializedItemTransfer(const nlohmann::json& msg)>
make_deserializer(
  const agv::FleetUpdateHandle::PlaceDeserializer& place_deser,
  const std::shared_ptr<agv::FleetUpdateHandle::ConsiderRequest>& consider)
{
  auto parse_payload_component = [](const nlohmann::json& msg)
    -> rmf_task::Payload::Component
    {
      std::string compartment = "";

      if (const auto& compartment_json = msg["compartment"])
        compartment = compartment_json.get<std::string>();

      return rmf_task::Payload::Component(
        msg["sku"].get<std::string>(),
        msg["quantity"].get<uint64_t>(),
        std::move(compartment));
    };

  return
    [
      place_deser,
      consider,
      parse_payload_component = std::move(parse_payload_component)
    ](const nlohmann::json& msg)
      -> agv::FleetUpdateHandle::DeserializedEvent
    {
      if (!consider || !(*consider))
        return {nullptr, {"Not accepting delivery requests"}};

      auto place = place_deser(msg["place"]);
      if (!place.description.has_value())
      {
        return {nullptr, std::move(place.errors)};
      }

      std::vector<rmf_task::Payload::Component> payload_components;
      const auto& payload_json = msg["payload"];
      if (payload_json.is_object())
      {
        payload_components.push_back(parse_payload_component(payload_json));
      }
      else if (payload_json.is_array())
      {
        for (const auto& component_json : payload_json)
        {
          payload_components.push_back(parse_payload_component(component_json));
        }
      }
      else
      {
        return {
          nullptr,
          {"invalid data type provided for 'payload' property: expected an "
           "object or an array but got " + std::string(payload_json.type_name())
           + " type instead"}
        };
      }

      std::string handler;
      if (const auto& handler_json = msg["handler"])
      {
        handler = handler_json.get<std::string>();
      }

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      if (!confirm.is_accepted())
        return {nullptr, confirm.errors()};

      // TODO(MXG): Add a way for system integrators to specify a duration
      // estimate for the payload transfer
      return {
        T::make(
          std::move(*place.description),
          std::move(handler),
          rmf_task::Payload(std::move(payload_components)),
          rmf_traffic::Duration(0)),
        confirm.errors()
      };
    };
}

//==============================================================================
void add_delivery(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock)
{
  using Bundle = rmf_task_sequence::events::Bundle;
  using PickUp = rmf_task_sequence::events::PickUp;
  using DropOff = rmf_task_sequence::events::DropOff;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using DeliveryDescription = rmf_task::requests::Delivery::Description;

  auto validate_payload_transfer =
    deserialization.make_validator_shared(
      schemas::event_description_PayloadTransfer);
  deserialization.add_schema(schemas::event_description_PayloadTransfer);

  deserialization.consider_pickup =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();
  auto deserialize_pickup =
    make_deserializer<PickUp::Description>(
      deserialization.place, deserialization.consider_pickup);
  deserialization.event.add(
    "pickup", validate_payload_transfer, deserialize_pickup);

  deserialization.consider_dropoff =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();
  auto deserialize_dropoff =
    make_deserializer<DropOff::Description>(
      deserialization.place, deserialization.consider_dropoff);
  deserialization.event.add(
    "dropoff", validate_payload_transfer, deserialize_dropoff);

  auto validate_delivery =
    deserialization.make_validator_shared(schemas::task_description_Delivery);
  deserialization.add_schema(schemas::task_description_Delivery);
  deserialization.add_schema(schemas::event_description_PickUp);
  deserialization.add_schema(schemas::event_description_DropOff);

  auto deserialize_delivery =
    [deserialize_pickup, deserialize_dropoff](
      const nlohmann::json& msg) -> agv::FleetUpdateHandle::DeserializedTask
    {
      const auto pickup = deserialize_pickup(msg["pickup"]);
      const auto dropoff = deserialize_dropoff(msg["dropoff"]);
      std::vector<std::string> errors;
      errors.reserve(pickup.errors.size() + dropoff.errors.size());
      errors.insert(
        errors.end(), pickup.errors.begin(), pickup.errors.end());
      errors.insert(
        errors.end(), dropoff.errors.begin(), dropoff.errors.end());

      if (!pickup.description || !dropoff.description)
      {
        return {nullptr, std::move(errors)};
      }

      rmf_task_sequence::Task::Builder builder;
      builder.add_phase(Phase::Description::make(pickup.description), {});
      builder.add_phase(Phase::Description::make(dropoff.description), {});
      // TODO(MXG): Consider making the category and detail more detailed
      return {builder.build("Delivery", ""), std::move(errors)};
    };

  deserialization.task.add("delivery", validate_delivery, deserialize_delivery);

  auto private_initializer =
    std::make_shared<rmf_task_sequence::Event::Initializer>();

  TransferItems::add(*private_initializer);
  events::GoToPlace::add(*private_initializer);

  auto pickup_unfolder =
    [](const PickUp::Description& pickup)
    {
      return Bundle::Description({
        events::GoToPlace::Description::make(pickup.pickup_location()),
        std::make_shared<TransferItems>(pickup)
      }, Bundle::Type::Sequence, "Pick up");
    };

  Bundle::unfold<PickUp::Description>(
    std::move(pickup_unfolder), *activation.event, private_initializer);

  auto dropoff_unfolder =
    [](const DropOff::Description& dropoff)
    {
      return Bundle::Description({
        events::GoToPlace::Description::make(dropoff.drop_off_location()),
        std::make_shared<TransferItems>(dropoff)
      }, Bundle::Type::Sequence, "Drop Off");
    };

  Bundle::unfold<DropOff::Description>(
    std::move(dropoff_unfolder), *activation.event, private_initializer);

  auto delivery_unfolder =
    [](const DeliveryDescription& delivery)
    {
      rmf_task_sequence::Task::Builder builder;
      builder
        .add_phase(
          Phase::Description::make(
            PickUp::Description::make(
              delivery.pickup_waypoint(),
              delivery.pickup_from_dispenser(),
              delivery.payload(),
              delivery.pickup_wait())), {})
        .add_phase(
          Phase::Description::make(
            DropOff::Description::make(
              delivery.dropoff_waypoint(),
              delivery.dropoff_to_ingestor(),
              delivery.payload(),
              delivery.dropoff_wait())), {});

      // TODO(MXG): Consider making the category and detail more detailed
      return *builder.build("Delivery", "");
    };

  rmf_task_sequence::Task::unfold<rmf_task::requests::Delivery::Description>(
    std::move(delivery_unfolder), *activation.task,
    activation.phase, std::move(clock));
}

} // namespace task
} // namespace rmf_fleet_adapter
