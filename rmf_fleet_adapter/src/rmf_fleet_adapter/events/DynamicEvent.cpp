/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "DynamicEvent.hpp"
#include "../log_to_json.hpp"

#include <rmf_task_sequence/events/Placeholder.hpp>
#include <rmf_task_msgs/msg/log_entry.hpp>

#include <rmf_fleet_adapter/schemas/event_description__dynamic_event.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
void DynamicEvent::add(
  agv::TaskDeserialization& deserialization,
  rmf_task_sequence::Event::InitializerPtr event_initializer)
{
  const auto event_deserializer = deserialization.event;
  auto validate_dynamic_event = deserialization.make_validator_shared(
    schemas::event_description__dynamic_event);

  auto deserialize_dynamic_event =
    [event_deserializer, event_initializer](
      const nlohmann::json& msg) -> DeserializedEvent
    {
      std::vector<std::string> errors;
      const auto estimate_it = msg.find("estimate");
      rmf_task_sequence::Event::ConstDescriptionPtr estimate;
      if (estimate_it != msg.end())
      {
        const auto& estimate_category =
          estimate_it->at("category").get<std::string>();
        const auto& estimate_description = estimate_it->at("description");
        const auto e_it = event_deserializer->handlers.find(estimate_category);
        if (e_it == event_deserializer->handlers.end())
        {
          errors.push_back(
            "No support for [" + estimate_category + "] activities");
          return {nullptr, std::move(errors)};
        }

        auto event = e_it->second.deserializer(estimate_description);
        errors.insert(errors.end(), event.errors.begin(), event.errors.end());
        if (!event.description)
        {
          return {nullptr, std::move(errors)};
        }

        estimate = event.description;
      }

      if (!estimate)
      {
        // If the estimate was left blank, just use a placeholder estimate.
        estimate = std::make_shared<rmf_task_sequence::events::Placeholder::Description>("", "");
      }

      const auto required_it = msg.find("required");
      if (required_it != msg.end())
      {
        bool any_error = false;
        for (const auto& event : *required_it)
        {
          const auto& req_category = event.at("category").get<std::string>();
          const auto& req_description = event.at("description");
          const auto r_it = event_deserializer->handlers.find(req_category);
          if (r_it == event_deserializer->handlers.end())
          {
            errors.push_back("No support for [" + req_category + "] activities");
            any_error = true;
          }
          else
          {
            auto event = r_it->second.deserializer(req_description);
            errors.insert(errors.end(), event.errors.begin(), event.errors.end());
            if (!event.description)
            {
              any_error = true;
            }
          }
        }

        if (any_error)
        {
          return {nullptr, std::move(errors)};
        }
      }

      std::string category = "dynamic_event";
      const auto category_it = msg.find("category");
      if (category_it != msg.end())
      {
        category = category_it->get<std::string>();
      }

      std::string detail;
      const auto detail_it = msg.find("detail");
      if (detail_it != msg.end())
      {
        detail = detail_it->get<std::string>();
      }

      const auto description = DynamicEvent::Description::make(
        category, detail, msg, std::move(estimate), event_deserializer, event_initializer);
      return {description, std::move(errors)};
    };

  deserialization.event->add(
    "dynamic_event",
    validate_dynamic_event,
    deserialize_dynamic_event);

  event_initializer->add<Description>(
    [event_deserializer, event_initializer](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update) -> StandbyPtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [=](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> ActivePtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });
}

//==============================================================================
const std::string& DynamicEvent::Description::json_text() const
{
  return _json_text;
}

//==============================================================================
const nlohmann::json& DynamicEvent::Description::json() const
{
  return _json;
}

//==============================================================================
rmf_task_sequence::Activity::ConstModelPtr
DynamicEvent::Description::make_model(
  rmf_task::State initial_state,
  const rmf_task::Parameters& parameters) const
{
  return _estimate->make_model(initial_state, parameters);
}

//==============================================================================
rmf_task::Header DynamicEvent::Description::generate_header(
  const rmf_task::State& initial_state,
  const rmf_task::Parameters& parameters) const
{
  const auto estimate = _estimate->generate_header(initial_state, parameters);
  return rmf_task::Header(
    _category, _detail, estimate.original_duration_estimate());
}

//==============================================================================
const DeserializeJSONPtr<DeserializedEvent>&
DynamicEvent::Description::event_deserializer() const
{
  return _event_deserializer;
}

//==============================================================================
const rmf_task_sequence::Event::ConstInitializerPtr&
DynamicEvent::Description::event_initializer() const
{
  return _event_initializer;
}

//==============================================================================
DynamicEvent::Description::Description(
  std::string category,
  std::string detail,
  nlohmann::json description_json,
  rmf_task_sequence::Event::ConstDescriptionPtr estimate,
  DeserializeJSONPtr<DeserializedEvent> event_deserializer,
  rmf_task_sequence::Event::ConstInitializerPtr event_initializer)
: _json_text(description_json.dump()),
  _json(std::move(description_json)),
  _category(std::move(category)),
  _detail(std::move(detail)),
  _estimate(std::move(estimate)),
  _event_deserializer(std::move(event_deserializer)),
  _event_initializer(std::move(event_initializer))
{
  // Do nothing
}

//==============================================================================
auto DynamicEvent::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const Description& description,
  std::function<void()> update) -> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>(Standby{description});
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

//==============================================================================
auto DynamicEvent::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration DynamicEvent::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto DynamicEvent::Standby::begin(
  std::function<void()> /*checkpoint*/,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = DynamicEvent::Active::make(
      _assign_id,
      _context,
      _description,
      _state,
      _update,
      finished);
  }

  return _active;
}

//==============================================================================
DynamicEvent::Standby::Standby(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
auto DynamicEvent::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  Description description,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>(Active{description});

  active->_assign_ids = id;
  active->_context = context;
  active->_state = state;
  active->_update = update;
  active->_finished = finished;

  auto executor = [w = active->weak_from_this()](
    std::shared_ptr<DynamicEventHandle> handle,
    std::shared_ptr<void> event_raii)
  {
    const auto me = w.lock();
    if (!me)
    {
      handle->abort(dynamic_event_execution_failure("shutting down"));
      return;
    }

    if (DynamicEventAction::Goal::EVENT_TYPE_FINISHED == handle->get_goal()->event_type)
    {
      // The dynamic event is completely finished now
      me->_state->update_status(rmf_task::Event::State::Status::Completed);
      me->_publish_update();
      me->_finished();

      const auto result = std::make_shared<DynamicEventAction::Result>(
        rmf_task_msgs::build<DynamicEventAction::Result>()
          .execution_failure("")
          .status(status_to_string(me->_state->status()))
          .id(me->_state->id())
      );
      handle->succeed(result);
      return;
    }

    if (DynamicEventAction::Goal::EVENT_TYPE_CANCEL == handle->get_goal()->event_type)
    {
      if (!me->_current_event)
      {
        handle->abort(dynamic_event_execution_failure(
          "an idle dynamic event cannot be cancelled"));
        return;
      }

      const auto id = me->_current_event->state()->id();
      if (id != handle->get_goal()->id)
      {
        handle->abort(dynamic_event_execution_failure(
          "ID for cancel request does not match current event"));
        return;
      }

      me->_event_cancellation_handles.push_back(handle);
      me->_context->worker().schedule([w = me->weak_from_this()](const auto&)
        {
          if (const auto me = w.lock())
            me->_current_event->cancel();
        });
      return;
    }

    if (DynamicEventAction::Goal::EVENT_TYPE_NEXT != handle->get_goal()->event_type)
    {
      handle->abort(dynamic_event_execution_failure("invalid event_type"));
      return;
    }

    const std::string& child_category = handle->get_goal()->category;

    const auto child_description_json =
      nlohmann::json::parse(handle->get_goal()->description);

    // We can use .at here because we already validated that we can run the
    // description. Unfortunately because of the way the rclcpp_action API is
    // designed, we need to parse this twice.
    const auto deserializer = me->_description.event_deserializer()->handlers
      .at(child_category);

    const auto deser = deserializer.deserializer(child_description_json);
    const auto child_description = deser.description;

    if (!child_description)
    {
      std::string failure_msg = "Description parsing failure";
      for (const auto& error : deser.errors)
      {
        failure_msg += "\n -- " + error;
      }

      handle->abort(dynamic_event_execution_failure(failure_msg));
      return;
    }

    const auto stubborn_period = handle->get_goal()->stubborn_period;
    me->_deferred_event_start = nullptr;
    me->_current_event_raii = std::move(event_raii);
    me->_current_handle = std::move(handle);
    if (me->_interrupted)
    {
      me->_deferred_event_start = [
        w = me->weak_from_this(),
        child_description,
        stubborn_period
      ]()
        {
          const auto me = w.lock();
          if (!me)
            return;

          me->_begin_next_event(child_description, stubborn_period);
        };

      return;
    }

    me->_begin_next_event(child_description, stubborn_period);
  };

  auto cancel = [w = active->weak_from_this()](
    std::shared_ptr<DynamicEventHandle>)
  {
    const auto me = w.lock();
    if (!me || !me->_current_event)
      return;

    me->_current_event->cancel();
    // NOTE(@mxgrey): We do not add this handle to _cancellation_handles because
    // this is a cancellation request from the main driving action, not an
    // external cancellation request.
  };

  std::string logger_name =
    std::string("rmf.dynamic_event.")
    + active->_context->group() + "."
    + active->_context->name();
  auto logger = rclcpp::get_logger(logger_name);
  auto validator = [w = active->weak_from_this(), logger](
    const std::string& category,
    const std::string& description)
  {
    const auto me = w.lock();
    if (!me)
    {
      RCLCPP_ERROR(
        logger,
        "Cannot validate dynamic event goal because the dynamic event has ended.");
      return false;
    }

    const auto& handlers = me->_description.event_deserializer()->handlers;

    const auto deserialize_it = handlers.find(category);
    if (deserialize_it == handlers.end())
    {
      std::string supported;
      for (const auto& [category, _] : handlers)
      {
        supported += "[" + category + "] ";
      }

      RCLCPP_ERROR(
        logger,
        "Dynamic event goal is invalid because the category [%s] is not "
        "supported by the robot. Supported categories include %s",
        category.c_str(),
        supported.c_str());
      return false;
    }

    nlohmann::json description_json;
    try
    {
      description_json = nlohmann::json::parse(description);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        logger,
        "Dynamic event goal is invalid because its description could not be "
        "parsed as json: %s",
        e.what());
      return false;
    }

    try
    {
      deserialize_it->second.validator->validate(description_json);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        logger,
        "Dynamic event goal is invalid because its description is not "
        "compatible with the category [%s]: %s",
        category.c_str(),
        e.what());
      return false;
    }

    const auto deser = deserialize_it->second.deserializer(description_json);
    if (deser.description == nullptr)
    {
      std::string errors;
      for (const auto& error : deser.errors)
      {
        errors += "\n -- " + error;
      }
      RCLCPP_ERROR(
        logger,
        "Dynamic event goal is invalid because its description is not "
        "compatible with the robot:%s",
        errors.c_str());
      return false;
    }

    return true;
  };

  active->_callbacks = std::make_shared<DynamicEventCallbacks>(
    DynamicEventCallbacks {
      executor,
      cancel,
      validator,
      active->_context
    });

  active->_seq = active->_context->_begin_dynamic_event(
    active->_description.json_text(),
    active->_callbacks);

  active->_state->update_status(rmf_task::Event::State::Status::Standby);

  active->_publish_update();
  active->_update();

  active->_context->_publish_dynamic_event_status(
    rmf_task::Event::State::Status::Standby,
    active->_state->id()
  );

  return active;
}

//==============================================================================
auto DynamicEvent::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration DynamicEvent::Active::remaining_time_estimate() const
{
  if (_current_event)
    return _current_event->remaining_time_estimate();

  return rmf_traffic::Duration(0);
}

//==============================================================================
auto DynamicEvent::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto DynamicEvent::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _interrupted = true;
  if (_current_event)
    return _current_event->interrupt(task_is_interrupted);

  return Resume::make(
    [w = weak_from_this()]()
    {
      const auto me = w.lock();
      if (!me)
        return;

      me->_interrupted = false;
      if (me->_deferred_event_start)
      {
        me->_context->worker().schedule(
          [job = me->_deferred_event_start](const auto&)
          {
            job();
          });

        me->_deferred_event_start = nullptr;
      }
    });
}

//==============================================================================
void DynamicEvent::Active::cancel()
{
  _cancelled = true;
  _state->update_status(Status::Canceled);
  _state->update_log().info("Received signal to cancel");
  if (_current_event)
  {
    _current_event->cancel();
    return;
  }

  _context->worker().schedule([finished = _finished](const auto&)
    {
      finished();
    });
}

//==============================================================================
void DynamicEvent::Active::kill()
{
  _cancelled = true;
  _state->update_status(Status::Killed);
  _state->update_log().info("Received signal to kill");
  if (_current_event)
  {
    _current_event->kill();
    return;
  }

  _context->worker().schedule([finished = _finished](const auto&)
    {
      finished();
    });
}

//==============================================================================
DynamicEvent::Active::Active(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
void DynamicEvent::Active::_publish_update()
{
  _update();

  if (!_current_event || !_current_handle)
    return;

  using LogEntry = rmf_task_msgs::msg::LogEntry;

  const auto& state = _current_event->state();
  std::vector<LogEntry> logs;
  for (auto log : _log_reader.read(state->log()))
  {
    logs.push_back(
      rmf_task_msgs::build<LogEntry>()
        .seq(_seq)
        .tier(tier_to_string(log.tier()))
        .unix_millis_time(to_millis(log.time().time_since_epoch()).count())
        .text(log.text()));
  }

  auto msg = rmf_task_msgs::build<DynamicEventAction::Feedback>()
    .status(status_to_string(state->status()))
    .id(state->id())
    .logs(std::move(logs));

  _current_handle->publish_feedback(
    std::make_shared<DynamicEventAction::Feedback>(std::move(msg)));

  _context->_publish_dynamic_event_status(state->status(), state->id());
}

//==============================================================================
bool early_termination_status(rmf_task::Event::Status status)
{
  return status == rmf_task::Event::Status::Canceled
    || status == rmf_task::Event::Status::Killed
    || status == rmf_task::Event::Status::Skipped;
}

//==============================================================================
rmf_traffic::PlanId set_to_holding_itinerary(
  const std::string& map,
  Eigen::Vector3d position,
  rmf_traffic::Time start_time,
  double period,
  rmf_traffic::schedule::Participant& participant)
{
  const auto plan_id = participant.assign_plan_id();
  const auto finish_time = start_time + rmf_traffic::time::from_seconds(period);

  rmf_traffic::Trajectory trajectory;
  trajectory.insert(start_time, position, Eigen::Vector3d::Zero());
  trajectory.insert(finish_time, position, Eigen::Vector3d::Zero());
  rmf_traffic::schedule::Itinerary itinerary;
  itinerary.push_back(rmf_traffic::Route(map, trajectory));

  participant.set(plan_id, itinerary);
  return plan_id;
}

//==============================================================================
void DynamicEvent::Active::_begin_next_event(
  std::shared_ptr<const rmf_task_sequence::Event::Description> child_description,
  const float stubborn_period)
{
  auto on_update = [w = this->weak_from_this()]()
  {
    const auto me = w.lock();
    if (!me)
      return;

    const auto current_status = me->_state->status();
    const bool need_status_update = !early_termination_status(current_status);

    if (need_status_update)
    {
      if (me->_current_event)
      {
        const auto child_status = me->_current_event->state()->status();
        if (!early_termination_status(child_status))
        {
          me->_state->update_status(child_status);
        }
      }
      else
      {
        me->_state->update_status(rmf_task::Event::Status::Standby);
      }
    }

    me->_publish_update();
  };

  auto on_finish = [w = this->weak_from_this(), handle = _current_handle, stubborn_period]()
  {
    const auto me = w.lock();
    if (!me || !me->_current_event)
    {
      if (handle)
        handle->abort(dynamic_event_execution_failure("shutting down"));

      return;
    }

    if (!me->_cancelled)
    {
      me->_state->update_status(rmf_task::Event::Status::Standby);
    }
    me->_publish_update();

    const auto state = me->_current_event->state();
    const auto result = std::make_shared<DynamicEventAction::Result>(
      rmf_task_msgs::build<DynamicEventAction::Result>()
        .execution_failure("")
        .status(status_to_string(state->status()))
        .id(state->id())
    );

    if (early_termination_status(state->status()))
    {
      if (handle->is_canceling())
      {
        handle->canceled(result);
      }
      else
      {
        handle->abort(result);
      }
    }
    else
    {
      handle->succeed(result);
    }

    for (const auto& handle : me->_event_cancellation_handles)
    {
      // Whether the event terminated early or not, we tell the cancellation
      // requests that the cancellation has succeeded, because the most
      // important thing for those action clients is to know that the event is
      // no longer running. We would only return a canceled result to them if
      // it isn't possible to bring the action to an end, e.g. the action they
      // requested is unknown.
      handle->succeed(result);
    }

    me->_current_event = nullptr;
    me->_current_event_raii = nullptr;
    me->_current_handle = nullptr;
    me->_deferred_event_start = nullptr;
    me->_event_cancellation_handles.clear();

    if (me->_cancelled)
    {
      me->_finished();
    }
    else if (stubborn_period > 0.0)
    {
      float period = stubborn_period;
      if (period < 2.0)
      {
        period = 2.0;
      }

      const auto stubbornness = me->_context->be_stubborn();

      const auto position = std::make_shared<Eigen::Vector3d>(me->_context->position());
      const auto map = std::make_shared<std::string>(me->_context->map());
      const auto period_start = std::make_shared<rmf_traffic::Time>(me->_context->now());
      const auto plan_id = std::make_shared<rmf_traffic::PlanId>(
        set_to_holding_itinerary(
          *map, *position, *period_start, period, me->_context->itinerary()));

      me->_stubborn_timer = me->_context->node()->try_create_wall_timer(
        rmf_traffic::time::from_seconds(1.0),
        [plan_id, map, stubbornness, period_start, position, period, w = me->weak_from_this()]()
        {
          const auto me = w.lock();
          if (!me)
            return;

          const auto now = me->_context->now();
          const auto current_position = me->_context->position();
          const auto& current_map = me->_context->map();
          const double threshold = std::max(
            me->_context->nav_params()->max_merge_waypoint_distance,
            1e-2);

          const double dist = (current_position - *position).norm();

          if (threshold < dist || *map != current_map)
          {
            *position = current_position;
            *map = current_map;
            *period_start = now;

            // The robot has moved substantially from its last position, so
            // let's update the location in the schedule
            RCLCPP_INFO(
              me->_context->node()->get_logger(),
              "Robot [%s] has moved a distance of %.2fm while idle in a "
              "dynamic event so we are adjusting its stubbornness location.",
              me->_context->requester_id().c_str(),
              dist);

            *plan_id = set_to_holding_itinerary(
              *map, *position, *period_start, period, me->_context->itinerary());
          }
          else
          {
            const auto delay = now - *period_start;
            me->_context->itinerary().cumulative_delay(*plan_id, delay);
          }

        });
    }
  };

  // End the stubborn behavior if it was active before
  _stubborn_timer = nullptr;

  // We don't use checkpoints for dynamic events
  auto on_checkpoint = []() {};

  _current_event = _description.event_initializer()->initialize(
    _assign_ids,
    _context->make_get_state(),
    _context->task_parameters(),
    *child_description,
    std::move(on_update))
    ->begin(std::move(on_checkpoint), std::move(on_finish));

  _state->add_dependency(_current_event->state());
  _state->update_status(rmf_task::Event::Status::Underway);
  _publish_update();
}

} // namespace events
} // namespace rmf_fleet_adapter
