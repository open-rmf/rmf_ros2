#include "GoToZone.hpp"
#include "GoToPlace.hpp"

#include "../phases/Utils.hpp"

#include <rmf_task_sequence/events/GoToPlace.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_zone_msgs/msg/zone_request.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <nlohmann/json.hpp>

#include <utility>
#include <vector>

namespace rmf_fleet_adapter {
namespace events {

// How often we re-send the entry request while waiting for a waypoint.
const auto WAITING_WARN_INTERVAL = std::chrono::seconds(10);

// How long before the warning points at the zone manager.
const auto SUSPECT_MANAGER_AFTER = std::chrono::seconds(45);

// How often we actually say we are still waiting.
const auto WAITING_LOG_INTERVAL = std::chrono::seconds(60);

namespace {

//==============================================================================
rmf_zone_msgs::msg::ZoneModifiers zone_modifiers(
  const GoToZone::Description& description)
{
  rmf_zone_msgs::msg::ZoneModifiers modifiers;
  if (description.modifiers().has_value())
  {
    const auto& m = *description.modifiers();
    modifiers.group_hint = m.group_hint;
    modifiers.has_orientation_hint = m.orientation_hint.has_value();
    modifiers.orientation_hint = m.orientation_hint.value_or(0.0);
    modifiers.preferred_waypoints = m.preferred_waypoints;
  }
  return modifiers;
}
} // anonymous namespace

//==============================================================================
void GoToZone::add(rmf_task_sequence::Event::Initializer& initializer)
{
  initializer.add<Description>(
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update) -> StandbyPtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [](
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
auto GoToZone::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const Description& description,
  std::function<void()> update)
-> std::shared_ptr<Standby>
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

  // With reservations off there is no ticket to hand over, so the zone
  // assignment is advisory only. Warn every task rather than reject.
  if (!context->_parking_spot_manager_enabled())
  {
    RCLCPP_WARN(
      context->node()->get_logger(),
      "GoToZone for [%s] in zone [%s], but use_parking_reservations is off "
      "for this fleet. Another robot could take the assigned waypoint.",
      context->requester_id().c_str(),
      description.zone_name().c_str());

    standby->_state->update_log().warn(
      "parking-spot reservations are off, so another robot could take the "
      "assigned waypoint");
  }

  return standby;
}

//==============================================================================
GoToZone::Standby::Standby(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
auto GoToZone::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToZone::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto GoToZone::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (_active)
    return _active;

  _active = Active::make(
    _assign_id,
    _context,
    _description,
    _state,
    _update,
    std::move(finished));

  return _active;
}

//==============================================================================
auto GoToZone::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  Description description,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::shared_ptr<Active>(new Active(std::move(description)));
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_state = std::move(state);
  active->_update = std::move(update);
  active->_finished = std::move(finished);

  active->_state->update_status(Status::Underway);

  // _preference is the only thing keeping this alive: RobotContext observes
  // it through a weak_ptr, so dropping the handle discards it at once. Held
  // until this event dies, since bookings outlive their task and ZoneEntry
  // must not send hints from a task that has ended.
  active->_preference = active->_context->set_zone_preference(
    active->_description.zone_name(),
    zone_modifiers(active->_description));

  active->_replan_subscription =
    active->_context->observe_replan_request()
    .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
    .subscribe(
    [w = active->weak_from_this()](const auto&)
    {
      if (const auto self = w.lock())
        self->_on_replan();
    });

  // A repeated GoToZone for a zone we are already parked in finishes
  // immediately through this path.
  const auto booking =
    active->_context->zone_booking(active->_description.zone_name());
  if (booking)
  {
    active->_booking = booking;
    active->_state->update_log().info(
      "Already assigned waypoint [" + booking->waypoint_name
      + "] in zone [" + active->_description.zone_name() + "]");
    active->_start_inner(booking->goal);
    return active;
  }

  active->_request_booking();
  return active;
}

//==============================================================================
GoToZone::Active::Active(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
void GoToZone::Active::_publish_entry_request()
{
  _current_request_id = phases::generate_zone_request_id(
    _context->group(), _context->name(), _description.zone_name());

  _context->node()->zone_request()->publish(
    phases::make_zone_entry_request(
      _context->group(),
      _context->name(),
      _description.zone_name(),
      _current_request_id,
      _preference ? *_preference : rmf_zone_msgs::msg::ZoneModifiers()));

  _has_pending_request = true;
}

//==============================================================================
void GoToZone::Active::_request_booking()
{
  const auto node = _context->node();

  _state->update_log().info(
    "Requesting a waypoint in zone [" + _description.zone_name() + "]");

  _requested_at = _context->now();

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  _state_sub = node->create_subscription<rmf_zone_msgs::msg::ZoneState>(
    ZoneStateTopicName, qos,
    [w = weak_from_this()](const rmf_zone_msgs::msg::ZoneState::SharedPtr msg)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto& zone_name = self->_description.zone_name();

      const auto result = phases::handle_zone_state(
        self->_context, *msg, zone_name, self->_current_request_id,
        "GoToZone");

      switch (result.status)
      {
        case phases::ZoneStateResult::Status::Granted:
        {
          // Hold our own reference so the release sweep can tell this booking
          // is in use for as long as this event lives.
          self->_booking = self->_context->zone_booking(zone_name);

          self->_state->update_log().info(
            "Assigned waypoint [" + result.waypoint_name + "]");

          // We have our vertex, so stop listening. Every ZoneState repeats
          // the whole booking log, so ours would go on matching here and
          // restarting the inner event on every zone change.
          self->_clear_manager_subscriptions();

          if (self->_interrupted)
            return;

          self->_start_inner(*result.goal);
          return;
        }

        case phases::ZoneStateResult::Status::UnknownWaypoint:
        {
          self->_state->update_status(Status::Error);
          self->_state->update_log().error(
            "assigned waypoint [" + result.waypoint_name
            + "] is not in the navigation graph");
          self->_clear_manager_subscriptions();
          self->_complete();
          return;
        }

        case phases::ZoneStateResult::Status::TicketMismatch:
        {
          self->_state->update_status(Status::Error);
          self->_state->update_log().error(
            "zone manager granted [" + result.waypoint_name + "] backed by a "
            "ticket for a different vertex, so it was not adopted");
          self->_clear_manager_subscriptions();
          self->_complete();
          return;
        }

        case phases::ZoneStateResult::Status::UnknownZone:
        {
          self->_state->update_status(Status::Error);
          self->_state->update_log().error(
            "zone [" + zone_name + "] is unknown to the Zone Manager");
          self->_clear_manager_subscriptions();
          self->_complete();
          return;
        }

        case phases::ZoneStateResult::Status::ZoneUnusable:
        {
          self->_state->update_status(Status::Error);
          self->_state->update_log().error(
            "zone [" + zone_name + "] cannot assign any waypoint ("
            + result.reason + ")");
          self->_clear_manager_subscriptions();
          self->_complete();
          return;
        }

        case phases::ZoneStateResult::Status::Deferred:
        {
          // The manager cannot answer yet, so keep waiting.
          self->_had_any_answer = true;
          self->_state->update_status(Status::Blocked);
          self->_state->update_log().info(
            "waiting for a free waypoint in zone [" + zone_name + "]: "
            + result.reason);
          self->_has_pending_request = false;
          return;
        }

        case phases::ZoneStateResult::Status::NoMatch:
          break;
      }

      if (self->_has_pending_request)
        return;

      if (self->_context->zone_booking(zone_name))
        return;

      self->_publish_entry_request();
    });

  _publish_entry_request();

  _request_timer = node->try_create_wall_timer(
    WAITING_WARN_INTERVAL,
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self || self->_completed || !self->_requested_at)
        return;

      const auto now = self->_context->now();
      const auto& zone = self->_description.zone_name();

      const auto elapsed = now - *self->_requested_at;
      const auto waited =
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

      const bool manager_silent =
        elapsed >= SUSPECT_MANAGER_AFTER && !self->_had_any_answer;

      const bool due = !self->_last_warned
        || now - *self->_last_warned >= WAITING_LOG_INTERVAL;

      if (due || (manager_silent && !self->_warned_manager_silent))
      {
        self->_last_warned = now;

        if (manager_silent)
        {
          self->_warned_manager_silent = true;
          RCLCPP_WARN(
            self->_context->node()->get_logger(),
            "GoToZone: [%s] has waited %lds for zone [%s] and the zone "
            "manager has never answered.",
            self->_context->requester_id().c_str(), waited, zone.c_str());
        }
        else
        {
          RCLCPP_WARN(
            self->_context->node()->get_logger(),
            "GoToZone: [%s] has been waiting %lds for a waypoint in zone [%s]",
            self->_context->requester_id().c_str(), waited, zone.c_str());
        }

        self->_state->update_log().info(
          "still waiting for a waypoint in zone [" + zone + "] after "
          + std::to_string(waited) + "s");
      }

      // Re-send zone entry request.
      if (!self->_has_pending_request
        && !self->_context->zone_booking(zone))
      {
        self->_publish_entry_request();
      }
    });
}

//==============================================================================
void GoToZone::Active::_clear_manager_subscriptions()
{
  _state_sub.reset();
  _request_timer.reset();
  _has_pending_request = false;
}

//==============================================================================
void GoToZone::Active::_start_inner(rmf_traffic::agv::Plan::Goal goal)
{
  // Dropping the last reference destroys the inner and, with it, the
  // callback below, so a replaced inner can never report completion.
  _inner = nullptr;

  _inner_goal = goal;

  const auto place_desc =
    rmf_task_sequence::events::GoToPlace::Description::make(goal);

  _inner = GoToPlace::Active::make(
    _assign_id,
    _context,
    *place_desc,
    std::nullopt,
    _state,
    _update,
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_complete();
    });
}

//==============================================================================
void GoToZone::Active::_on_replan()
{
  if (_completed || _interrupted)
    return;

  const auto booked = _context->zone_booking(_description.zone_name());

  if (!booked)
  {
    // Nothing booked for our zone. Either the request is outstanding or
    // the manager revoked it, and neither gives us a better target, so let
    // the inner replan toward what it has.
    _booking = nullptr;
    return;
  }

  _booking = booked;

  // Already aimed at the vertex we own, so leave the inner alone and let it
  // re-route.
  if (_inner && _inner_goal.has_value()
    && _inner_goal->waypoint() == booked->goal.waypoint())
  {
    const double* a = _inner_goal->orientation();
    const double* b = booked->goal.orientation();
    const bool same_orientation =
      (a == nullptr && b == nullptr)
      || (a != nullptr && b != nullptr && *a == *b);

    if (same_orientation)
      return;
  }

  // The vertex we own has changed, so re-aim.
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "GoToZone: re-aiming [%s] at waypoint [%s] in zone [%s]",
    _context->requester_id().c_str(),
    booked->waypoint_name.c_str(),
    _description.zone_name().c_str());

  _state->update_log().info(
    "re-aiming at waypoint [" + booked->waypoint_name + "]");

  _clear_manager_subscriptions();
  _start_inner(booked->goal);
}

//==============================================================================
void GoToZone::Active::_complete()
{
  if (_completed)
    return;

  _completed = true;
  _clear_manager_subscriptions();

  if (_finished)
    _finished();
}

//==============================================================================
auto GoToZone::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToZone::Active::remaining_time_estimate() const
{
  if (_inner)
    return _inner->remaining_time_estimate();

  return rmf_traffic::Duration(0);
}

//==============================================================================
auto GoToZone::Active::backup() const -> Backup
{
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto GoToZone::Active::interrupt(
  std::function<void()> task_is_interrupted) -> Resume
{
  _interrupted = true;

  if (_inner)
  {
    // Relay the interrupt to the inner GoToPlace
    auto inner_resume = std::make_shared<Resume>(
      _inner->interrupt(std::move(task_is_interrupted)));

    return Resume::make(
      [w = weak_from_this(), inner_resume]()
      {
        const auto self = w.lock();
        if (!self || self->_completed)
          return;

        self->_interrupted = false;
        (*inner_resume)();
      });
  }

  _clear_manager_subscriptions();

  _state->update_status(Status::Standby);
  _state->update_log().info("Going into standby for an interruption");

  _context->worker().schedule(
    [task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });

  return Resume::make(
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self || self->_completed)
        return;

      self->_interrupted = false;

      const auto booking =
        self->_context->zone_booking(self->_description.zone_name());
      if (booking)
      {
        self->_booking = booking;
        self->_start_inner(booking->goal);
        return;
      }

      self->_request_booking();
    });
}

//==============================================================================
void GoToZone::Active::cancel()
{
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Canceling GoToZone for robot [%s]",
    _context->requester_id().c_str());

  _stop(false);
}

//==============================================================================
void GoToZone::Active::kill()
{
  _stop(true);
}

//==============================================================================
void GoToZone::Active::_stop(const bool killed)
{
  const bool had_pending_request = _has_pending_request;

  _clear_manager_subscriptions();

  if (_inner)
  {
    if (killed)
      _inner->kill();
    else
      _inner->cancel();
  }
  _inner = nullptr;
  _booking = nullptr;

  // Only for a request still in flight, which the manager may grant after we
  // stop listening. Nothing else would release that. Skipped when we hold a
  // booking, since the robot may be sitting on it.
  if (had_pending_request
    && !_context->zone_booking(_description.zone_name()))
  {
    _context->node()->zone_request()->publish(
      phases::make_zone_exit_request(
        _context->group(), _context->name(), _description.zone_name()));
  }

  _state->update_status(killed ? Status::Killed : Status::Canceled);
  _state->update_log().info(
    killed ? "Received signal to kill" : "Received signal to cancel");
  _complete();
}

} // namespace events
} // namespace rmf_fleet_adapter
