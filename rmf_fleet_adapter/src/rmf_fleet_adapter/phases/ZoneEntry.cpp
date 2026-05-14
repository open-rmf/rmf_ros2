#include "ZoneEntry.hpp"
#include "Utils.hpp"

namespace rmf_fleet_adapter {
namespace phases {

namespace {
//==============================================================================
rmf_zone_msgs::msg::ZoneModifiers to_msg_modifiers(
  const agv::RobotContext::ZoneTaskModifiers& mods)
{
  rmf_zone_msgs::msg::ZoneModifiers out;
  out.group_hint = mods.group_hint;
  out.has_orientation_hint = mods.orientation_hint.has_value();
  out.orientation_hint = mods.orientation_hint.value_or(0.0);
  out.preferred_waypoints = mods.preferred_waypoints;
  return out;
}
} // anonymous namespace

//==============================================================================
ZoneEntry::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string zone_name,
  rmf_traffic::Time expected_finish,
  std::shared_ptr<rmf_traffic::schedule::Itinerary> resume_itinerary,
  std::shared_ptr<rmf_traffic::PlanId> plan_id)
: _context(std::move(context)),
  _description("Requesting zone entry for [" + zone_name + "]")
{
  _data.zone_name = std::move(zone_name);
  _data.expected_finish = expected_finish;
  _data.resume_itinerary = std::move(resume_itinerary);
  _data.plan_id = std::move(plan_id);
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> ZoneEntry::PendingPhase::begin()
{
  return ActivePhase::make(_context, std::move(_data));
}

//==============================================================================
rmf_traffic::Duration ZoneEntry::PendingPhase::estimate_phase_duration() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& ZoneEntry::PendingPhase::description() const
{
  return _description;
}

//==============================================================================
std::shared_ptr<ZoneEntry::ActivePhase> ZoneEntry::ActivePhase::make(
  agv::RobotContextPtr context, Data data)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(std::move(context), std::move(data)));
  inst->_init_obs();
  return inst;
}

//==============================================================================
ZoneEntry::ActivePhase::ActivePhase(
  agv::RobotContextPtr context, Data data)
: _context(std::move(context)),
  _description_text("Requesting zone entry for [" + data.zone_name + "]"),
  _data(std::move(data))
{

}

//==============================================================================
void ZoneEntry::ActivePhase::_init_obs()
{
  _obs = rxcpp::observable<>::create<LegacyTask::StatusMsg>(
    [w = weak_from_this()](rxcpp::subscriber<LegacyTask::StatusMsg> s)
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto node = self->_context->node();

      auto qos = rclcpp::QoS(10).transient_local().reliable();
      self->_state_sub = node->create_subscription<
        rmf_zone_msgs::msg::ZoneState>(
        ZoneStateTopicName, qos,
        [w, s](const rmf_zone_msgs::msg::ZoneState::SharedPtr msg)
        {
          const auto self = w.lock();
          if (!self)
            return;

          const auto& robot_name = self->_context->name();
          const auto& fleet_name = self->_context->group();

          // Check bookings
          for (const auto& booking : msg->bookings)
          {
            if (booking.robot_name == robot_name
              && booking.fleet_name == fleet_name
              && booking.zone_name == self->_data.zone_name
              && booking.request_id == self->_current_request_id)
            {
              RCLCPP_INFO(self->_context->node()->get_logger(),
                "Zone booking confirmed: [%s] assigned to waypoint [%s] "
                "in zone [%s]",
                robot_name.c_str(),
                booking.assigned_waypoint_name.c_str(),
                booking.zone_name.c_str());

              self->_context->schedule_itinerary(
                self->_data.plan_id, *self->_data.resume_itinerary);

              const auto& graph = self->_context->navigation_graph();
              const auto* wp = graph.find_waypoint(
                booking.assigned_waypoint_name);
              if (!wp)
              {
                RCLCPP_ERROR(self->_context->node()->get_logger(),
                  "Zone supervisor assigned waypoint [%s] which does not "
                  "exist in the navigation graph for robot [%s]",
                  booking.assigned_waypoint_name.c_str(),
                  robot_name.c_str());

                self->_delay_timer.reset();
                self->_state_sub.reset();

                auto error = LegacyTask::StatusMsg();
                error.state = LegacyTask::StatusMsg::STATE_FAILED;
                error.status = "assigned_waypoint_not_in_graph";
                s.on_next(error);
                s.on_completed();
                return;
              }

              rmf_traffic::agv::Plan::Goal goal(wp->index());
              if (booking.has_orientation)
                goal = rmf_traffic::agv::Plan::Goal(
                  wp->index(), booking.orientation);

              self->_context->set_booked_zone_goal(goal);
              self->_context->set_booked_zone_waypoint(
                booking.assigned_waypoint_name);

              self->_context->request_replan();

              self->_delay_timer.reset();
              self->_state_sub.reset();

              auto completion = LegacyTask::StatusMsg();
              completion.state = LegacyTask::StatusMsg::STATE_COMPLETED;
              s.on_next(completion);
              s.on_completed();
              return;
            }
          }

          // Check rejections
          for (const auto& rejection : msg->rejected)
          {
            if (rejection.robot_name == robot_name
              && rejection.fleet_name == fleet_name
              && rejection.request_id == self->_current_request_id)
            {
              if (rejection.reason == "unknown_zone")
              {
                RCLCPP_ERROR(self->_context->node()->get_logger(),
                  "Zone request rejected: unknown zone [%s]",
                  self->_data.zone_name.c_str());
                self->_delay_timer.reset();
                self->_state_sub.reset();
                auto error = LegacyTask::StatusMsg();
                error.state = LegacyTask::StatusMsg::STATE_FAILED;
                error.status = "unknown_zone";
                s.on_next(error);
                s.on_completed();
                return;
              }

              // Stay subscribed and retry on next state change
              // (could be zone full)
              RCLCPP_WARN(self->_context->node()->get_logger(),
                "Zone request rejected: %s, waiting for state change",
                rejection.reason.c_str());

              self->_has_pending_request = false;
              return;
            }
          }

          // nothing to do if we already have a booking or
          // a request in flight
          if (!self->_context->booked_zone_waypoint().empty())
            return;
          if (self->_has_pending_request)
            return;

          // Re-request after a rejection
          self->_current_request_id = generate_zone_request_id(
            self->_context->group(), self->_context->name(),
            self->_data.zone_name);
          auto request = rmf_zone_msgs::msg::ZoneRequest();
          request.robot_name = self->_context->name();
          request.fleet_name = self->_context->group();
          request.request_id = self->_current_request_id;
          request.zone_name = self->_data.zone_name;
          request.request_type = rmf_zone_msgs::msg::ZoneRequest::ENTRY;
          request.modifiers =
            to_msg_modifiers(self->_context->zone_task_modifiers());

          self->_request_pub->publish(request);
          self->_has_pending_request = true;
        });

      // Publish initial ENTRY request
      self->_request_pub = node->create_publisher<
        rmf_zone_msgs::msg::ZoneRequest>(
        ZoneRequestTopicName, rclcpp::QoS(10).reliable());

      self->_current_request_id = generate_zone_request_id(
        self->_context->group(), self->_context->name(),
        self->_data.zone_name);
      auto request = rmf_zone_msgs::msg::ZoneRequest();
      request.robot_name = self->_context->name();
      request.fleet_name = self->_context->group();
      request.request_id = self->_current_request_id;
      request.zone_name = self->_data.zone_name;
      request.request_type = rmf_zone_msgs::msg::ZoneRequest::ENTRY;
      request.modifiers =
        to_msg_modifiers(self->_context->zone_task_modifiers());

      self->_request_pub->publish(request);
      self->_has_pending_request = true;

      // Report delay while waiting for zone supervisor
      self->_delay_timer = node->try_create_wall_timer(
        std::chrono::milliseconds(1000),
        [w]()
        {
          const auto self = w.lock();
          if (!self)
            return;

          const auto delay =
            self->_context->now() - self->_data.expected_finish;
          if (delay > std::chrono::seconds(0))
          {
            self->_context->worker().schedule(
              [
                context = self->_context,
                plan_id = *self->_data.plan_id,
                delay
              ](const auto&)
              {
                context->itinerary().cumulative_delay(plan_id, delay);
              });
          }
        });
    });
}

//==============================================================================
const rxcpp::observable<LegacyTask::StatusMsg>&
ZoneEntry::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration ZoneEntry::ActivePhase::estimate_remaining_time() const
{
  // TODO: implement
  return rmf_traffic::Duration{0};
}

//==============================================================================
void ZoneEntry::ActivePhase::emergency_alarm(bool /*on*/)
{
  // TODO: implement
}

//==============================================================================
void ZoneEntry::ActivePhase::cancel()
{
  _state_sub.reset();
  _delay_timer.reset();

  // Always send EXIT on cancel. If no booking exists, this is a defensive
  // guard against a race where the supervisor grants one after we stop
  // listening. If a booking exists, ZoneExit will never run (task is being
  // cancelled), so we must release it here.
  if (_request_pub)
  {
    auto request = rmf_zone_msgs::msg::ZoneRequest();
    request.robot_name = _context->name();
    request.fleet_name = _context->group();
    request.request_id = generate_zone_request_id(
      _context->group(), _context->name(), _data.zone_name);
    request.zone_name = _data.zone_name;
    request.request_type = rmf_zone_msgs::msg::ZoneRequest::EXIT;
    _request_pub->publish(request);
  }

  // Clear fleet adapter side zone state if a booking was granted
  if (!_context->booked_zone_waypoint().empty())
  {
    _context->clear_booked_zone_goal();
    _context->clear_booked_zone_waypoint();
  }

}

//==============================================================================
const std::string& ZoneEntry::ActivePhase::description() const
{
  return _description_text;
}

} // namespace phases
} // namespace rmf_fleet_adapter
