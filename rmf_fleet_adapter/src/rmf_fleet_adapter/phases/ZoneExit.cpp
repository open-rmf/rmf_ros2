#include "ZoneExit.hpp"
#include "Utils.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
ZoneExit::PendingPhase::PendingPhase(
  agv::RobotContextPtr context,
  std::string zone_name,
  rmf_traffic::Time expected_finish,
  std::shared_ptr<rmf_traffic::PlanId> plan_id)
: _context(std::move(context)),
  _expected_finish(expected_finish),
  _plan_id(std::move(plan_id)),
  _description("Releasing zone booking for [" + zone_name + "]")
{
  _zone_name = std::move(zone_name);
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> ZoneExit::PendingPhase::begin()
{
  return ActivePhase::make(
    _context, std::move(_zone_name), _expected_finish, _plan_id);
}

//==============================================================================
rmf_traffic::Duration ZoneExit::PendingPhase::estimate_phase_duration() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& ZoneExit::PendingPhase::description() const
{
  return _description;
}

//==============================================================================
std::shared_ptr<ZoneExit::ActivePhase> ZoneExit::ActivePhase::make(
  agv::RobotContextPtr context,
  std::string zone_name,
  rmf_traffic::Time expected_finish,
  std::shared_ptr<rmf_traffic::PlanId> plan_id)
{
  auto inst = std::shared_ptr<ActivePhase>(
    new ActivePhase(
      std::move(context), std::move(zone_name),
      expected_finish, std::move(plan_id)));
  inst->_init_obs();
  return inst;
}

//==============================================================================
ZoneExit::ActivePhase::ActivePhase(
  agv::RobotContextPtr context,
  std::string zone_name,
  rmf_traffic::Time expected_finish,
  std::shared_ptr<rmf_traffic::PlanId> plan_id)
: _context(std::move(context)),
  _expected_finish(expected_finish),
  _plan_id(std::move(plan_id)),
  _description_text("Releasing zone booking for [" + zone_name + "]")
{
  _assigned_waypoint = _context->booked_zone_waypoint();
  _zone_name = std::move(zone_name);

  if (_assigned_waypoint.empty())
  {
    RCLCPP_WARN(_context->node()->get_logger(),
      "ZoneExit created for zone [%s] but robot [%s] has no active "
      "zone booking, phase will complete immediately",
      _zone_name.c_str(), _context->requester_id().c_str());
  }
}

//==============================================================================
void ZoneExit::ActivePhase::_init_obs()
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

          for (const auto& booking : msg->bookings)
          {
            if (booking.assigned_waypoint_name == self->_assigned_waypoint
              && booking.robot_name == self->_context->name()
              && booking.fleet_name == self->_context->group()
              && booking.zone_name == self->_zone_name)
              return;  // still booked
          }

          // Booking gone from snapshot, release confirmed
          RCLCPP_INFO(self->_context->node()->get_logger(),
            "Zone booking released: [%s] cleared from zone [%s]",
            self->_assigned_waypoint.c_str(),
            self->_zone_name.c_str());

          self->_context->clear_booked_zone_goal();
          self->_context->clear_booked_zone_waypoint();

          self->_timeout_timer.reset();
          self->_delay_timer.reset();
          self->_state_sub.reset();

          auto completion = LegacyTask::StatusMsg();
          completion.state = LegacyTask::StatusMsg::STATE_COMPLETED;
          s.on_next(completion);
          s.on_completed();
        });

      // Publish EXIT request
      self->_request_pub = node->create_publisher<
        rmf_zone_msgs::msg::ZoneRequest>(
        ZoneRequestTopicName, rclcpp::QoS(10).reliable());

      auto request = rmf_zone_msgs::msg::ZoneRequest();
      request.robot_name = self->_context->name();
      request.fleet_name = self->_context->group();
      request.request_id = generate_zone_request_id(
        self->_context->group(), self->_context->name(), self->_zone_name);
      request.zone_name = self->_zone_name;
      request.request_type = rmf_zone_msgs::msg::ZoneRequest::EXIT;

      self->_request_pub->publish(request);

      // Fail if supervisor doesn't confirm within 30s
      self->_timeout_timer = node->try_create_wall_timer(
        std::chrono::seconds(30),
        [w, s]()
        {
          const auto self = w.lock();
          if (!self)
            return;

          RCLCPP_ERROR(self->_context->node()->get_logger(),
            "ZoneExit timeout: [%s] not released from zone [%s] within 30s. "
            "Supervisor may hold a stale booking.",
            self->_assigned_waypoint.c_str(),
            self->_zone_name.c_str());

          // Re-send EXIT in case the first was lost
          if (self->_request_pub)
          {
            auto request = rmf_zone_msgs::msg::ZoneRequest();
            request.robot_name = self->_context->name();
            request.fleet_name = self->_context->group();
            request.request_id = generate_zone_request_id(
              self->_context->group(), self->_context->name(),
              self->_zone_name);
            request.zone_name = self->_zone_name;
            request.request_type = rmf_zone_msgs::msg::ZoneRequest::EXIT;
            self->_request_pub->publish(request);
          }

          // Clean up locally even on timeout
          self->_context->clear_booked_zone_goal();
          self->_context->clear_booked_zone_waypoint();

          self->_state_sub.reset();
          self->_delay_timer.reset();

          auto error = LegacyTask::StatusMsg();
          error.state = LegacyTask::StatusMsg::STATE_FAILED;
          error.status = "zone_exit_timeout";
          s.on_next(error);
          s.on_completed();
        });

      // Report delay while waiting
      self->_delay_timer = node->try_create_wall_timer(
        std::chrono::milliseconds(1000),
        [w]()
        {
          const auto self = w.lock();
          if (!self)
            return;

          const auto delay =
            self->_context->now() - self->_expected_finish;
          if (delay > std::chrono::seconds(0))
          {
            self->_context->worker().schedule(
              [
                context = self->_context,
                plan_id = *self->_plan_id,
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
ZoneExit::ActivePhase::observe() const
{
  return _obs;
}

//==============================================================================
rmf_traffic::Duration ZoneExit::ActivePhase::estimate_remaining_time() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
void ZoneExit::ActivePhase::emergency_alarm(bool /*on*/)
{
}

//==============================================================================
void ZoneExit::ActivePhase::cancel()
{
  _state_sub.reset();
  _timeout_timer.reset();
  _delay_timer.reset();

  RCLCPP_WARN(_context->node()->get_logger(),
    "ZoneExit cancelled for zone [%s] -- booking for [%s] "
    "persists on supervisor",
    _zone_name.c_str(), _assigned_waypoint.c_str());
}

//==============================================================================
const std::string& ZoneExit::ActivePhase::description() const
{
  return _description_text;
}

} // namespace phases
} // namespace rmf_fleet_adapter
