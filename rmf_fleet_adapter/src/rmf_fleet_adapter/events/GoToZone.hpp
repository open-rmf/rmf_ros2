#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP

#include "../agv/RobotContext.hpp"

#include <rmf_zone_msgs/msg/zone_state.hpp>

#include <rmf_rxcpp/RxJobs.hpp>

#include <optional>
#include <rmf_task_sequence/Event.hpp>
#include <rmf_task_sequence/events/GoToZone.hpp>
#include <rmf_task/events/SimpleEventState.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
class GoToZone : public rmf_task_sequence::Event
{
public:

  using Description = rmf_task_sequence::events::GoToZone::Description;

  static void add(rmf_task_sequence::Event::Initializer& initializer);

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:

    static std::shared_ptr<Standby> make(
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:

    Standby(Description description);

    Description _description;
    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    rmf_traffic::Duration _time_estimate;
    std::function<void()> _update;
    rmf_task::events::SimpleEventStatePtr _state;
    ActivePtr _active = nullptr;
  };

  //============================================================================
  /// Owns the journey into a zone.
  ///
  /// GoToZone books a zone vertex from the Zone Manager up front and then
  /// drives a single inner GoToPlace to it. The vertex is not fixed. As the
  /// robot crosses the boundary the entry lane event re-asks the manager
  /// with the preference we set, seats whatever comes back, drives its own
  /// hop to it, and then requests a replan. We stopped listening to the
  /// manager once we were granted, so that replan is all we see, and
  /// reacting to it is what this class is for.
  ///
  /// This class also holds _booking for as long as it lives. The release
  /// sweep gives a vertex back once no event is using the booking and the
  /// robot is outside the zone, and a robot driving towards a zone is
  /// outside it the whole way.
  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:

    static std::shared_ptr<Active> make(
      const AssignIDPtr& id,
      agv::RobotContextPtr context,
      Description description,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> update,
      std::function<void()> finished);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

    void _stop(bool killed);

  private:

    Active(Description description);

    /// Ask the manager for a vertex in the destination zone and wait for
    /// the grant. Used only when we do not already hold a booking.
    void _request_booking();

    /// Publish a zone PREBOOKING request for the destination zone.
    void _publish_prebooking_request();

    /// Drop any inner event and start a fresh GoToPlace toward goal.
    void _start_inner(rmf_traffic::agv::Plan::Goal goal);

    /// Re-aim the inner event when the vertex we own is no longer the one
    /// it is driving to.
    void _on_replan();

    /// Finish the event.
    void _complete();

    /// Stop listening to the manager.
    void _clear_manager_subscriptions();

    AssignIDPtr _assign_id;
    agv::RobotContextPtr _context;
    Description _description;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _update;
    std::function<void()> _finished;

    ActivePtr _inner = nullptr;
    std::optional<rmf_traffic::agv::Plan::Goal> _inner_goal;

    /// Our reference to the booking. Holding it is what tells the fleet
    /// adapter's release sweep that this booking is in use. The sweep only
    /// reclaims bookings whose reference count has dropped back to 1.
    agv::RobotContext::ZoneBookingPtr _booking;

    bool _completed = false;

    rmf_rxcpp::subscription_guard _replan_subscription;

    rclcpp::Subscription<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_sub;
    agv::RobotContext::ZonePreferenceHandle _preference;

    std::string _current_request_id;
    bool _has_pending_request = false;
    bool _interrupted = false;

    /// Ticks while we are waiting on the zone manager. Re-sends the request
    /// and warns periodically.
    rclcpp::TimerBase::SharedPtr _request_timer;

    /// When we first asked, so the periodic warning can report how long we
    /// have been waiting.
    std::optional<rmf_traffic::Time> _requested_at;

    /// When we last said we were still waiting.
    std::optional<rmf_traffic::Time> _last_warned;

    // Whether the manager has responded to our request at all. A grant and
    // a refusal both count.
    bool _had_any_answer = false;

    // Whether the louder complaint about a silent manager has been made.
    bool _warned_manager_silent = false;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__GOTOZONE_HPP
