/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__EVENTS__ZONEPREENTRY_HPP
#define SRC__RMF_FLEET_ADAPTER__EVENTS__ZONEPREENTRY_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"
#include "../phases/MoveRobot.hpp"

#include <optional>

#include <rmf_task/events/SimpleEventState.hpp>
#include <rmf_task_sequence/Event.hpp>

#include <rmf_zone_msgs/msg/zone_state.hpp>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
/// Confirms which vertex a robot gets as it crosses a zone's entry lane,
/// holding it there until the zone manager answers. Every robot crossing the
/// lane runs this, booked or not. A grant that names the vertex the robot
/// already held changes nothing, so the plan carries on untouched. Any other
/// answer is driven to before requesting a replan.
class ZonePreEntry : public rmf_task_sequence::Event
{
public:

  struct Data
  {
    std::string zone_name;
    rmf_traffic::Time expected_finish;
    std::shared_ptr<rmf_traffic::PlanId> plan_id;

    /// The zone this plan ends in, or an empty string for none.
    std::optional<std::string> plan_end_zone;

    std::shared_ptr<rmf_traffic::schedule::Itinerary> resume_itinerary;
  };

  class Standby : public rmf_task_sequence::Event::Standby
  {
  public:
    static std::shared_ptr<Standby> make(
      agv::RobotContextPtr context,
      const AssignIDPtr& id,
      Data data);

    ConstStatePtr state() const final;

    rmf_traffic::Duration duration_estimate() const final;

    ActivePtr begin(
      std::function<void()> checkpoint,
      std::function<void()> finished) final;

  private:
    Standby(Data data);
    agv::RobotContextPtr _context;
    AssignIDPtr _assign_id;
    rmf_task::events::SimpleEventStatePtr _state;
    Data _data;
  };

  class Active
    : public rmf_task_sequence::Event::Active,
    public std::enable_shared_from_this<Active>
  {
  public:
    static std::shared_ptr<Active> make(
      agv::RobotContextPtr context,
      const AssignIDPtr& id,
      rmf_task::events::SimpleEventStatePtr state,
      std::function<void()> finished,
      Data data);

    ConstStatePtr state() const final;

    rmf_traffic::Duration remaining_time_estimate() const final;

    Backup backup() const final;

    Resume interrupt(std::function<void()> task_is_interrupted) final;

    void cancel() final;

    void kill() final;

  private:
    Active(Data data);

    void _initialize();

    /// Send the finalize request and mark one as outstanding.
    void _publish_finalize_request();

    /// Drive onto the vertex the manager just assigned.
    ///
    /// MoveRobot rather than a nested GoToPlace, because GoToPlace consults
    /// the reservation system, and a vertex it finds booked can send the
    /// robot to a parking spot instead of into the zone.
    void _begin_move(
      rmf_traffic::agv::Plan::Goal goal,
      const std::string& waypoint_name);

    /// Announce arrival on the vertex, then ask for a replan and finish.
    void _finish_at_waypoint();

    /// Put back the itinerary ExecutePlan cut at the boundary, for the paths
    /// that leave the plan to carry on.
    void _resume_plan();

    /// Ask for a replan so the driving event re-aims from here, then finish.
    void _finish_with_replan();

    void _on_request_timer();

    void _complete(Status status);

    agv::RobotContextPtr _context;
    AssignIDPtr _assign_id;
    rmf_task::events::SimpleEventStatePtr _state;
    std::function<void()> _finished;

    /// The booking we are finalizing. Our reference is what tells the release
    /// sweep it is in use.
    agv::RobotContext::ZoneBookingPtr _booking;

    /// The vertex we arrived holding, which is the one the plan is aimed at.
    std::string _entry_waypoint;

    /// The post-entry event, held while it runs so its completion can
    /// carry on into the replan.
    ActivePtr _post_entry;

    rclcpp::Subscription<rmf_zone_msgs::msg::ZoneState>::SharedPtr _state_sub;

    std::string _current_request_id;
    bool _has_pending_request = false;

    rclcpp::TimerBase::SharedPtr _delay_timer;

    /// Re-sends the request while we wait, and escalates the log if the
    /// manager never answers at all.
    rclcpp::TimerBase::SharedPtr _request_timer;
    std::optional<rmf_traffic::Time> _requested_at;
    std::optional<rmf_traffic::Time> _last_warned;
    bool _had_any_answer = false;
    bool _warned_manager_silent = false;

    /// Only the first request is announced.
    bool _announced_request = false;

    std::shared_ptr<phases::MoveRobot::ActivePhase> _move;
    rmf_rxcpp::subscription_guard _move_sub;

    Data _data;
  };
};

} // namespace events
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__EVENTS__ZONEPREENTRY_HPP
