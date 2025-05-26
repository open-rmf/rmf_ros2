/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_RESERVATION_NEGOTIATOR_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_RESERVATION_NEGOTIATOR_HPP

#include "../agv/RobotContext.hpp"
#include "internal_utilities.hpp"
#include <memory.h>

namespace rmf_fleet_adapter {
namespace reservation {

/// This class implements the protocol for negotiating a spot with the "reservation"
/// node. The reservation node maintains a list of spots which are free.
class ReservationNodeNegotiator :
  public std::enable_shared_from_this<ReservationNodeNegotiator>
{
public:
  static std::shared_ptr<ReservationNodeNegotiator> make(
    std::shared_ptr<agv::RobotContext> context,
    const std::vector<rmf_traffic::agv::Plan::Goal> goals,
    const bool same_map,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)>
    selected_final_destination_cb,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)> selected_waitpoint_cb)
  {
    auto negotiator = std::shared_ptr<ReservationNodeNegotiator>(
      new ReservationNodeNegotiator(
        context, goals, selected_final_destination_cb, selected_waitpoint_cb));

    negotiator->_reservation_ticket =
      context->node()->location_ticket_obs().observe_on(rxcpp::identity_same_worker(
          context->worker()))
      .subscribe([ptr = negotiator->weak_from_this()](
        const std::shared_ptr<rmf_reservation_msgs::msg::Ticket>
        & msg)
        {
          auto self = ptr.lock();
          if(!self)
          {
            return;
          }

          RCLCPP_DEBUG(
            self->_context->node()->get_logger(),
            "Reservations: Got ticket issueing claim");

          if (msg->header.request_id != self->_reservation_id
          || msg->header.robot_name != self->_context->name()
          || msg->header.fleet_name != self->_context->group())
          {
            return;
          }

          self->_ticket = msg;
          if(self->_goals.size() == 1)
          {
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Sorting waitpoint by distance from goal %lu",
              self->_goals[0].waypoint());

            // If there is only one destination to go to then we should rank
            // waiting spots by their distance from said destination.
            // In this case we allow the robot to board lifts to go to the waiting pot nearest
            // their target.
            self->_waitpoints = self->_context->_find_and_sort_parking_spots(
              self->_goals[0], false);
          }
          else
          {
            // Otherwise go to the nearest destination based on your current location
            self->_waitpoints = self->_context->_find_and_sort_parking_spots(
              true);
          }
          if (self->_waitpoints.size() == 0)
          {
            // This may happen if the robot is lost.
            RCLCPP_ERROR(
              self->_context->node()->get_logger(),
              "Reservations: Got no waitpoints for %s", self->_context->requester_id().c_str());
            return;
          }

          // Immediately make claim cause we don't yet support flexible reservations.
          rmf_reservation_msgs::msg::ClaimRequest claim_request;
          claim_request.ticket = *msg;
          for (const auto& goal: self->_waitpoints)
          {
            auto wp =
            self->_context->navigation_graph().get_waypoint(goal.waypoint());
            claim_request.wait_points.push_back(*wp.name());
          }
          self->_context->node()->claim_location_ticket()->publish(
            claim_request);
          RCLCPP_DEBUG(
            self->_context->node()->get_logger(),
            "Reservations: Claim issued by %s",  self->_context->name().c_str());
        });


    negotiator->_reservation_allocation =
      context->node()->allocated_claims_obs().observe_on(rxcpp::identity_same_worker(
          context->worker()))
      .subscribe([ptr = negotiator->weak_from_this()](const std::shared_ptr<rmf_reservation_msgs::msg::ReservationAllocation>
        & msg)
        {
          auto self = ptr.lock();
          if(!self)
          {
            return;
          }
          if (!self->_ticket.has_value())
          {
            return;
          }

          if (msg->ticket.ticket_id != self->_ticket.value()->ticket_id)
          {
            return;
          }

          self->_final_allocated_destination = msg;
          self->_context->_set_allocated_destination(*msg.get());

          if (msg->instruction_type
          == rmf_reservation_msgs::msg::ReservationAllocation::IMMEDIATELY_PROCEED)
          {
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Reservation: Robot %s is going to final destination %lu",
              self->_context->name().c_str(),
              self->_goals[self->_final_allocated_destination.value()->
              chosen_alternative].waypoint());
            self->_current_reservation_state = ReservationState::ReceivedResponseProceedImmediate;
            self->_selected_final_destination_cb(self->_goals[self->
            _final_allocated_destination.value()->
            chosen_alternative].waypoint());
          }

          if (msg->instruction_type
          == rmf_reservation_msgs::msg::ReservationAllocation::WAIT_IDENTIFIED)
          {
            self->_current_reservation_state = ReservationState::ReceivedResponseProceedWaitPoint;
            self->_selected_waitpoint_cb(self->_waitpoints[self->
            _final_allocated_destination.value()->
            chosen_alternative]);
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Reservation: Robot %s is being asked to proceed to a waitpoint %lu",
              self->_context->name().c_str(),
              self->_waitpoints[self->_final_allocated_destination.value()->
              chosen_alternative].waypoint());
          }
        });


    for (std::size_t i = 0; i < negotiator->_goals.size(); ++i)
    {
      if (events::wp_name(*context.get(), negotiator->_goals[i]) == context->_get_reserved_location())
      {
        RCLCPP_INFO(context->node()->get_logger(),
          "%s: Already at goal no need to engage reservation system\n",
          context->requester_id().c_str());
        context->worker().schedule([
            cb = negotiator->_selected_final_destination_cb,
            wp = negotiator->_goals[i]
          ](const auto&)
          {
            cb(wp);
          });
        return negotiator;
      }
    }
    RCLCPP_INFO(negotiator->_context->node()->get_logger(),
      "%s: Sending reservation request",
      negotiator->_context->requester_id().c_str());

    context->worker().schedule(
      [ptr = negotiator->weak_from_this(), same_map](
        const auto&)
    {
      auto negotiator = ptr.lock();
      if (!negotiator)
        return;

      negotiator->make_request(same_map);
    });
    return negotiator;
  }

private:

  ReservationNodeNegotiator(
    std::shared_ptr<agv::RobotContext> context,
    const std::vector<rmf_traffic::agv::Plan::Goal> goals,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)>
    selected_final_destination_cb,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)> selected_waitpoint_cb)
  {
    _context = context;
    _goals = goals;
    _selected_final_destination_cb = std::move(selected_final_destination_cb);
    _selected_waitpoint_cb = std::move(selected_waitpoint_cb);
    _reservation_id = _context->last_reservation_request_id();
  }

  enum class ReservationState
  {
    Pending=0,
    Requested=1,
    ReceivedResponseProceedWaitPoint=2,
    ReceivedResponseProceedImmediate=3
  };


  void make_request(bool only_same_map)
  {
    auto current_location = _context->location();
    const auto& graph = _context->navigation_graph();
    if (current_location.size() == 0)
    {
      //unable to get location. We should return some form of error state.
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "reservation: Robot [%s] can't get location",
        _context->requester_id().c_str());
      return;
    }

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Reservation Negotiator: Selecting a new go_to_place location from [%lu] choices for robot [%s]",
      _goals.size(),
      _context->requester_id().c_str());

    if (_current_reservation_state == ReservationState::Pending)
    {
      // Submit costs of each alternative
      rmf_reservation_msgs::msg::FlexibleTimeRequest ftr;
      ftr.header.robot_name = _context->name();
      ftr.header.fleet_name = _context->group();
      ftr.header.request_id = _reservation_id;
      ftr.header.task_id = _context->copy_current_task_id();

      std::optional<std::size_t> selected_idx;
      for (std::size_t i = 0; i < _goals.size(); ++i)
      {
        const auto wp_idx = _goals[i].waypoint();
        const auto& wp = graph.get_waypoint(wp_idx);

        if (only_same_map)
        {

          // Check if same map. If not don't consider location. This is to ensure
          // the robot does not try to board a lift.
          if (wp.get_map_name() != _context->map())
          {
            RCLCPP_INFO(
              _context->node()->get_logger(),
              "Skipping [%lu] as it is on map [%s] but robot is on map [%s].",
              wp_idx,
              wp.get_map_name().c_str(),
              _context->map().c_str());
            continue;
          }
        }

        // Find distance to said point
        auto result =
          _context->planner()->quickest_path(current_location, wp_idx);
        if (result.has_value())
        {
          RCLCPP_INFO(
            _context->node()->get_logger(),
            "Got distance from [%lu] as %f",
            wp_idx,
            result->cost());

          const auto* name = wp.name();
          if (name == nullptr)
          {
            RCLCPP_ERROR(_context->node()->get_logger(),
              "Got a parking spot without a name."
              "This parking spot will not be used by the reservation system.");
            continue;
          }

          rmf_reservation_msgs::msg::FlexibleTimeReservationAlt alternative;
          alternative.resource_name = *name;
          alternative.cost = result->cost();
          alternative.has_end = false;

          rmf_reservation_msgs::msg::StartTimeRange start;
          start.earliest_start_time = _context->node()->get_clock()->now();
          start.latest_start_time = start.earliest_start_time;
          start.has_earliest_start_time = true;
          start.has_latest_start_time = true;
          alternative.start_time = start;

          ftr.alternatives.push_back(alternative);
        }
        else
        {
          RCLCPP_ERROR(
            _context->node()->get_logger(),
            "No path found for robot [%s] to waypoint [%lu]",
            _context->requester_id().c_str(),
            wp_idx);
        }
      }
      _context->node()->location_requester()->publish(ftr);
      _current_reservation_state = ReservationState::Requested;
    }
  }

  ReservationState _current_reservation_state = ReservationState::Pending;
  std::shared_ptr<agv::RobotContext> _context;
  std::function<void(const rmf_traffic::agv::Plan::Goal&)>
  _selected_waitpoint_cb;
  std::function<void(const rmf_traffic::agv::Plan::Goal&)>
  _selected_final_destination_cb;
  rmf_rxcpp::subscription_guard _reservation_ticket;
  rmf_rxcpp::subscription_guard _reservation_allocation;

  uint64_t _reservation_id = 0;
  std::optional<std::shared_ptr<rmf_reservation_msgs::msg::Ticket>> _ticket{std::
    nullopt};
  std::optional<std::shared_ptr<rmf_reservation_msgs::msg::ReservationAllocation>>
  _final_allocated_destination{std::nullopt};

  std::vector<rmf_traffic::agv::Plan::Goal> _goals;
  std::vector<rmf_traffic::agv::Plan::Goal> _waitpoints;
};
} // namespace rmf_fleet_adapter
} // namespace

#endif
