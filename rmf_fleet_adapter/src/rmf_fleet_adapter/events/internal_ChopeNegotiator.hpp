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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_CHOPE_NEGOTIATOR_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_CHOPE_NEGOTIATOR_HPP

#include "../agv/RobotContext.hpp"

#include <memory.h>

namespace rmf_fleet_adapter {
namespace chope {

/// This class implements the protocol for negotiating a spot with the "chope"
/// node. The chope node maintains a list of spots which are free.
class ChopeNodeNegotiator:
  public std::enable_shared_from_this<ChopeNodeNegotiator>
{
public:
  static std::shared_ptr<ChopeNodeNegotiator> make(
    std::shared_ptr<agv::RobotContext> context,
    const std::vector<rmf_traffic::agv::Plan::Goal> goals,
    const bool same_map,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)>
      selected_final_destination_cb,
    const std::function<void(const rmf_traffic::agv::Plan::Goal&)> selected_waitpoint_cb)
  {

    auto negotiator = std::make_shared<ChopeNodeNegotiator>();
    negotiator->_context = context;
    negotiator->_goals = std::move(goals);
    negotiator->_selected_final_destination_cb = std::move(selected_final_destination_cb);
    negotiator->_selected_waitpoint_cb = std::move(selected_waitpoint_cb);
    negotiator->_reservation_id = negotiator->_context->last_reservation_request_id();
    negotiator->_reservation_ticket =
      negotiator->_context->node()->location_ticket_obs().observe_on(rxcpp::identity_same_worker(
        negotiator->_context->worker()))
      .subscribe([w =
        negotiator->weak_from_this()](const std::shared_ptr<rmf_chope_msgs::msg::Ticket>
        &msg)
      {

        const auto self = w.lock();
        if (!self)
          return;

        RCLCPP_ERROR(
          self->_context->node()->get_logger(),
          "Got ticket issueing claim");

        if (msg->header.request_id != self->_reservation_id
        || msg->header.robot_name != self->_context->name()
        || msg->header.fleet_name != self->_context->group())
        {
          return;
        }

        self->_ticket = msg;

        // Pick the nearest location to wait
        auto current_location = self->_context->location();
        if (current_location.size() == 0)
        {
          return;
        }

        // Order wait points by the distance from the destination.
        std::vector<std::tuple<double, std::string, rmf_traffic::agv::Plan::Goal>>
          waitpoints_order;
        for (std::size_t wp_idx = 0;
        wp_idx < self->_context->navigation_graph().num_waypoints(); wp_idx++)
        {
          const auto wp = self->_context->navigation_graph().get_waypoint(
            wp_idx);

          auto name = wp.name();
          if (name == nullptr)
          {
            RCLCPP_ERROR(self->_context->node()->get_logger(),
                "Got a parking spot without a name."
                "This parking spot will not be used by the reservation system.");
            continue;
          }

          // Wait at parking spot and check its on same floor.
          if (!wp.is_parking_spot() ||
          wp.get_map_name() != self->_context->map())
          {
            continue;
          }

          auto result =
          self->_context->planner()->quickest_path(current_location, wp_idx);
          if (!result.has_value())
          {
            continue;
          }

          rmf_traffic::agv::Plan::Goal goal(wp_idx);
          waitpoints_order.emplace_back(result->cost(), *name, goal);
        }

        std::sort(waitpoints_order.begin(), waitpoints_order.end(),
        [](const std::tuple<double, std::string, rmf_traffic::agv::Plan::Goal>& a,
          const std::tuple<double, std::string, rmf_traffic::agv::Plan::Goal>& b)
        {
          return std::get<0>(a) < std::get<0>(b);
        });

        // Immediately make claim cause we don't yet support flexible reservations.
        rmf_chope_msgs::msg::ClaimRequest claim_request;
        claim_request.ticket = *msg;
        std::vector<std::string> waitpoints;
        for (auto &[_, waitpoint, waitpoint_goal]: waitpoints_order)
        {
          claim_request.wait_points.push_back(waitpoint);
          self->_waitpoints.push_back(waitpoint_goal);
        }
        self->_context->node()->claim_location_ticket()->publish(claim_request);
        RCLCPP_ERROR(
          self->_context->node()->get_logger(),
          "Claim issued");
      });


    negotiator->_reservation_allocation =
      negotiator->_context->node()->allocated_claims_obs().observe_on(rxcpp::identity_same_worker(
          negotiator->_context->worker()))
      .subscribe([w =
        negotiator->weak_from_this()](const std::shared_ptr<rmf_chope_msgs::msg::ReservationAllocation>
        &msg)
        {
          const auto self = w.lock();
          if (!self)
            return;

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

          if (self->_current_reservation_state ==  ReservationState::Requested)
          {
            while (auto allocation = self->_context->_release_resource())
            {
              rmf_chope_msgs::msg::ReleaseRequest msg;
              std::stringstream str;
              str << self->_context->location()[0].waypoint();
              str >> msg.location;
              msg.ticket = allocation->ticket;
              self->_context->node()->release_location()->publish(msg);
              RCLCPP_ERROR(
                self->_context->node()->get_logger(),
                "Releasing waypoint"
              );
            }
          }

          if (msg->instruction_type
            == rmf_chope_msgs::msg::ReservationAllocation::IMMEDIATELY_PROCEED)
          {
            RCLCPP_INFO(
                self->_context->node()->get_logger(), "chope: Robot %s is going to final destination",
                  self->_context->name().c_str());
            self->_current_reservation_state = ReservationState::RecievedResponseProceedImmediate;
            self->_selected_final_destination_cb(self->_goals[self->_final_allocated_destination.value()->
              satisfies_alternative]);
          }

          if (msg->instruction_type
            == rmf_chope_msgs::msg::ReservationAllocation::WAIT_PERMANENTLY)
          {
            self->_current_reservation_state = ReservationState::RecievedResponceProceedWaitPoint;
            self->_selected_waitpoint_cb(self->_waitpoints[self->_final_allocated_destination.value()->
              satisfies_alternative]);
            RCLCPP_INFO(
                self->_context->node()->get_logger(), "chope: Robot %s is being asked to proceed to a to waitpoint",
                  self->_context->name().c_str());
          }
        });

    RCLCPP_INFO(context->node()->get_logger(),
      "Sending chope request");
    negotiator->make_request(same_map);
    return negotiator;
  }

ChopeNodeNegotiator()
  {

  }
private:
  enum class ReservationState
  {
    Pending=0,
    Requested=1,
    RecievedResponceProceedWaitPoint=2,
    RecievedResponseProceedImmediate=3
  };



  void make_request(bool only_same_map)
  {
    auto current_location = _context->location();
    auto graph = _context->navigation_graph();
    if (current_location.size() == 0)
    {
      //unable to get location. We should return some form of error stste.
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "Robot [%s] can't get location",
        _context->requester_id().c_str());
      return;
    }

    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Selecting a new go_to_place location from [%lu] choices for robot [%s]",
      _goals.size(),
      _context->requester_id().c_str());

    if (_current_reservation_state == ReservationState::Pending)
    {
      // Select node
      rmf_chope_msgs::msg::FlexibleTimeRequest ftr;
      ftr.header.robot_name = _context->name();
      ftr.header.fleet_name = _context->group();
      ftr.header.request_id = _reservation_id;

      auto lowest_cost = std::numeric_limits<double>::infinity();
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

          if (result->cost() < lowest_cost)
          {
            selected_idx = i;
            lowest_cost = result->cost();
          }

          auto name = wp.name();
          if (name == nullptr)
          {
            RCLCPP_ERROR(_context->node()->get_logger(),
                "Got a parking spot without a name."
                "This parking spot will not be used by the reservation system.");
            continue;
          }

          rmf_chope_msgs::msg::FlexibleTimeReservationAlt alternative;
          alternative.resource_name = *name;
          alternative.cost = result->cost();
          alternative.has_end = false;

          rmf_chope_msgs::msg::StartTimeRange start;
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
  std::function<void(const rmf_traffic::agv::Plan::Goal&)> _selected_waitpoint_cb;
  std::function<void(const rmf_traffic::agv::Plan::Goal&)> _selected_final_destination_cb;
  rmf_rxcpp::subscription_guard _reservation_ticket;
  rmf_rxcpp::subscription_guard _reservation_allocation;

  uint64_t _reservation_id = 0;
  std::optional<std::shared_ptr<rmf_chope_msgs::msg::Ticket>> _ticket{std::
    nullopt};
  std::optional<std::shared_ptr<rmf_chope_msgs::msg::ReservationAllocation>>
    _final_allocated_destination{std::nullopt};

  std::vector<rmf_traffic::agv::Plan::Goal> _goals;
  std::vector<rmf_traffic::agv::Plan::Goal> _waitpoints;
};
} // namespace rmf_fleet_adapter
} // namespace

#endif