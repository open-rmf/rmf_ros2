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

#include "internal_FleetUpdateHandle.hpp"
#include "internal_RobotUpdateHandle.hpp"
#include "RobotContext.hpp"

#include "../tasks/Delivery.hpp"

namespace rmf_fleet_adapter {
namespace agv {

namespace {
//==============================================================================
class LiaisonNegotiator : public rmf_traffic::schedule::Negotiator
{
public:

  LiaisonNegotiator(
      std::shared_ptr<rmf_traffic::schedule::Negotiator> negotiator)
    : w_negotiator(negotiator)
  {
    // Do nothing
  }

  std::weak_ptr<rmf_traffic::schedule::Negotiator> w_negotiator;

  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final
  {
    const auto negotiator = w_negotiator.lock();
    if (!negotiator)
    {
      // If we no longer have access to the upstream negotiator, then we simply
      // forfeit.
      //
      // TODO(MXG): Consider issuing a warning here
      return responder->forfeit({});
    }

    negotiator->respond(table_viewer, responder);
  }

};
} // anonymous namespace

//==============================================================================
auto FleetUpdateHandle::Implementation::estimate_delivery(
    const rmf_task_msgs::msg::Delivery& request) const
-> rmf_utils::optional<FleetUpdateHandle::Implementation::DeliveryEstimate>
{
  const auto& graph = planner->get_configuration().graph();
  const auto pickup_wp = graph.find_waypoint(request.pickup_place_name);
  if (!pickup_wp)
    return rmf_utils::nullopt;

  const auto dropoff_wp = graph.find_waypoint(request.dropoff_place_name);
  if (!dropoff_wp)
    return rmf_utils::nullopt;

  const auto pickup_goal = rmf_traffic::agv::Plan::Goal(pickup_wp->index());
  const auto dropoff_goal = rmf_traffic::agv::Plan::Goal(dropoff_wp->index());

  // TODO(MXG): At some point we should consider parallelizing this estimation
  // process and taking the existing schedule into account, but for now we'll
  // try to use a very quick rough estimate.
  DeliveryEstimate best;
  for (const auto& element : task_managers)
  {
    const auto& mgr = element.second;
    auto start = mgr.expected_finish_location();
    const auto pickup_plan = planner->plan(start, pickup_goal);
    if (!pickup_plan)
      continue;

    assert(pickup_plan->get_waypoints().back().graph_index());
    const auto dropoff_start = rmf_traffic::agv::Plan::Start(
          pickup_plan->get_waypoints().back().time(),
          *pickup_plan->get_waypoints().back().graph_index(),
          pickup_plan->get_waypoints().back().position()[2]);

    const auto dropoff_plan = planner->plan(dropoff_start, dropoff_goal);
    if (!dropoff_plan)
      continue;

    const auto& final_wp = dropoff_plan->get_waypoints().back();

    const auto estimate = final_wp.time();
    rmf_traffic::agv::Plan::Start finish{
      estimate,
      *final_wp.graph_index(),
      final_wp.position()[2]
    };

    if (estimate < best.time)
    {
      best = DeliveryEstimate{
        estimate,
        element.first,
        std::move(start.front()),
        std::move(dropoff_start),
        std::move(finish)
      };
    }
  }

  if (best.robot)
    return best;

  return rmf_utils::nullopt;
}

//==============================================================================
void FleetUpdateHandle::Implementation::perform_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const DeliveryEstimate& estimate)
{
  auto& mgr = task_managers.at(estimate.robot);
  mgr.queue_task(
        tasks::make_delivery(
          request, estimate.robot,
          *estimate.pickup_start,
          *estimate.dropoff_start),
        *estimate.finish);
}

//==============================================================================
void FleetUpdateHandle::add_robot(
    std::shared_ptr<RobotCommandHandle> command,
    const std::string& name,
    const rmf_traffic::Profile& profile,
    rmf_traffic::agv::Plan::StartSet start,
    std::function<void(std::shared_ptr<RobotUpdateHandle>)> handle_cb)
{
  rmf_traffic::schedule::ParticipantDescription description(
        name,
        _pimpl->name,
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        profile);

  _pimpl->writer->async_make_participant(
        std::move(description),
        [command = std::move(command),
         start = std::move(start),
         handle_cb = std::move(handle_cb),
         fleet = shared_from_this()](
        rmf_traffic::schedule::Participant participant)
  {
    auto context = std::make_shared<RobotContext>(
          RobotContext{
            std::move(command),
            std::move(start),
            std::move(participant),
            fleet->_pimpl->snappable,
            fleet->_pimpl->planner,
            fleet->_pimpl->node,
            fleet->_pimpl->worker
          });

    // TODO(MXG): We need to perform this test because we do not currently
    // support the distributed negotiation in unit test environments. We should
    // create an abstract NegotiationRoom interface in rmf_traffic and use that
    // instead.
    if (fleet->_pimpl->negotiation)
    {
      context->_negotiation_license =
          fleet->_pimpl->negotiation
          ->register_negotiator(
            context->itinerary().id(),
            std::make_unique<LiaisonNegotiator>(context));
    }

    fleet->_pimpl->task_managers.insert({context, context});
    return RobotUpdateHandle::Implementation::make(std::move(context));
  });
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_delivery_requests(
    AcceptDeliveryRequest check)
{
  _pimpl->accept_delivery = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle::FleetUpdateHandle()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_fleet_adapter
