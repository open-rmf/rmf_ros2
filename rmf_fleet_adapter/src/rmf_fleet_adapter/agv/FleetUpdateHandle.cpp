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
#include "../tasks/Loop.hpp"

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

    const auto& pickup_plan_end = pickup_plan->get_waypoints().back();
    assert(pickup_plan_end.graph_index());
    const auto dropoff_start = rmf_traffic::agv::Plan::Start(
          pickup_plan_end.time(),
          *pickup_plan_end.graph_index(),
          pickup_plan_end.position()[2]);

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
          request,
          estimate.robot,
          *estimate.pickup_start,
          *estimate.dropoff_start),
        *estimate.finish);
}

//==============================================================================
auto FleetUpdateHandle::Implementation::estimate_loop(
    const rmf_task_msgs::msg::Loop& request) const
-> rmf_utils::optional<LoopEstimate>
{
  if (request.robot_type != name)
    return rmf_utils::nullopt;

  const std::size_t n = request.num_loops;
  if (n == 0)
    return rmf_utils::nullopt;

  const auto& graph = planner->get_configuration().graph();
  const auto loop_start_wp = graph.find_waypoint(request.start_name);
  if (!loop_start_wp)
    return rmf_utils::nullopt;

  const auto loop_end_wp = graph.find_waypoint(request.finish_name);
  if (!loop_end_wp)
    return rmf_utils::nullopt;

  const auto loop_start_goal =
      rmf_traffic::agv::Plan::Goal(loop_start_wp->index());

  const auto loop_end_goal =
      rmf_traffic::agv::Plan::Goal(loop_end_wp->index());

  LoopEstimate best;
  for (const auto& element : task_managers)
  {
    LoopEstimate estimate;
    estimate.robot = element.first;

    const auto& mgr = element.second;
    auto start = mgr.expected_finish_location();
    const auto loop_init_plan = planner->plan(start, loop_start_goal);
    if (!loop_init_plan)
      continue;

    rmf_traffic::Duration init_duration = std::chrono::seconds(0);
    if (loop_init_plan->get_waypoints().size() > 1)
    {
      // If loop_init_plan is not empty, then that means we are not starting at
      // the starting point of the loop. Therefore we will need an initial plan
      // to reach the first point in the loop.
      estimate.init_start = start.front();

      init_duration =
          loop_init_plan->get_waypoints().back().time()
          - loop_init_plan->get_waypoints().front().time();
    }

    const auto loop_forward_start = [&]() -> rmf_traffic::agv::Plan::StartSet
    {
      if (loop_init_plan->get_waypoints().empty())
        return start;

      const auto& loop_init_wp = loop_init_plan->get_waypoints().back();
      assert(loop_init_wp.graph_index());
      return {rmf_traffic::agv::Plan::Start(
            loop_init_wp.time(),
            *loop_init_wp.graph_index(),
            loop_init_wp.position()[2])};
    }();

    const auto loop_forward_plan =
        planner->plan(loop_forward_start, loop_end_goal);
    if (!loop_forward_plan)
      continue;

    // If the forward plan is empty then that means the start and end of the
    // loop are the same, making it a useless request.
    // TODO(MXG): We should probably make noise here instead of just ignoring
    // the request.
    if (loop_forward_plan->get_waypoints().empty())
      return rmf_utils::nullopt;

    estimate.loop_start = loop_forward_start.front();

    const auto loop_duration =
        loop_forward_plan->get_waypoints().back().time()
        - loop_forward_plan->get_waypoints().front().time();

    // We only need to provide this if there is supposed to be more than one
    // loop.
    const auto& final_wp = loop_forward_plan->get_waypoints().back();
    assert(final_wp.graph_index());
    estimate.loop_end = rmf_traffic::agv::Plan::Start{
      final_wp.time(),
      *final_wp.graph_index(),
      final_wp.position()[2]
    };

    const auto start_time = [&]()
    {
      if (loop_init_plan->get_waypoints().empty())
        return loop_forward_plan->get_waypoints().front().time();

      return loop_init_plan->get_waypoints().front().time();
    }();

    estimate.time =
        start_time + init_duration + (2*n - 1)*loop_duration;

    estimate.loop_end->time(estimate.time);

    if (estimate.time < best.time)
      best = std::move(estimate);
  }

  if (best.robot)
    return best;

  return rmf_utils::nullopt;
}

//==============================================================================
void FleetUpdateHandle::Implementation::perform_loop(
    const rmf_task_msgs::msg::Loop& request,
    const LoopEstimate& estimate)
{
  auto& mgr = task_managers.at(estimate.robot);
  mgr.queue_task(
        tasks::make_loop(
          request,
          estimate.robot,
          estimate.init_start,
          *estimate.loop_start,
          estimate.loop_end),
        *estimate.loop_end);
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
        [worker = _pimpl->worker,
         command = std::move(command),
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

    // We schedule the following operations on the worker to make sure we do not
    // have a multiple read/write race condition on the FleetUpdateHandle.
    worker.schedule(
          [context, fleet, node = fleet->_pimpl->node,
           handle_cb = std::move(handle_cb)](const auto&)
    {
      // TODO(MXG): We need to perform this test because we do not currently
      // support the distributed negotiation in unit test environments. We
      // should create an abstract NegotiationRoom interface in rmf_traffic and
      // use that instead.
      if (fleet->_pimpl->negotiation)
      {
        context->_negotiation_license =
            fleet->_pimpl->negotiation
            ->register_negotiator(
              context->itinerary().id(),
              std::make_unique<LiaisonNegotiator>(context));
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Added a robot named [%s] with participant ID [%d]",
        context->name().c_str(), context->itinerary().id());

      fleet->_pimpl->task_managers.insert({context, context});
      if (handle_cb)
      {
        handle_cb(RobotUpdateHandle::Implementation::make(std::move(context)));
      }
      else
      {
        RCLCPP_WARN(
          node->get_logger(),
          "FleetUpdateHandle::add_robot(~) was not provided a callback to "
          "receive the RobotUpdateHandle of the new robot. This means you will "
          "not be able to update the state of the new robot. This is likely to "
          "be a fleet adapter development error.");
      }
    });
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

//==============================================================================
void request_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets)
{
  FleetUpdateHandle::Implementation::DeliveryEstimate best;
  FleetUpdateHandle::Implementation* chosen_fleet = nullptr;

  for (auto& fleet : fleets)
  {
    auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
    if (!fimpl.accept_delivery || !fimpl.accept_delivery(request))
      continue;

    const auto estimate = fimpl.estimate_delivery(request);
    if (!estimate)
      continue;

    if (estimate->time < best.time)
    {
      best = *estimate;
      chosen_fleet = &fimpl;
    }
  }

  if (!chosen_fleet)
    return;

  chosen_fleet->perform_delivery(request, best);
}

//==============================================================================
void request_loop(
    const rmf_task_msgs::msg::Loop& request,
    const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets)
{
  FleetUpdateHandle::Implementation::LoopEstimate best;
  FleetUpdateHandle::Implementation* chosen_fleet = nullptr;

  for (auto& fleet : fleets)
  {
    auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
    const auto estimate = fimpl.estimate_loop(request);
    if (!estimate)
      continue;

    if (estimate->time < best.time)
    {
      best = *estimate;
      chosen_fleet = &fimpl;
    }
  }

  if (!chosen_fleet)
    return;

  chosen_fleet->perform_loop(request, best);
}


} // namespace agv
} // namespace rmf_fleet_adapter
