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
    const FleetUpdateHandle& fleet,
    const rmf_task_msgs::msg::Delivery& request)
-> DeliveryEstimate
{

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

    fleet->_pimpl->robots.push_back(context);
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
