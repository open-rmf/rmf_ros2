/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "Negotiator.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
std::shared_ptr<Negotiator> Negotiator::make(
  agv::RobotContextPtr context,
  Respond respond)
{
  auto negotiator = std::make_shared<Negotiator>();
  negotiator->_context = std::move(context);
  negotiator->_respond = std::move(respond);
  negotiator->claim_license();

  return negotiator;
}

//==============================================================================
void Negotiator::respond(
  const TableViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  auto service = _respond(table_viewer, responder);
  if (!service)
  {
    // If we didn't get a service back then we will trust that the response was
    // already submitted upstream.
    return;
  }

  auto negotiate_sub =
    rmf_rxcpp::make_job<services::Negotiate::Result>(service)
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [w = weak_from_this()](const auto& result)
    {
      if (auto self = w.lock())
      {
        result.respond();
        self->_negotiate_services.erase(result.service);
      }
      else
      {
        // We need to make sure we respond in some way so that we don't risk
        // making a negotiation hang forever. If this task is dead, then we
        // should at least respond by forfeiting.
        const auto service = result.service;
        const auto responder = service->responder();
        responder->forfeit({});
      }
    });

  using namespace std::chrono_literals;
  const auto wait_duration = 2s + table_viewer->sequence().back().version * 10s;
  auto negotiation_timer = _context->node()->try_create_wall_timer(
    wait_duration,
    [s = service->weak_from_this()]
    {
      if (const auto service = s.lock())
        service->interrupt();
    });

  _negotiate_services[service] = NegotiationManagers{
    std::move(negotiate_sub),
    std::move(negotiation_timer)
  };
}

//==============================================================================
services::ProgressEvaluator Negotiator::make_evaluator(
  const TableViewerPtr& table_viewer)
{
  services::ProgressEvaluator evaluator;
  if (table_viewer->parent_id())
  {
    const auto& s = table_viewer->sequence();
    assert(s.size() >= 2);
    evaluator.compliant_leeway_base *= s[s.size()-2].version + 1;
    evaluator.max_cost_threshold = 90.0 + 30.0*s[s.size()-2].version;
  }

  return evaluator;
}

//==============================================================================
void Negotiator::clear_license()
{
  _license = nullptr;
}

//==============================================================================
void Negotiator::claim_license()
{
  _license = _context->set_negotiator(this);
}

} // namespace rmf_fleet_adapter
