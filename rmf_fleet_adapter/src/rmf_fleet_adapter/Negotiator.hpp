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

#ifndef SRC__RMF_FLEET_ADAPTER__NEGOTIATOR_HPP
#define SRC__RMF_FLEET_ADAPTER__NEGOTIATOR_HPP

#include <rmf_traffic/schedule/Negotiator.hpp>

#include "agv/RobotContext.hpp"
#include "services/Negotiate.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
class Negotiator
  : public rmf_traffic::schedule::Negotiator,
  public std::enable_shared_from_this<Negotiator>
{
public:

  using NegotiatePtr = std::shared_ptr<services::Negotiate>;
  using Respond =
    std::function<NegotiatePtr(const TableViewerPtr&, const ResponderPtr&)>;

  static std::shared_ptr<Negotiator> make(
    agv::RobotContextPtr context,
    Respond respond);

  void respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) final;

  static services::ProgressEvaluator make_evaluator(
    const TableViewerPtr& table_viewer);

  void clear_license();

  void claim_license();

private:

  struct NegotiationManagers
  {
    rmf_rxcpp::subscription_guard subscription;
    rclcpp::TimerBase::SharedPtr timer;
  };
  using NegotiateServiceMap =
    std::unordered_map<NegotiatePtr, NegotiationManagers>;
  NegotiateServiceMap _negotiate_services;

  agv::RobotContextPtr _context;
  std::shared_ptr<void> _license;
  Respond _respond;
};

} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__NEGOTIATOR_HPP
