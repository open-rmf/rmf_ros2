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

#ifndef SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP
#define SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP

#include "../Negotiate.hpp"

namespace rmf_fleet_adapter {
namespace services {

namespace detail {
//==============================================================================
std::vector<rmf_utils::clone_ptr<rmf_traffic::agv::NegotiatingRouteValidator>>
make_validators(
    const rmf_traffic::schedule::Negotiator::TableViewerPtr& viewer);
} // namespace detail

//==============================================================================
template<typename Subscriber>
void Negotiate::operator()(const Subscriber& s)
{
  auto validators = detail::make_validators(_viewer);

  std::vector<rmf_utils::clone_ptr<rmf_traffic::agv::NegotiatingRouteValidator>>
      validator_queue;

  _jobs.reserve(_goals.size());


  for (const auto& goal : _goals)
  {
//    auto search = std::make_shared<jobs::Planning>(
//          _planner, _starts, goal, )
  }
}

} // namespace services
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__SERVICES__DETAIL__IMPL_NEGOTIATE_HPP
