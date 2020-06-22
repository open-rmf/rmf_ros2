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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_ROLLOUT_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_ROLLOUT_HPP

#include "../Rollout.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
template<typename Subscriber, typename Worker>
void Rollout::operator()(const Subscriber& s, const Worker&)
{
  s.on_next(Result{_rollout.expand(_blocker, _span, _max_rollouts)});
  s.on_completed();
}

} // namespace jobs
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__DETAIL__IMPL_ROLLOUT_HPP
