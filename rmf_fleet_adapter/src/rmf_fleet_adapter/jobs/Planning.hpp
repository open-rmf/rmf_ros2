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

#ifndef SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP
#define SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP

#include <rmf_rxcpp/RxJobs.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/schedule/Snapshot.hpp>

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
/// A job to advance a planning effort forward
class Planning : public std::enable_shared_from_this<Planning>
{
public:
  struct Result
  {
    std::shared_ptr<Planning> job;
  };

  Planning(
    std::shared_ptr<const rmf_traffic::agv::Planner> planner,
    const rmf_traffic::agv::Plan::StartSet& starts,
    rmf_traffic::agv::Plan::Goal goal,
    rmf_traffic::agv::Plan::Options options);

  Planning(rmf_traffic::agv::Planner::Result _setup);

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w);

  void resume();

  void discard();

  bool active() const;

  rmf_traffic::agv::Planner::Result& progress();

  const rmf_traffic::agv::Planner::Result& progress() const;

private:
  mutable std::mutex _resume_mutex;
  std::function<void()> _resume;
  rmf_utils::optional<rmf_traffic::agv::Planner::Result> _current_result;

  std::unique_lock<std::mutex> _lock_resume() const;
};

} // namespace jobs
} // namespace rmf_fleet_adapter

#include "detail/impl_Planning.hpp"

#endif // SRC__RMF_FLEET_ADAPTER__JOBS__PLANNINGJOB_HPP
