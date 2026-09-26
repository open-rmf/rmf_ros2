/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_LOCALIZATION_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_LOCALIZATION_HPP

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace rmf_fleet_adapter {
namespace agv {

class RobotContext;

//==============================================================================
/// Everything a fleet-adapter flavour needs in order to build its own
/// localization request, expressed without naming any flavour's types.
///
/// Shared phase code — `RequestLift`, and anything else that asks a robot to
/// re-localize — fills one of these and hands it to
/// `RobotContext::localize`. Which flavour receives it is decided by whichever
/// `LocalizationHandler` that robot registered, so shared code needs no
/// knowledge of flavours at all and a new flavour needs no change here.
///
/// This exists because the alternative does not scale and failed silently.
/// `RequestLift` used to call a `RobotContext::localize` overload typed on
/// `EasyFullControl::Destination`; a PathGuide robot registered a different
/// slot, so the call found an empty `std::function` and returned `false` —
/// indistinguishable from "this integrator does not implement localization".
/// Post-lift relocalization was skipped for every PathGuide robot from the day
/// the adapter was written until 2026-08-28.
struct LocalizationHandoff
{
  /// Carried rather than recovered via `shared_from_this()`: `localize` is
  /// const, which would yield a `shared_ptr<const RobotContext>` while every
  /// flavour's `make_hold` wants a mutable one.
  std::shared_ptr<RobotContext> context;

  // The `Destination` fields. Every flavour models these identically — same
  // seven members, same order — so they are the neutral currency here.
  std::string map;
  Eigen::Vector3d position;
  std::optional<std::size_t> graph_index;
  std::string name;
  std::optional<double> speed_limit;
  rmf_traffic::agv::Graph::LiftPropertiesPtr lift;
  std::optional<std::string> dock;

  // What every flavour's `CommandExecution::Implementation::make_hold` takes.
  // `expected_finish` and `plan_id` are the reason this is a struct rather
  // than a translation layer over the existing typed slots: a `LocalizationRequest`
  // signature carries neither, so an adapter registered at that level cannot
  // reproduce `RequestLift`'s cumulative_delay bookkeeping and would have to
  // stamp a delay from `now()`, erasing whatever had genuinely accumulated.
  rmf_traffic::Time expected_finish;
  rmf_traffic::PlanId plan_id;
  std::function<void()> finisher;
};

//==============================================================================
/// A flavour's adapter from the neutral handoff to its own integrator
/// callback. Returns true if the request actually reached an integrator, which
/// is what tells `RequestLift` whether to arm its 300 s watchdog.
using LocalizationHandler = std::function<bool(LocalizationHandoff)>;

//==============================================================================
/// Build a flavour's `Destination` from a handoff.
///
/// A template rather than two hand-written call sites, because this is a
/// seven-argument positional aggregate construction over fields that are not
/// distinguishable by type: `map` and `name` are both `std::string`,
/// `graph_index` and `dock` are both optional. Transposing a pair compiles
/// cleanly and hands the integrator a perfectly plausible `Destination`
/// describing somewhere else. Writing it once means the order can be wrong in
/// at most one place, and `test_LocalizationHandoff.cpp` pins that place.
///
/// Instantiated only where the flavour's `Destination::Implementation` is
/// visible — `RobotContext.cpp` for EasyFullControl, `PathGuide.cpp` for
/// PathGuide.
template<typename Destination>
Destination make_localization_destination(const LocalizationHandoff& handoff)
{
  return Destination::Implementation::make(
    handoff.map,
    handoff.position,
    handoff.graph_index,
    handoff.name,
    handoff.speed_limit,
    handoff.lift,
    handoff.dock);
}

//==============================================================================
/// Build a flavour's hold `CommandExecution` from a handoff.
///
/// Consumes `handoff.finisher`, so call it once. Same reasoning as above,
/// though the risk is lower here — the four arguments have distinct types.
template<typename CommandExecution>
CommandExecution make_localization_hold(LocalizationHandoff& handoff)
{
  return CommandExecution::Implementation::make_hold(
    handoff.context,
    handoff.expected_finish,
    handoff.plan_id,
    std::move(handoff.finisher));
}

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_LOCALIZATION_HPP
