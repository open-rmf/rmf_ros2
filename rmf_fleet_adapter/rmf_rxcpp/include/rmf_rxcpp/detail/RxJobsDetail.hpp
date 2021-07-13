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

#ifndef RMF_RXCPP__RXJOBSDETAIL_HPP
#define RMF_RXCPP__RXJOBSDETAIL_HPP

#include <rxcpp/rx.hpp>

namespace rmf_rxcpp {
namespace detail {
template<typename Action, typename Subscriber>
using IsAsyncAction = std::is_constructible<
  std::function<void(Subscriber, rxcpp::schedulers::worker)>,
  std::reference_wrapper<std::remove_reference_t<Action>>>;

template<typename Action, typename Subscriber>
void schedule_job(
  const std::weak_ptr<Action>& a,
  const Subscriber& s,
  const rxcpp::schedulers::worker& w,
  typename std::enable_if_t<IsAsyncAction<Action, Subscriber>::value>* = 0)
{
  w.schedule(
    [a, s, w](const auto&)
    {
      if (const auto action = a.lock())
        (*action)(s, w);
    });
}

template<typename Action, typename Subscriber>
void schedule_job(
  const std::weak_ptr<Action>& a,
  const Subscriber& s,
  const rxcpp::schedulers::worker& w,
  typename std::enable_if_t<!IsAsyncAction<Action, Subscriber>::value>* = 0)
{
  w.schedule(
    [a, s](const auto&)
    {
      if (const auto action = a.lock())
        (*action)(s);
    });
}

inline auto get_event_loop()
{
  static auto event_loop = rxcpp::schedulers::make_event_loop();
  return event_loop;
}

/**
 * Creates an observable from a job, the observable runs the job in an event loop until it has
 * completed or cancelled. Each progress update on a job is queued at the back of the event loop
 * so that other jobs has a chance to start before earlier jobs are finished.
 *
 * @tparam Action
 * @tparam Result
 * @param action
 * @return
 */
template<typename T, typename Action>
auto make_observable(const std::shared_ptr<Action>& action)
{
  return rxcpp::observable<>::create<T>(
    [a = std::weak_ptr<Action>(action)](const auto& s)
    {
      auto worker = get_event_loop().create_worker();
      detail::schedule_job(a, s, worker);
    });
}

/// Alternative to make_observable that is unconcerned about memory leaks
template<typename T, typename Action>
auto make_leaky_observable(const std::shared_ptr<Action>& action)
{
  return rxcpp::observable<>::create<T>(
    [a = std::move(action)](const auto& s)
    {
      auto worker = get_event_loop().create_worker();
      detail::schedule_job(a, s, worker);
    });
}

template<typename T, typename ActionsIterable>
auto make_merged_observable(const ActionsIterable& actions)
{
  // needed to prevent dynamic observables, which reduces performance
  using Observable = decltype(detail::make_observable<T>(*actions.begin()));

  return rxcpp::observable<>::create<Observable>([&actions](const auto& s)
      {
        for (const auto& a : actions)
          s.on_next(detail::make_observable<T>(a));
        s.on_completed();
      }).merge(rxcpp::serialize_event_loop());
}

} // namespace detail
} // namespace rmf_rxcpp

#endif //RMF_RXCPP__RXJOBSDETAIL_HPP
