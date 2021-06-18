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

#ifndef RMF_RXCPP__RXJOBS_HPP
#define RMF_RXCPP__RXJOBS_HPP

#include <rmf_rxcpp/detail/RxJobsDetail.hpp>
#include <rxcpp/rx.hpp>
#include <memory>
#include <cassert>

namespace rmf_rxcpp {

template<typename T, typename Action>
inline auto make_job(const std::shared_ptr<Action>& action)
{
  return detail::make_observable<T>(action);
}

template<typename Job0, typename... Jobs>
inline auto merge_jobs(const Job0& o0, Jobs&& ... os)
{
  return o0.merge(rxcpp::serialize_event_loop(), os...);
}

template<typename ActionsIterable>
inline auto make_job_from_action_list(const ActionsIterable& actions)
{
  using Action =
    typename std::iterator_traits<decltype(actions.begin())>::value_type::
    element_type;
  return detail::make_merged_observable<typename Action::Result>(actions);
}

struct subscription_guard
{
  subscription_guard(rxcpp::subscription s = rxcpp::subscription())
  : _subscription(std::move(s))
  {
    // Do nothing
  }

  subscription_guard(const subscription_guard&) = delete;
  subscription_guard(subscription_guard&& other)
  {
    _subscription = std::move(other._subscription);
    other._subscription = rxcpp::subscription();
    assert(!other._subscription.is_subscribed());
  }

  subscription_guard& operator=(const subscription_guard&) = delete;
  subscription_guard& operator=(subscription_guard&& other)
  {
    _subscription = std::move(other._subscription);
    other._subscription = rxcpp::subscription();
    assert(!other._subscription.is_subscribed());
    return *this;
  }

  rxcpp::subscription& get()
  {
    return _subscription;
  }

  const rxcpp::subscription& get() const
  {
    return _subscription;
  }

  ~subscription_guard()
  {
    if (_subscription.is_subscribed())
      _subscription.unsubscribe();
  }

private:
  rxcpp::subscription _subscription;
};

} // namespace rmf_rxcpp

#endif //RMF_RXCPP__RXJOBS_HPP
