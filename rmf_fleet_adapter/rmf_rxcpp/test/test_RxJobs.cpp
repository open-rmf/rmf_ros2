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

#include <rmf_utils/catch.hpp>

#include <rmf_rxcpp/RxJobs.hpp>

struct AsyncCounterAction
{
  int counter = 0;

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w)
  {
    s.on_next(++counter);
    if (counter >= 10)
    {
      s.on_completed();
      return;
    }
    w.schedule([this, s, w](const auto&)
      {
        (*this)(s, w);
      });
  }
};

TEST_CASE("async job", "[Jobs]")
{
  auto action = std::make_shared<AsyncCounterAction>();
  auto j = rmf_rxcpp::make_job<int>(action);
  j.as_blocking().subscribe();
  REQUIRE(action->counter == 10);
}

struct DummyAction
{
  using Result = int;

  int call_count = 0;

  template<typename Subscriber>
  void operator()(const Subscriber& s)
  {
    s.on_next(++call_count);
    s.on_completed();
  }
};

TEST_CASE("make group jobs", "[Jobs]")
{
  std::vector<std::shared_ptr<DummyAction>> actions{
    std::make_shared<DummyAction>(),
    std::make_shared<DummyAction>()
  };
  auto j = rmf_rxcpp::make_job_from_action_list(actions);
  j.as_blocking().subscribe();
  for (const auto& a : actions)
  {
    REQUIRE(a->call_count == 1);
  }
}
