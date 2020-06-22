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

TEST_CASE("run simple job", "[Jobs]")
{
  bool ran = false;
  auto j = rmf_rxcpp::make_leaky_job<int>([](const auto& s)
  {
    s.on_next(1);
    s.on_completed();
  });
  j.as_blocking().subscribe([&ran](const auto&) { ran = true; });
  REQUIRE(ran);
}

TEST_CASE("run multiple jobs in parallel", "[Jobs]")
{
  bool job1_success = false;
  bool job2_success = false;
  auto job1 = rmf_rxcpp::make_leaky_job<int>([&job1_success, &job2_success](const auto& s) {
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < timeout)
    {
      if (job2_success)
        break;
    }
    job1_success = job2_success;
    s.on_completed();
  });
  auto job2 = rmf_rxcpp::make_leaky_job<int>([&job2_success](const auto& s) {
    job2_success = true;
    s.on_completed();
  });
  auto job3 = rmf_rxcpp::merge_jobs(job1, job2);

  job3.as_blocking().subscribe();
  REQUIRE(job1_success);
}

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

TEST_CASE("job completion handler is called", "[Jobs]")
{
  bool called = false;
  auto j = rmf_rxcpp::make_leaky_job<int>([](const auto& s)
  {
    s.on_completed();
  });
  j.as_blocking().subscribe([](const auto&) {}, [&called]()
  {
    called = true;
  });
  REQUIRE(called);
}

TEST_CASE("nested job", "[Jobs]")
{
  auto job1 = rmf_rxcpp::make_leaky_job<std::string>([](const auto& s)
  {
    auto job2 = rmf_rxcpp::make_leaky_job<std::string>([](const auto& s)
    {
      s.on_next("hello");
      s.on_completed();
    });

    job2.subscribe([s](const auto& val)
    {
      s.on_next(val + " world");
      s.on_completed();
    });
  });
  job1.as_blocking().subscribe([](const auto& val)
  {
    REQUIRE(val == "hello world");
  });
}

TEST_CASE("cancelling job", "[Jobs]")
{
  int counter = 0;
  auto j = rmf_rxcpp::make_leaky_job<int>([&counter](const auto& s)
  {
    for (int i = 1; i <= 10; i++)
    {
      if (!s.is_subscribed())
        break;
      counter = i;
      s.on_next(i);
    }
    s.on_completed();
  });
  rxcpp::composite_subscription subscription{};
  j.as_blocking().subscribe(subscription, [&subscription](int i)
  {
    if (i >= 5)
      subscription.unsubscribe();
  });
  REQUIRE(counter == 5);
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
