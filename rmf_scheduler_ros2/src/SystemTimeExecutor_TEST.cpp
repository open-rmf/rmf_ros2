/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include "SystemTimeExecutor.hpp"

#include <chrono>
#include <future>
#include <thread>

namespace rmf::scheduler::test {

TEST_CASE("Execute tasks")
{
  SystemTimeExecutor executor;
  std::thread t{[&executor]() { executor.spin(); }};

  std::promise<bool> ran;
  executor.schedule_task(executor.now() + 1, [&]() { ran.set_value(true); });

  auto fut = ran.get_future();
  REQUIRE(fut.wait_for(
      std::chrono::seconds{3}) == std::future_status::ready);
  REQUIRE(fut.get());

  executor.stop();
  t.join();
}

TEST_CASE("Stress test", "[.][slow]")
{
  SystemTimeExecutor executor;
  std::thread t{[&executor]() { executor.spin(); }};

  int64_t start = executor.now();
  int64_t end = start + 10;

  while (executor.now() < end)
  {
    executor.schedule_task(executor.now(), []() {});
  }

  executor.stop();
  t.join();
}

}
