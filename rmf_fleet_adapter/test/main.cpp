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


#define CATCH_CONFIG_RUNNER
#include <rmf_utils/catch.hpp>

#include <thread>

#include "thread_cooldown.hpp"

namespace rmf_fleet_adapter_test {
bool thread_cooldown;
} // namespace rmf_fleet_adapter_test

int main(int argc, char* argv[])
{
  rmf_fleet_adapter_test::thread_cooldown = false;
  const int result = Catch::Session().run(argc, argv);

  if (rmf_fleet_adapter_test::thread_cooldown)
  {
    using namespace std::chrono_literals;
#ifdef NDEBUG
    const auto cooldown_time = 100ms;
#else
    const auto cooldown_time = 2s;
#endif

    std::this_thread::sleep_for(cooldown_time);
  }

  return result;
}
