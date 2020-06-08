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

#ifndef TEST__THREAD_COOLDOWN_HPP
#define TEST__THREAD_COOLDOWN_HPP

namespace rmf_fleet_adapter_test {

// This is used by multi-threaded tests to have the executable wait some time
// before quitting so that all the threads can wind down before the executable
// exits. Set this to true when running a multi-threaded test. Leave this alone
// when running single-threaded tests.
extern bool thread_cooldown;
} // namespace rmf_fleet_adapter_test

#endif // TEST__THREAD_COOLDOWN_HPP
