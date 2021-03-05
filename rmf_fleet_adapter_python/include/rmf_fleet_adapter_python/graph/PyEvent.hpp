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

#ifndef PYEVENT_HPP
#define PYEVENT_HPP

#include <rmf_fleet_adapter/agv/Adapter.hpp>

using Duration = rmf_traffic::Duration;
using Lane = rmf_traffic::agv::Graph::Lane;
using EventPtr = Lane::EventPtr;

// Trampoline rmf_traffic::agv::Graph::Lane::Event wrapper class
// to allow method overrides from Python
class PyEvent : public Lane::Event
{
public:
  // Constructor
  using Lane::Event::Event;

  Duration duration() const override
  {
    PYBIND11_OVERLOAD_PURE(
      Duration,
      Lane::Event,
      duration,  // Trailing comma required to specify no args
    );
  }

  Lane::Executor& execute(Lane::Executor& executor) const override
  {
    PYBIND11_OVERLOAD_PURE(
      Lane::Executor&,
      Lane::Event,
      execute,
      executor
    );
  }

  EventPtr clone() const override
  {
    PYBIND11_OVERLOAD_PURE(
      EventPtr,
      Lane::Event,
      clone,  // Trailing comma required to specify no args
    );
  }
};

#endif // PYEVENT_HPP
