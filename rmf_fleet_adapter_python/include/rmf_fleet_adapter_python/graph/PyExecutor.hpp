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

#ifndef PYEXECUTOR_HPP
#define PYEXECUTOR_HPP

#include <rmf_fleet_adapter/agv/Adapter.hpp>

using Lane = rmf_traffic::agv::Graph::Lane;

// Trampoline rmf_traffic::agv::Graph::Lane::Executor wrapper class
// to allow method overrides from Python
class PyExecutor :
  public Lane::Executor
{
public:
  // Constructor
  using Lane::Executor::Executor;

  void execute(const Lane::DoorOpen& open) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "door_open_execute",
      execute,
      open
    );
  }

  void execute(const Lane::DoorClose& close) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "door_close_execute",
      execute,
      close
    );
  }

  void execute(const Lane::LiftSessionBegin& begin) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "lift_session_begin_execute",
      execute,
      begin
    );
  }

  void execute(const Lane::LiftDoorOpen& open) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "lift_door_open_execute",
      execute,
      open
    );
  }

  void execute(const Lane::LiftSessionEnd& end) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "lift_session_end_execute",
      execute,
      end
    );
  }

  void execute(const Lane::LiftMove& move) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "lift_move_execute",
      execute,
      move
    );
  }

  void execute(const Lane::Dock& dock) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "dock_execute",
      execute,
      dock
    );
  }

  void execute(const Lane::Wait& wait) override
  {
    PYBIND11_OVERLOAD_PURE_NAME(
      void,
      Lane::Executor,
      "wait_execute",
      execute,
      wait
    );
  }
};

#endif // PYEXECUTOR_HPP
