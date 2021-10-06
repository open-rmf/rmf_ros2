/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "../../src/rmf_task_ros2/scheduler/internal_ScheduleQueue.hpp"
#include <rmf_utils/catch.hpp>

using namespace rmf_task_ros2;

//==============================================================================
rclcpp::Duration operator*(const int num, const rclcpp::Duration dur)
{
  // LAZY HACK!!
  rclcpp::Duration result(0,0);
  for(int i = 0; i < num; i++)
  {
    result = result + dur;
  }
  return result;
}

//==============================================================================
SCENARIO("Enqueue one task with fixed time scale that repeats every second")
{
  SimpleSchedulerQueue queue;

  auto time_start = rclcpp::Time();

  auto seconds = rclcpp::Duration(1,0);
  
  TaskRequest task_request1;
  task_request1._start_time = time_start + 10*seconds;
  task_request1._end_time = time_start + 13*seconds;
  task_request1._repeat_interval = seconds;

  auto id = queue.enqueue_request(task_request1);

  WHEN("we step through the first time step")
  {
    auto pending = queue.check_and_deque(time_start);
    THEN("Nothing should be returned")
    {
      REQUIRE(pending.size() == 0);
    }
  }

  WHEN("we step through to T+11")
  {
    auto pending = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending.size() == 1);
      REQUIRE(queue.get_queued().size() == 1);
    }
  }

  WHEN("we step through to T+12")
  {
    auto pending = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be two pending tasks")
    {
      REQUIRE(queue.get_queued().size() == 1);
      REQUIRE(pending.size() == 2);
    }
  }

  WHEN("we step through to T+15")
  {
    auto pending = queue.check_and_deque(time_start+15*seconds);
    THEN("There should be 3 pending tasks and no tasks queued")
    {
      REQUIRE(queue.get_queued().size() == 0);
      REQUIRE(pending.size() == 3);
    }
  }

  WHEN("we step through T+11 and T+12")
  {
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    auto pending2 = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending1.size() == 1);
      REQUIRE(pending2.size() == 1);
    }
  }

  WHEN("We cancel before dequeing")
  {
    queue.cancel_request(id);
    THEN("State should immediately be updated")
    {
      REQUIRE(queue.get_queued().size() == 0);
    }
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be no pending tasks")
    {
      REQUIRE(pending1.size() == 0);
      REQUIRE(queue.get_queued().size() == 0);
    }
  }

  WHEN("we cancel after T+11")
  {
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    queue.cancel_request(id);
    auto pending2 = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending1.size() == 1);
      REQUIRE(pending2.size() == 0);
    }
  }
}

//==============================================================================
SCENARIO(
  "Enqueue one task with fixed time scale that repeats every second and never ends")
{
  SimpleSchedulerQueue queue;

  auto time_start = rclcpp::Time();

  auto seconds = rclcpp::Duration(1,0);
  
  TaskRequest task_request1;
  task_request1._start_time = time_start + 10*seconds;
  task_request1._repeat_interval = seconds;

  auto id = queue.enqueue_request(task_request1);

  WHEN("we step through the first time step")
  {
    auto pending = queue.check_and_deque(time_start);
    THEN("Nothing should be returned")
    {
      REQUIRE(pending.size() == 0);
    }
  }

  WHEN("we step through to T+11")
  {
    auto pending = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending.size() == 1);
      REQUIRE(queue.get_queued().size() == 1);
    }
  }

  WHEN("we step through to T+12")
  {
    auto pending = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be two pending tasks")
    {
      REQUIRE(queue.get_queued().size() == 1);
      REQUIRE(pending.size() == 2);
    }
  }

  WHEN("we step through to T+15")
  {
    auto pending = queue.check_and_deque(time_start+15*seconds);
    THEN("There should be 3 pending tasks and no tasks queued")
    {
      REQUIRE(queue.get_queued().size() == 1);
      REQUIRE(pending.size() == 5);
    }
  }

  WHEN("we step through T+11 and T+12")
  {
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    auto pending2 = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending1.size() == 1);
      REQUIRE(pending2.size() == 1);
    }
  }

  WHEN("We cancel before dequeing")
  {
    queue.cancel_request(id);
    THEN("State should immediately be updated")
    {
      REQUIRE(queue.get_queued().size() == 0);
    }
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be no pending tasks")
    {
      REQUIRE(pending1.size() == 0);
      REQUIRE(queue.get_queued().size() == 0);
    }
  }
}

//==============================================================================
SCENARIO(
  "Enqueue one task with fixed time scale that does not repeat")
{
  SimpleSchedulerQueue queue;

  auto time_start = rclcpp::Time();

  auto seconds = rclcpp::Duration(1,0);
  
  TaskRequest task_request1;
  task_request1._start_time = time_start + 10*seconds;
  task_request1._repeat_interval = std::nullopt;

  auto id = queue.enqueue_request(task_request1);

  WHEN("we step through the first time step")
  {
    auto pending = queue.check_and_deque(time_start);
    THEN("Nothing should be returned")
    {
      REQUIRE(pending.size() == 0);
    }
  }

  WHEN("we step through to T+11")
  {
    auto pending = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending.size() == 1);
      REQUIRE(queue.get_queued().size() == 1);
    }
  }

  WHEN("we step through to T+12")
  {
    auto pending = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be two pending tasks")
    {
      REQUIRE(queue.get_queued().size() == 1);
      REQUIRE(pending.size() == 1);
    }
  }

  WHEN("we step through to T+15")
  {
    auto pending = queue.check_and_deque(time_start+15*seconds);
    THEN("There should be 3 pending tasks and no tasks queued")
    {
      REQUIRE(queue.get_queued().size() == 1);
      REQUIRE(pending.size() == 1);
    }
  }

  WHEN("we step through T+11 and T+12")
  {
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    auto pending2 = queue.check_and_deque(time_start+12*seconds);
    THEN("There should be one pending task")
    {
      REQUIRE(pending1.size() == 1);
      REQUIRE(pending2.size() == 0);
    }
  }

  WHEN("We cancel before dequeing")
  {
    queue.cancel_request(id);
    THEN("State should immediately be updated")
    {
      REQUIRE(queue.get_queued().size() == 0);
    }
    auto pending1 = queue.check_and_deque(time_start+11*seconds);
    THEN("There should be no pending tasks")
    {
      REQUIRE(pending1.size() == 0);
      REQUIRE(queue.get_queued().size() == 0);
    }
  }
}

//==============================================================================
SCENARIO("Empty Queue")
{
  SimpleSchedulerQueue queue;
  WHEN("getting queued elements")
  {
    THEN("returned queue is empty")
    {
      auto state = queue.get_queued();

      REQUIRE(state.size() == 0);
    }
  }

  WHEN("Checking and dequeueing")
  {
    THEN("return empty list")
    {
      auto time_start = rclcpp::Time();
      auto queued = queue.check_and_deque(time_start);

      REQUIRE(queued.size() == 0);
    }
  }
}