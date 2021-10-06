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

#ifndef RMF_TASK_ROS2_SCHEDULER_INTERNAL_SCHEDULE_QUEUE_H
#define RMF_TASK_ROS2_SCHEDULER_INTERNAL_SCHEDULE_QUEUE_H

#include "internal_TaskRequest.hpp"

#include <chrono>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace rmf_task_ros2 {

class AbstractSchedulerQueue
{
public:
  virtual uint32_t enqueue_request(const TaskRequest& request) = 0;

  virtual void cancel_request(const uint32_t request) = 0;

  // TODO: Change to some type of Custom View to make it more efficient
  virtual std::unordered_map<uint32_t, TaskRequest> get_queued() const = 0;

  virtual ~AbstractSchedulerQueue() { };
};


class SimpleSchedulerQueue : AbstractSchedulerQueue
{
public:
  uint32_t enqueue_request(const TaskRequest& request) override;

  void cancel_request(const uint32_t request) override;

  std::unordered_map<uint32_t, TaskRequest> get_queued() const override;

  std::vector<TaskRequest> check_and_deque(const rclcpp::Time time);

private:
  uint32_t _increment_id;
  
  std::unordered_map<uint32_t, TaskRequest> _request_table;
  std::unordered_set<uint32_t> _to_be_deleted;

  struct QueueElement
  {
    rclcpp::Time _time;
    uint32_t _request_index;

    bool operator<(const QueueElement& element) const
    {
      // purposely flip so earlier time comes first
      return this->_time > element._time;
    }
  };

  std::priority_queue<QueueElement> _priority_queue;
};

}

#endif