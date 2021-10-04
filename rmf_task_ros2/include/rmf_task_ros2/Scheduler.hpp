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

#ifndef RMF_TASK_ROS2__SCHEDULER_HPP
#define RMF_TASK_ROS2__SCHEDULER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_task_ros2 {

class Scheduler
{
public:
  static std::unique_ptr<Scheduler> make(
    const std::shared_ptr<rclcpp::Node>& node);

  class Implementation;

private:
  Scheduler();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
}
}

#endif