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

#include <rmf_traffic_ros2/schedule/Change.hpp>
#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>

#include "internal_convert_vector.hpp"

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Change::Add::Item convert(
  const rmf_traffic_msgs::msg::ScheduleChangeAddItem& from)
{
  return {
    from.route_id,
    from.storage_id,
    std::make_shared<rmf_traffic::Route>(convert(from.route))
  };
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeAddItem convert(
  const rmf_traffic::schedule::Change::Add::Item& from)
{
  if (!from.route)
    throw std::runtime_error("Cannot convert a nullptr route into a message");

  return rmf_traffic_msgs::build<rmf_traffic_msgs::msg::ScheduleChangeAddItem>()
    .route_id(from.route_id)
    .storage_id(from.storage_id)
    .route(convert(*from.route));
}

//==============================================================================
rmf_traffic::schedule::Change::Add convert(
  const rmf_traffic_msgs::msg::ScheduleChangeAdd& from)
{
  return {
    from.plan_id,
    convert_vector<rmf_traffic::schedule::Change::Add::Item>(from.items)
  };
}

//==============================================================================
rmf_traffic::schedule::Change::Delay convert(
  const rmf_traffic_msgs::msg::ScheduleChangeDelay& from)
{
  return rmf_traffic::schedule::Change::Delay{
    rmf_traffic::Duration(from.delay)
  };
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeDelay convert(
  const rmf_traffic::schedule::Change::Delay& from)
{
  rmf_traffic_msgs::msg::ScheduleChangeDelay output;
  output.delay = from.duration().count();
  return output;
}

//==============================================================================
rmf_traffic::schedule::Change::Cull convert(
  const rmf_traffic_msgs::msg::ScheduleChangeCull& from)
{
  return rmf_traffic::Time(rmf_traffic::Duration(from.time));
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleChangeCull convert(
  const rmf_traffic::schedule::Change::Cull& from)
{
  rmf_traffic_msgs::msg::ScheduleChangeCull output;
  output.time = from.time().time_since_epoch().count();
  return output;
}

} // namespace rmf_traffic_ros2
