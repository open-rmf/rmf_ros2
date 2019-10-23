/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/schedule/Query.hpp>

#include <rmf_traffic_ros2/geometry/Shape.hpp>

namespace rmf_traffic_ros2 {

//==============================================================================
Eigen::Isometry2d to_eigen(const geometry_msgs::msg::Pose2D& pose)
{
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  tf.translation() = Eigen::Vector2d{pose.x, pose.y};
  tf.rotate(Eigen::Rotation2Dd(pose.theta));

  return tf;
}

namespace {
//==============================================================================
rmf_traffic::geometry::Space parse_space(
    const rmf_traffic_msgs::msg::Space& space,
    const rmf_traffic_ros2::geometry::ShapeContext& context)
{
  return {context.at(space.shape), to_eigen(space.pose)};
}

//==============================================================================
rmf_traffic::Region parse_region(
    const rmf_traffic_msgs::msg::Region& region,
    const rmf_traffic_ros2::geometry::ShapeContext& context)
{
  using namespace rmf_traffic;
  Region output(region.map, {});

  for(const auto& space : region.spaces)
    output.push_back(parse_space(space, context));

  if(region.timespan.has_lower_bound)
    output.set_lower_time_bound(Time(Duration(region.timespan.lower_bound)));

  if(region.timespan.has_upper_bound)
    output.set_upper_time_bound(Time(Duration(region.timespan.upper_bound)));

  return output;
}

//==============================================================================
rmf_traffic::schedule::Query::Spacetime parse_regions(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  const rmf_traffic_ros2::geometry::ShapeContext context =
      convert(from.shape_context);

  std::vector<rmf_traffic::Region> regions;
  for(const auto& region : from.regions)
    regions.emplace_back(parse_region(region, context));

  return regions;
}

//==============================================================================
rmf_traffic::schedule::Query::Spacetime parse_timespan(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  using namespace rmf_traffic;

  rmf_traffic::schedule::Query::Spacetime output;
  auto& timespan = output.query_timespan(from.timespan.maps);

  if(from.timespan.has_lower_bound)
    timespan.set_lower_time_bound(Time{Duration{from.timespan.lower_bound}});

  if(from.timespan.has_upper_bound)
    timespan.set_upper_time_bound(Time{Duration{from.timespan.upper_bound}});

  return output;
}
} // anonymous namespace

//==============================================================================
rmf_traffic::schedule::Query::Spacetime convert(
    const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::ALL == from.type)
    return rmf_traffic::schedule::Query::Spacetime();
  else if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::REGIONS == from.type)
    return parse_regions(from);
  else if(rmf_traffic_msgs::msg::ScheduleQuerySpacetime::TIMESPAN == from.type)
    return parse_timespan(from);

  throw std::runtime_error(
        "Invalid rmf_traffic_msgs/ScheduleQuerySpacetime type ["
        + std::to_string(from.type) + "]");
}

} // namespace rmf_traffic_ros2
