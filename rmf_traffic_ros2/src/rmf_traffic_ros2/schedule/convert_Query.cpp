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

namespace {
//==============================================================================
Eigen::Isometry2d to_eigen(const geometry_msgs::msg::Pose2D& pose)
{
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  tf.translation() = Eigen::Vector2d{pose.x, pose.y};
  tf.rotate(Eigen::Rotation2Dd(pose.theta));

  return tf;
}

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

  for (const auto& space : region.spaces)
    output.push_back(parse_space(space, context));

  if (region.timespan.has_lower_bound)
    output.set_lower_time_bound(Time(Duration(region.timespan.lower_bound)));

  if (region.timespan.has_upper_bound)
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
  for (const auto& region : from.regions)
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

  if (from.timespan.has_lower_bound)
    timespan.set_lower_time_bound(Time{Duration{from.timespan.lower_bound}});

  if (from.timespan.has_upper_bound)
    timespan.set_upper_time_bound(Time{Duration{from.timespan.upper_bound}});

  return output;
}
} // anonymous namespace

//==============================================================================
rmf_traffic::schedule::Query::Spacetime convert(
  const rmf_traffic_msgs::msg::ScheduleQuerySpacetime& from)
{
  if (rmf_traffic_msgs::msg::ScheduleQuerySpacetime::ALL == from.type)
    return rmf_traffic::schedule::Query::Spacetime();
  else if (rmf_traffic_msgs::msg::ScheduleQuerySpacetime::REGIONS == from.type)
    return parse_regions(from);
  else if (rmf_traffic_msgs::msg::ScheduleQuerySpacetime::TIMESPAN == from.type)
    return parse_timespan(from);

  // *INDENT-OFF*
  throw std::runtime_error(
    "Invalid rmf_traffic_msgs/ScheduleQuerySpacetime type ["
    + std::to_string(from.type) + "]");
  // *INDENT-ON*
}

namespace {
//==============================================================================
geometry_msgs::msg::Pose2D from_eigen(const Eigen::Isometry2d& pose)
{
  geometry_msgs::msg::Pose2D msg;
  msg.x = pose.translation().x();
  msg.y = pose.translation().y();
  msg.theta = Eigen::Rotation2Dd(pose.rotation()).angle();

  return msg;
}

//==============================================================================
rmf_traffic_msgs::msg::Timespan convert_timespan(
  const rmf_traffic::Time* lower_bound,
  const rmf_traffic::Time* upper_bound)
{
  rmf_traffic_msgs::msg::Timespan msg;
  if (lower_bound)
  {
    msg.has_lower_bound = true;
    msg.lower_bound = lower_bound->time_since_epoch().count();
  }
  else
    msg.has_lower_bound = false;

  if (upper_bound)
  {
    msg.has_upper_bound = true;
    msg.upper_bound = upper_bound->time_since_epoch().count();
  }
  else
    msg.has_upper_bound = false;

  return msg;
}

//==============================================================================
void convert_regions(
  rmf_traffic_msgs::msg::ScheduleQuerySpacetime& msg,
  const rmf_traffic::schedule::Query::Spacetime::Regions& from)
{
  geometry::ShapeContext shape_context;
  for (const auto& region : from)
  {
    rmf_traffic_msgs::msg::Region region_msg;
    region_msg.map = region.get_map();

    region_msg.timespan = convert_timespan(
      region.get_lower_time_bound(),
      region.get_upper_time_bound());

    for (const auto& space : region)
    {
      rmf_traffic_msgs::msg::Space space_msg;
      space_msg.shape = shape_context.insert(space.get_shape());
      space_msg.pose = from_eigen(space.get_pose());
      region_msg.spaces.emplace_back(std::move(space_msg));
    }

    msg.regions.emplace_back(std::move(region_msg));
  }

  msg.shape_context = convert(shape_context);
}

//==============================================================================
void convert_timespan(
  rmf_traffic_msgs::msg::ScheduleQuerySpacetime& msg,
  const rmf_traffic::schedule::Query::Spacetime::Timespan& from)
{
  msg.timespan = convert_timespan(
    from.get_lower_time_bound(),
    from.get_upper_time_bound());
}
} // anonymous namespace

//==============================================================================
rmf_traffic_msgs::msg::ScheduleQuerySpacetime convert(
  const rmf_traffic::schedule::Query::Spacetime& from)
{
  rmf_traffic_msgs::msg::ScheduleQuerySpacetime msg;
  const auto mode = from.get_mode();
  msg.type = static_cast<uint16_t>(mode);

  if (rmf_traffic::schedule::Query::Spacetime::Mode::Regions == mode)
    convert_regions(msg, *from.regions());
  else if (rmf_traffic::schedule::Query::Spacetime::Mode::Timespan == mode)
    convert_timespan(msg, *from.timespan());

  return msg;
}

//==============================================================================
rmf_traffic::schedule::Query::Participants convert(
  const rmf_traffic_msgs::msg::ScheduleQueryParticipants& from)
{
  using Participants = rmf_traffic::schedule::Query::Participants;

  if (from.ALL == from.type)
    return Participants::make_all();
  else if (from.INCLUDE == from.type)
    return Participants::make_only(from.ids);
  else if (from.EXCLUDE == from.type)
    return Participants::make_all_except(from.ids);

  // *INDENT-OFF*
  throw std::runtime_error(
    "[rmf_traffic_ros2::convert] Invalid type value for "
    "rmf_traffic::schedule::Query::Participants: " +
    std::to_string(from.type));
  // *INDENT-ON*
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleQueryParticipants convert(
  const rmf_traffic::schedule::Query::Participants& from)
{
  using Participants = rmf_traffic::schedule::Query::Participants;

  rmf_traffic_msgs::msg::ScheduleQueryParticipants output;

  const auto mode = from.get_mode();
  output.type = static_cast<uint16_t>(mode);

  if (Participants::Mode::Exclude == mode)
    output.ids = from.exclude()->get_ids();
  else if (Participants::Mode::Include == mode)
    output.ids = from.include()->get_ids();

  return output;
}

//==============================================================================
rmf_traffic::schedule::Query convert(
  const rmf_traffic_msgs::msg::ScheduleQuery& from)
{
  auto output = rmf_traffic::schedule::query_all();
  output.spacetime() = convert(from.spacetime);
  output.participants() = convert(from.participants);

  return output;
}

//==============================================================================
rmf_traffic_msgs::msg::ScheduleQuery convert(
  const rmf_traffic::schedule::Query& from)
{
  rmf_traffic_msgs::msg::ScheduleQuery output;
  output.spacetime = convert(from.spacetime());
  output.participants = convert(from.participants());

  return output;
}

} // namespace rmf_traffic_ros2
