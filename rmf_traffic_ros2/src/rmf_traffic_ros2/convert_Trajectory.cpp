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

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>

#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <unordered_map>
#include <vector>

namespace rmf_traffic_ros2 {

//==============================================================================
Eigen::Vector3d to_eigen(const std::array<double, 3>& values)
{
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

//==============================================================================
std::array<double, 3> from_eigen(const Eigen::Vector3d& values)
{
  return {values[0], values[1], values[2]};
}

//==============================================================================
rmf_traffic::Trajectory convert(const rmf_traffic_msgs::msg::Trajectory& from)
{
  if(from.maps.empty())
    throw std::runtime_error("No map found!");

  // TODO(MXG): Remember to add multi-map support to the Trajectory class. For
  // now, we'll just grab the first map in the message.
  rmf_traffic::Trajectory output(from.maps.front());

  geometry::ConvexShapeContext context = convert(from.convex_shape_context);

  std::vector<rmf_traffic::Trajectory::ProfilePtr> profiles;
  profiles.reserve(from.profiles.size());
  for(const auto& profile : from.profiles)
  {
    using rmf_traffic_msgs::msg::ConvexShape;
    const auto shape = context.at(profile.shape);

    using rmf_traffic_msgs::msg::TrajectoryProfile;
    if(TrajectoryProfile::GUIDED == profile.autonomy)
    {
      profiles.emplace_back(
            rmf_traffic::Trajectory::Profile::make_guided(shape));
    }
    else if(TrajectoryProfile::QUEUED == profile.autonomy)
    {
      profiles.emplace_back(
            rmf_traffic::Trajectory::Profile::make_queued(
              shape, profile.queue_id));
    }
    else if(TrajectoryProfile::AUTONOMOUS == profile.autonomy)
    {
      profiles.emplace_back(
            rmf_traffic::Trajectory::Profile::make_autonomous(shape));
    }
    else
    {
      throw std::runtime_error(
            "Invalid trajectory profile autonomy type: "
            + std::to_string(profile.autonomy));
    }
  }

  for(const auto& segment : from.segments)
  {
    output.insert(
          rmf_traffic::Time(rmf_traffic::Duration(segment.finish_time)),
          profiles.at(segment.profile_index),
          to_eigen(segment.finish_position),
          to_eigen(segment.finish_velocity));
  }

  return output;
}

namespace {
//==============================================================================
struct ProfileContext
{
  using ConstProfilePtr = rmf_traffic::Trajectory::ConstProfilePtr;
  std::vector<ConstProfilePtr> profiles;

  using EntryMap = std::unordered_map<ConstProfilePtr, std::size_t>;
  EntryMap entry_map;

  std::size_t insert(ConstProfilePtr profile)
  {
    const auto insertion =
        entry_map.insert(std::make_pair(profile, profiles.size()));

    const bool inserted = insertion.second;
    if(inserted)
      profiles.emplace_back(std::move(profile));

    if(profiles.size() > std::numeric_limits<uint8_t>::max())
    {
      // TODO(MXG): In the future, we should be able to support arbitrarily
      // large and complex trajectories by automatically splitting them into
      // multiple Trajectory messages.
      throw std::runtime_error(std::string()
            + "[convert(rmf_traffic::Trajectory)] Unexpectedly large "
            +"trajectory being converted (more than 256 unique profiles).");
    }

    return insertion.first->second;
  }
};

//==============================================================================
void insert_context(
    rmf_traffic_msgs::msg::Trajectory& msg,
    const ProfileContext& profile_context)
{
  geometry::ConvexShapeContext shape_context;
  std::vector<rmf_traffic_msgs::msg::TrajectoryProfile> outputs;
  for(const auto& profile : profile_context.profiles)
  {
    rmf_traffic_msgs::msg::TrajectoryProfile output;
    output.autonomy = static_cast<uint16_t>(profile->get_autonomy());
    output.shape = shape_context.insert(profile->get_shape());

    const auto* queue_info = profile->get_queue_info();
    if(queue_info)
      output.queue_id = queue_info->get_queue_id();

    outputs.emplace_back(std::move(output));
  }

  msg.profiles = outputs;
  msg.convex_shape_context = convert(shape_context);
}

//==============================================================================
rmf_traffic_msgs::msg::TrajectorySegment convert(
    const rmf_traffic::Trajectory::Segment& segment,
    ProfileContext& context)
{
  rmf_traffic_msgs::msg::TrajectorySegment output;
  output.finish_time = segment.get_finish_time().time_since_epoch().count();
  output.finish_position = from_eigen(segment.get_finish_position());
  output.finish_velocity = from_eigen(segment.get_finish_velocity());
  output.profile_index =
      static_cast<uint8_t>(context.insert(segment.get_profile()));

  return output;
}

} // anonymous namespace

//==============================================================================
rmf_traffic_msgs::msg::Trajectory convert(const rmf_traffic::Trajectory& from)
{
  ProfileContext profile_context;
  rmf_traffic_msgs::msg::Trajectory output;
  output.maps.push_back(from.get_map_name());

  for(const auto& segment : from)
    output.segments.emplace_back(convert(segment, profile_context));

  insert_context(output, profile_context);
  return output;
}

} // namespace rmf_traffic_ros2
