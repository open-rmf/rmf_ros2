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
          rmf_traffic::Time(rmf_traffic::Duration(segment.final_time)),
          profiles.at(segment.profile_index),
          to_eigen(segment.final_position),
          to_eigen(segment.final_velocity));
  }

  return output;
}

namespace {
//==============================================================================
struct ProfileContext
{
  using ProfilePtr = rmf_traffic::Trajectory::ProfilePtr;
  std::vector<ProfilePtr> profiles;

  using EntryMap = std::unordered_map<ProfilePtr, std::size_t>;
  EntryMap entry_map;

  std::size_t insert(ProfilePtr profile)
  {
    const auto insertion =
        entry_map.insert(std::make_pair(profile, profiles.size()));

    const bool inserted = insertion.second;
    if(inserted)
      profiles.emplace_back(std::move(profile));

    return insertion.first->second;
  }
};

//==============================================================================
std::vector<rmf_traffic_msgs::msg::TrajectoryProfile> convert(
    geometry::ConvexShapeContext& shape_context,
    const ProfileContext& profile_context)
{
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

  return outputs;
}

} // anonymous namespace

//==============================================================================
rmf_traffic_msgs::msg::Trajectory convert(const rmf_traffic::Trajectory& from)
{
  ProfileContext profile_context;

}

} // namespace rmf_traffic_ros2
