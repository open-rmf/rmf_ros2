/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include "internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
namespace agv {

class EasyCommandHandle;
using EasyCommandHandlePtr = std::shared_ptr<EasyCommandHandle>;
using TransformDictionary = std::unordered_map<std::string, Transformation>;

//==============================================================================
struct NavParams
{
  bool skip_rotation_commands;
  std::optional<TransformDictionary> transforms_to_robot_coords;
  double max_merge_waypoint_distance = 0.3;
  double max_merge_lane_distance = 0.1;
  double min_lane_length = 1e-8;

  std::optional<Eigen::Vector3d> to_rmf_coordinates(
    const std::string& map,
    Eigen::Vector3d position)
  {
    if (!transforms_to_robot_coords.has_value())
    {
      return position;
    }

    const auto tf_it = transforms_to_robot_coords->find(map);
    if (tf_it == transforms_to_robot_coords->end())
    {
      return std::nullopt;
    }

    return tf_it->second.apply_inverse(position);
  }
};

//==============================================================================
class EasyFullControl::Implementation
{
public:
  std::shared_ptr<FleetUpdateHandle> fleet_handle;
  // Map robot name to its EasyCommandHandle
  std::unordered_map<std::string, EasyCommandHandlePtr> cmd_handles;
  // TODO(YV): Get these constants from EasyCommandHandle::Configuration
  std::shared_ptr<NavParams> nav_params;

  static std::shared_ptr<EasyFullControl> make(
    std::shared_ptr<FleetUpdateHandle> fleet_handle,
    bool skip_rotation_commands,
    std::optional<TransformDictionary> transforms_to_robot_coords)
  {
    auto handle = std::shared_ptr<EasyFullControl>(new EasyFullControl);
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        Implementation{
          fleet_handle,
          {},
          std::make_shared<NavParams>(NavParams{
            skip_rotation_commands,
            std::move(transforms_to_robot_coords)
          })
        });
    return handle;
  }

  const std::shared_ptr<Node>& node() const
  {
    return FleetUpdateHandle::Implementation::get(*fleet_handle).node;
  }
};

} // namespace agv
} // namespace rmf_fleet_adapter
