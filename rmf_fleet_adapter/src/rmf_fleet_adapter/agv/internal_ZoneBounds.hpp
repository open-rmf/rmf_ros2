/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ZONEBOUNDS_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ZONEBOUNDS_HPP

#include <Eigen/Geometry>

#include <cmath>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// Is a point inside a zone's bounds?
///
/// \param[in] point
///   The (x, y) to test, in RMF canonical coordinates.
///
/// \param[in] zone_location
///   The zone's centre, in RMF canonical coordinates.
///
/// \param[in] zone_orientation
///   The zone's orientation in radians, in RMF canonical coordinates.
///
/// \param[in] zone_dimensions
///   The zone's full extents, aligned with the zone's local (x, y) axes.
///
/// \param[in] margin
///   Grows the zone by this much on every side.
inline bool is_inside_zone(
  const Eigen::Vector2d& point,
  const Eigen::Vector2d& zone_location,
  double zone_orientation,
  const Eigen::Vector2d& zone_dimensions,
  double margin = 0.0)
{
  const Eigen::Vector2d d = point - zone_location;

  // Express the offset in the zone's frame. On the map the zone's width
  // axis points along (c, s) and its depth axis along (-s, c), so local_x
  // and local_y are the dot products of d with each.
  const double c = std::cos(zone_orientation);
  const double s = std::sin(zone_orientation);
  const double local_x = d.x() * c + d.y() * s;
  const double local_y = -d.x() * s + d.y() * c;

  const double half_x = std::abs(zone_dimensions.x()) / 2.0 + margin;
  const double half_y = std::abs(zone_dimensions.y()) / 2.0 + margin;

  return std::abs(local_x) <= half_x && std::abs(local_y) <= half_y;
}

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_ZONEBOUNDS_HPP
