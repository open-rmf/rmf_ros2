/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SRC__INTERSECTIONCHECKER_HPP
#define SRC__INTERSECTIONCHECKER_HPP

#include <Eigen/Geometry>

#include <vision_msgs/msg/bounding_box2_d.hpp>

//==============================================================================
// TODO(YV): Consider making this a loadable plugin via pluginlib
namespace IntersectionChecker {

using CollisionGeometry = vision_msgs::msg::BoundingBox2D;

// Return true if intersect.
// If intersect, how_much represents the overlap in meters
// If not intersect, how_much represents the separating distance in meters.
bool between(
  const CollisionGeometry& o1,
  const CollisionGeometry& o2,
  double& how_much);

} // namespace IntersectionChecker

#endif // SRC__INTERSECTIONCHECKER_HPP
