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

#include "IntersectionChecker.hpp"

//==============================================================================
namespace IntersectionChecker {

namespace {



// CollisionGeometry make_collision_geometry(
//   Eigen::Vector2d origin,
//   Eigen::Vector2d p_x,
//   Eigen::Vector2d p_y)
// {
//   CollisionGeometry geometry;

//   return geometry;
// }







} // anonymous namespace


//==============================================================================
bool broadpahse(
  const CollisionGeometry& o1,
  const CollisionGeometry& o2)
{
  return false;
}

bool narrowphase(
  const CollisionGeometry& o1,
  const CollisionGeometry& o2,
  double& how_much)
{
  return false;
}
} // namespace IntersectionChecker
