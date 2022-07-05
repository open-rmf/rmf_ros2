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

Eigen::Matrix3d get_transform(const CollisionGeometry& to,
  const CollisionGeometry& from)
{
  Eigen::Transform<double, 2, Eigen::Affine> t;
  Eigen::Vector2d p1 = {to.center.x, to.center.y};
  Eigen::Vector2d p2 = {from.center.x, from.center.y};

  t = Eigen::Translation<double, 2>(p2 - p1);
  t.rotate(Eigen::Rotation2D<double>(from.center.theta - to.center.theta));
  return t.matrix();
}

struct CollisionBox
{
  Eigen::Vector2d min; // min x & y
  Eigen::Vector2d max; // max x & y
};

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

  auto make_collision_box =
   [](const CollisionGeometry& o) -> CollisionBox
   {
    CollisionBox box;
    box.min = {o.center.x - o.size_x * 0.5, o.center.y - o.size_y * 0.5};
    box.max = {o.center.x + o.size_x * 0.5, o.center.y + o.size_y * 0.5};
    return box;
   };

  const auto& o1_box = make_collision_box(o1);


  // std::vector<Eigen::Vector3d> vertices;
  // const auto o2_box = make_collision_box(o2);
  // for (std::size_t i = 0; i < 2; ++i)
  // {
  //   for (std::size_t j = 1; j >= 0; --j)
  //   {
  //     vertices.push_back({o2_box.min[0], o2_box.min[0], o2_box.min[0]});
  //   }
  // }

  // Use SAT


  return false;
}
} // namespace IntersectionChecker
