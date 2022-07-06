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

  t = Eigen::Translation<double, 2>(-p1);
  t.rotate(Eigen::Rotation2D<double>(from.center.theta - to.center.theta));
  const auto& matrix = t.matrix();
  return matrix;
}

struct CollisionBox
{
  Eigen::Vector2d min; // min x & y
  Eigen::Vector2d max; // max x & y
};

} // anonymous namespace


//==============================================================================
bool between(
  const CollisionGeometry& o1,
  const CollisionGeometry& o2,
  double& how_much)
{
  auto make_collision_box =
   [](const CollisionGeometry& o, const bool origin) -> CollisionBox
   {
    CollisionBox box;
    box.min = origin ? Eigen::Vector2d{-o.size_x * 0.5, -o.size_y * 0.5} :
      Eigen::Vector2d{o.center.x -o.size_x * 0.5, o.center.y -o.size_y * 0.5};
    box.max = origin ? Eigen::Vector2d{o.size_x * 0.5, o.size_y * 0.5} :
      Eigen::Vector2d{o.center.x + o.size_x * 0.5, o.center.y + o.size_y * 0.5};
    return box;
   };

  const auto& o1_box = make_collision_box(o1, true);
  std::vector<Eigen::Vector2d> vertices;
  const auto o2_box = make_collision_box(o2, false);
  vertices.push_back({o2_box.min[0], o2_box.min[1]});
  vertices.push_back({o2_box.max[0], o2_box.max[1]});
  // Transform o2 vertices into o1 coordinates
  const auto& transform = get_transform(o1, o2);
  for (auto& v : vertices)
    v = (transform * Eigen::Vector3d{v[0], v[1], 1.0}).block<2,1>(0, 0);

  // Use SAT. Project o2 vertices onto o1 X & Y axes
  CollisionBox o2t_box;
  o2t_box.min = vertices[0];
  o2t_box.max = vertices[1];

  how_much =
    [&]() -> double
    {
      double dist = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < 2; ++i)
      {
        dist = std::min(
          dist,
          std::min(
          std::abs(o2t_box.min[i] - o1_box.max[i]),
          std::abs(o1_box.min[i] - o2t_box.max[i]))
        );
      }
      return dist;
    }();

  for (std::size_t i = 0; i < 2; ++i)
  {
    if (o2t_box.min[i] > o1_box.max[i] || o2t_box.max[i] < o1_box.min[i])
      return false;
  }

  return true;
}
} // namespace IntersectionChecker
