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


  std::vector<Eigen::Vector2d> vertices;
  const auto o2_box = make_collision_box(o2);
  vertices.push_back({o2_box.min[0], o2_box.min[1]});
  vertices.push_back({o2_box.min[0], o2_box.max[1]});
  vertices.push_back({o2_box.max[0], o2_box.min[1]});
  vertices.push_back({o2_box.max[0], o2_box.max[1]});
  // Transform o2 vertices into o1 coordinates
  const auto& transform = get_transform(o1, o2);
  for (auto& v : vertices)
  {
    v = (transform * Eigen::Vector3d{v[0], v[1], 1.0}).block<2,1>(0, 0);
  }

  // Use SAT. Project o2 vertices onto o1 X & Y axes
  // Get the unit vector from o1 center along X Axis
  // auto x_axis = Eigen::Vector2d{o1_box.max[0] - o1.center.x, 0.0};
  // x_axis /= x_axis.norm();
  // auto y_axis = Eigen::Vector2d{0.0, o1_box.max[1] - o1.center.y};
  // y_axis /= y_axis.norm();

  // Project all transformed o2 points onto each axis and compare bounds with o1_box

  // X-Axis
  double min_value_x = std::numeric_limits<double>::max();
  double max_value_x = std::numeric_limits<double>::min();
  for (const auto& v : vertices)
  {
    if (v[0] < min_value_x)
      min_value_x = v[0];
  }
  for (const auto& v : vertices)
  {
    if (v[0] > max_value_x)
      max_value_x = v[0];
  }

  // X-Axis
  double min_value_y = std::numeric_limits<double>::max();
  double max_value_y = std::numeric_limits<double>::min();
  for (const auto& v : vertices)
  {
    if (v[1] < min_value_y)
      min_value_y = v[1];
  }
  for (const auto& v : vertices)
  {
    if (v[1] > max_value_y)
      max_value_y = v[1];
  }

  if ((min_value_x >= o1_box.min[0] && min_value_x <= o1_box.max[0]) ||
    (max_value_x >= o1_box.min[0] && min_value_x <= o1_box.max[0]) ||
    (min_value_y >= o1_box.min[1] && min_value_y <= o1_box.max[1]) ||
    (max_value_y >= o1_box.min[1] && min_value_y <= o1_box.max[1]))
  {
    return true;
  }


  return false;
}
} // namespace IntersectionChecker
