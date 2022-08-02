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

#ifndef NDEBUG
#include <iostream>
#endif

//==============================================================================
namespace IntersectionChecker {

namespace {

struct CollisionBox
{
  Eigen::Vector2d min; // min x & y
  Eigen::Vector2d max; // max x & y
};

CollisionBox make_reference_box(const CollisionGeometry& o)
{
  CollisionBox box;
  box.min = Eigen::Vector2d{-o.size_x * 0.5, -o.size_y * 0.5};
  box.max = Eigen::Vector2d{o.size_x* 0.5, o.size_y* 0.5};
  return box;
}

std::pair<CollisionBox, std::vector<Eigen::Vector3d>> make_transformed_box(
  const CollisionGeometry& to,
  const CollisionGeometry& from)
{
  // Create the 'from' geometry's vertices at the origin
  std::vector<Eigen::Vector3d> vertices;
  vertices.push_back({-from.size_x * 0.5, from.size_y * 0.5, 1.0});
  vertices.push_back({-from.size_x * 0.5, -from.size_y * 0.5, 1.0});
  vertices.push_back({from.size_x* 0.5, from.size_y* 0.5, 1.0});
  vertices.push_back({from.size_x* 0.5, -from.size_y* 0.5, 1.0});

  Eigen::Matrix<double, 3, 3> rot1;
  const double th2 = from.center.theta;
  rot1(0, 0) = std::cos(th2);
  rot1(0, 1) = std::sin(-1.0 *th2);
  rot1(0, 2) = 0;
  rot1(1, 0) = std::sin(th2);
  rot1(1, 1) = std::cos(th2);
  rot1(1, 2) = 0;
  rot1(2, 0) = 0;
  rot1(2, 1) =  0;
  rot1(2, 2) = 1.0;

  #ifndef NDEBUG
  std::cout << "Obs rot matrix: " << std::endl;
  std::cout << rot1 << std::endl;
  #endif

  // Translate and rotate the 'from' geometry's vertices to their actual positions
  for (auto& v : vertices)
  {
    v = rot1 * v;
    v[0] = v[0] + from.center.x;
    v[1] = v[1] + from.center.y;
  }

  #ifndef NDEBUG
  std::cout << "Obs vertices before trans: ";
  for (const auto& v : vertices)
    std::cout << "{" << v[0] << "," << v[1] << "}" << " ";
  std::cout << std::endl;
  #endif

  Eigen::Matrix<double, 3, 3> rot2;
  const double th3 = -to.center.theta;
  rot2(0, 0) = std::cos(th3);
  rot2(0, 1) = std::sin(-1.0 *th3);
  rot2(0, 2) = 0;
  rot2(1, 0) = std::sin(th3);
  rot2(1, 1) = std::cos(th3);
  rot2(1, 2) = 0;
  rot2(2, 0) = 0;
  rot2(2, 1) =  0;
  rot2(2, 2) = 1.0;

  #ifndef NDEBUG
  std::cout << "Lane rot inv matrix: " << std::endl;
  std::cout << rot2 << std::endl;
  #endif

  // Translate and rotate the 'from' geometry's vertices by
  // the inverse of the 'to' geometry.
  // Vertex coordinates of 'from' geometry are now w.r.t. the 'to' geometry's frame,
  // where the 'to' geometry's frame is at the origin without rotation.
  for (auto& v : vertices)
  {
    v[0] = v[0] - to.center.x;
    v[1] = v[1] - to.center.y;
    v = rot2 * v;
  }

  #ifndef NDEBUG
  std::cout << "Obs vertices after trans: ";
  for (const auto& v : vertices)
    std::cout << "{" << v[0] << "," << v[1] << "}" << " ";
  std::cout << std::endl;
  #endif

  CollisionBox o2_box;
  o2_box.min =
  {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  o2_box.max =
  {std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};

  for (const auto& v : vertices)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (v[i] < o2_box.min[i])
        o2_box.min[i] = v[i];
      if (v[i] > o2_box.max[i])
        o2_box.max[i] = v[i];
    }
  }

  return {o2_box, vertices};
}

} // anonymous namespace


//==============================================================================
bool between(
  const CollisionGeometry& o1,
  const CollisionGeometry& o2,
  double& how_much)
{
  #ifndef NDEBUG
  auto print_geom =
    [](const CollisionGeometry& o, const std::string& name)
    {
      std::cout << name << ": {" << o.center.x << ","
                << o.center.y << "," << o.center.theta << "} [" << o.size_x
                << "," << o.size_y << "]" << std::endl;
    };

  auto print_box =
    [](const CollisionBox& box, const std::string& name)
    {
      std::cout << name << "_min: " << box.min[0] << "," << box.min[1]
                << std::endl;
      std::cout << name << "_max: " << box.max[0] << "," << box.max[1]
                << std::endl;
    };

  std::cout << "================================================" << std::endl;
  std::cout << "Checking collision between: " << std::endl;
  print_geom(o1, "Lane");
  print_geom(o2, "obs");
  #endif

  const auto o1_box = make_reference_box(o1);
  const auto result = make_transformed_box(o1, o2);
  const auto& o2_box = result.first;

  #ifndef NDEBUG
  print_box(o1_box, "Lane box");
  print_box(o2_box, "obs transformed box");
  #endif

  // TODO(YV): Consider moving this to a separate narrowphase implementation
  // to speed up compute
  how_much = std::numeric_limits<double>::min();
  for (std::size_t i = 0; i < 2; ++i)
  {
    double dist = std::numeric_limits<double>::min();
    // O2 projections are on the left of O1 extremas
    if (o2_box.max[i] < o1_box.min[i])
    {
      dist = std::abs(o2_box.max[i] - o1_box.min[i]);
    }
    // O2 projections are on the right of O1 extremas
    else if (o2_box.min[i] > o1_box.max[i])
    {
      dist = std::abs(o2_box.min[i] - o1_box.max[i]);
    }
    // They intersect
    else
      continue;
    how_much = std::max(how_much, dist);
  }

  // Simple SAT theorem application
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (o2_box.min[i] > o1_box.max[i] || o2_box.max[i] < o1_box.min[i])
    {
      return false;
    }
  }

  return true;
}
} // namespace IntersectionChecker
