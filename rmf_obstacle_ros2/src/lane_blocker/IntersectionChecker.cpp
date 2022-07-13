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

#include <iostream>

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
  // We need to rotate the point about p1. To do this we translate p2 to p1,
  // apply rotation and translate back.
  auto r =
    t.inverse() * Eigen::Rotation2D<double>(to.center.theta - from.center.theta) * t;
  // t.rotate(Eigen::Rotation2D<double>(from.center.theta - to.center.theta));
  std::cout << "Transformation matrix: " << std::endl;
  auto final_t = t.matrix();
  std::cout << final_t << std::endl;
  return final_t;
  // const auto& matrix = t.matrix();
  // std::cout << matrix << std::endl;
  // return matrix;
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
      std::cout << name << "_min: " << box.min[0] << "," << box.min[1] << std::endl;
      std::cout << name << "_max: " << box.max[0] << "," << box.max[1] << std::endl;
    };

  std::cout << "================================================" << std::endl;
  std::cout << "Checking collision between: " << std:: endl;
  print_geom(o1, "Lane");
  print_geom(o2, "obs");

  auto make_collision_box =
   [](const CollisionGeometry& o, const bool origin) -> CollisionBox
   {
    CollisionBox box;
    if (origin)
    {
      box.min = Eigen::Vector2d{-o.size_x * 0.5, -o.size_y * 0.5};
      box.max = Eigen::Vector2d{o.size_x * 0.5, o.size_y * 0.5};
    }
    else
    {
      Eigen::Matrix<double, 3, 3> mat;
      const double th = o.center.theta;
      mat(0,0) = std::cos(th); mat(0,1) = std::sin(-1.0 *th); mat(0,2) = o.center.x;
      mat(1,0) = std::sin(th); mat(1,1) = std::cos(th); mat(1,2) =  o.center.y;
      mat(2,0) = 0; mat(2,1) =  0; mat(2,2) = 1.0;
      // Eigen::Transform<double, 2, Eigen::Affine> t;
      // t = Eigen::Translation<double, 2>(Eigen::Vector2d{o.center.x, o.center.y});
      // // We need to rotate the point about p1. To do this we translate p2 to p1,
      // // apply rotation and translate back.
      // t.rotate(Eigen::Rotation2D<double>(o.center.theta));
      // // t.rotate(Eigen::Rotation2D<double>(from.center.theta - to.center.theta));
      std::cout << "make collision Transformation matrix: " << std::endl;
      std::cout << mat << std::endl;
      std::vector<Eigen::Vector3d> vertices;
      vertices.push_back({-o.size_x * 0.5, o.size_y * 0.5, 1.0});
      vertices.push_back({-o.size_x * 0.5, -o.size_y * 0.5, 1.0});
      vertices.push_back({o.size_x * 0.5, o.size_y * 0.5, 1.0});
      vertices.push_back({o.size_x * 0.5, -o.size_y * 0.5, 1.0});

      for (auto& v : vertices)
      {
        v = mat * v;
        std::cout << v << std::endl;
      }

      box.min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
      box.max = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

      for (const auto& v : vertices)
      {
        for (std::size_t i = 0; i < 2; ++i)
        {
          if (v[i] < box.min[i])
            box.min[i] = v[i];
          if (v[i] > box.max[i])
            box.max[i] = v[i];
        }
      }
    }

    return box;
   };

  const auto& o1_box = make_collision_box(o1, true);
  print_box(o1_box, "Lane box");
  std::vector<Eigen::Vector2d> vertices;
  const auto o2_box = make_collision_box(o2, false);
  print_box(o2_box, "obs box");
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
  print_box(o2t_box, "obs transformed box");


  // how_much =
  //   [&]() -> double
  //   {
  //     double dist = std::numeric_limits<double>::max();
  //     for (std::size_t i = 0; i < 2; ++i)
  //     {
  //       std::cout << "dist: " << dist << std::endl;
  //       dist = std::min(
  //         dist,
  //         std::min(
  //         std::abs(o2t_box.min[i] - o1_box.max[i]),
  //         std::abs(o1_box.min[i] - o2t_box.max[i]))
  //       );
  //     }
  //     std::cout << "dist: " << dist << std::endl;

  //     return dist;
  //   }();

  how_much = 0.0;
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (o2t_box.min[i] > o1_box.max[i] || o2t_box.max[i] < o1_box.min[i])
    {
      // Does not overlap along this axis hence the two obstacles do not overlap
      how_much = std::min(
          std::abs(o2t_box.min[i] - o1_box.max[i]),
          std::abs(o2t_box.max[i] - o1_box.min[i]));
      return false;
    }
  }

  return true;
}
} // namespace IntersectionChecker
