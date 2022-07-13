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

void transform_vertices(const CollisionGeometry& to,
  const CollisionGeometry& from, std::vector<Eigen::Vector3d>& vertices)
{
  // Eigen::Transform<double, 2, Eigen::Affine> t;
  // Eigen::Vector2d p1 = {to.center.x, to.center.y};
  // Eigen::Vector2d p2 = {from.center.x, from.center.y};

  // t = Eigen::Translation<double, 2>(-p1);
  // // We need to rotate the point about p1. To do this we translate p2 to p1,
  // // apply rotation and translate back.
  // auto r =
  //   t.inverse() * Eigen::Rotation2D<double>(to.center.theta - from.center.theta) * t;
  // // t.rotate(Eigen::Rotation2D<double>(from.center.theta - to.center.theta));
  // std::cout << "Transformation matrix: " << std::endl;
  // auto final_t = t.matrix();
  // std::cout << final_t << std::endl;
  // return final_t;

  Eigen::Matrix<double, 3, 3> trans_mat;
  trans_mat(0,0) = 1.0; trans_mat(0,1) = 0.0; trans_mat(0,2) = -to.center.x;
  trans_mat(1,0) = 0.0; trans_mat(1,1) = 1.0; trans_mat(1,2) =  -to.center.y;
  trans_mat(2,0) = 0; trans_mat(2,1) =  0; trans_mat(2,2) = 1.0;

  // First translate to p1
  Eigen::Matrix<double, 3, 3> rot_mat;
  const double th = to.center.theta - from.center.theta;
  rot_mat(0,0) = std::cos(th); rot_mat(0,1) = -1.0 * std::sin(th); rot_mat(0,2) = -to.center.x;
  rot_mat(1,0) = std::sin(th); rot_mat(1,1) = std::cos(th); rot_mat(1,2) =  -to.center.y;
  rot_mat(2,0) = 0; rot_mat(2,1) =  0; rot_mat(2,2) = 1.0;

  for (auto& v : vertices)
  {
    // v = trans_mat * v;
    v = rot_mat * v;
  }
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
    [](const CollisionGeometry& o) -> CollisionBox
    {
      CollisionBox box;
      box.min = Eigen::Vector2d{-o.size_x * 0.5, -o.size_y * 0.5};
      box.max = Eigen::Vector2d{o.size_x * 0.5, o.size_y * 0.5};
      return box;
    };

  auto get_vertices =
    [](const CollisionGeometry& o) -> std::vector<Eigen::Vector3d>
    {

      Eigen::Matrix<double, 3, 3> mat;
      const double th = o.center.theta;
      mat(0,0) = std::cos(th); mat(0,1) = std::sin(-1.0 *th); mat(0,2) = o.center.x;
      mat(1,0) = std::sin(th); mat(1,1) = std::cos(th); mat(1,2) =  o.center.y;
      mat(2,0) = 0; mat(2,1) =  0; mat(2,2) = 1.0;

      std::cout << "get_vertices matrix: " << std::endl;
      std::cout << mat << std::endl;
      std::vector<Eigen::Vector3d> vertices;
      vertices.push_back({-o.size_x * 0.5, o.size_y * 0.5, 1.0});
      vertices.push_back({-o.size_x * 0.5, -o.size_y * 0.5, 1.0});
      vertices.push_back({o.size_x * 0.5, o.size_y * 0.5, 1.0});
      vertices.push_back({o.size_x * 0.5, -o.size_y * 0.5, 1.0});

      for (auto& v : vertices)
        v = mat * v;

      return vertices;
    };


  const auto& o1_box = make_collision_box(o1);
  print_box(o1_box, "Lane box");
  auto vertices = get_vertices(o2);

  std::cout << "Obs vertices before trans: ";
  for (const auto& v : vertices)
    std::cout << "{" << v[0] << "," << v[1] << "}" << " ";
  std::cout << std::endl;

  // Transform o2 vertices into o1 coordinates
  transform_vertices(o1, o2, vertices);

  std::cout << "Obs vertices after trans: ";
  for (const auto& v : vertices)
    std::cout << "{" << v[0] << "," << v[1] << "}" << " ";
  std::cout << std::endl;

  // for (auto& v : vertices)
  //   v = transform * v;

  // Use SAT. Project o2 vertices onto o1 X & Y axes
  CollisionBox o2_box;
  o2_box.min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  o2_box.max = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

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

  print_box(o2_box, "obs transformed box");


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
  //         std::abs(o2_box.min[i] - o1_box.max[i]),
  //         std::abs(o1_box.min[i] - o2_box.max[i]))
  //       );
  //     }
  //     std::cout << "dist: " << dist << std::endl;

  //     return dist;
  //   }();

  auto dist =
    [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) -> double
    {
      return (a-b).norm();
    };

  how_much = std::numeric_limits<double>::max();
  how_much = std::min(how_much, dist(o1_box.min, o2_box.min));
  how_much = std::min(how_much, dist(o1_box.min, o2_box.max));
  how_much = std::min(how_much, dist(o1_box.max, o2_box.min));
  how_much = std::min(how_much, dist(o1_box.max, o2_box.max));


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
