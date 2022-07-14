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


#include "../IntersectionChecker.hpp"

#include <rmf_utils/catch.hpp>

#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <iostream>

using CollisionGeometry = IntersectionChecker::CollisionGeometry;

SCENARIO("Test IntersectionChecker")
{

  auto radians =
    [](double degree) -> double
    {
      return degree * M_PI / 180.0;
    };

  WHEN("AABB geometries are not intersecting and 1m apart")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(4.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK(how_much - 1.0 == Approx(0.0).margin(1e-3));
  }

  WHEN("OBB geometries are not intersecting and 1m apart")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(4.0)
        .y(0.0)
        .theta(radians(45.0)))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - 0.586) == Approx(0.0).margin(1e-3));
  }

  WHEN("AABB geometries are overlapping along X-Axis")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(2.5)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are overlapping along Y-Axis")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(1.5)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are overlapping along X & Y-Axis")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(2.5)
        .y(0.5)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are touching")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(3.0)
        .y(0.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("OBB geometries are overlapping along X & Y-Axis")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(0.0)
        .y(0.0)
        .theta(radians(45.0)))
      .size_x(2.0)
      .size_y(2.0);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(1.414)
        .y(1.0)
        .theta(0.0))
      .size_x(2.0)
      .size_y(2.0);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("Test #1")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(8.6824)
        .y(-10.9616)
        .theta(0.255513))
      .size_x(1.43478)
      .size_y(0.5);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(12.002)
        .y(-10.1094)
        .theta(0.0))
      .size_x(0.6)
      .size_y(0.6);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    CHECK((how_much - 2.336) == Approx(0.0).margin(1e-1));
    std::cout <<"how_much: " << how_much << std::endl;
  }

  WHEN("Test #2")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(11.6892)
        .y(-3.52843)
        .theta(0.293981))
      .size_x(3.01193)
      .size_y(0.5);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(12.002)
        .y(-10.9738)
        .theta(0.0))
      .size_x(0.6)
      .size_y(0.6);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    CHECK((how_much - 6.773) == Approx(0.0).margin(1e-1));
    std::cout <<"how_much: " << how_much << std::endl;
  }

  WHEN("Test #3")
  {
    double how_much;
    const auto ob1 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(9.57985)
        .y(-4.6367)
        .theta(1.16262))
      .size_x(3.36581)
      .size_y(0.5);

    const auto ob2 = vision_msgs::build<CollisionGeometry>()
      .center(geometry_msgs::build<geometry_msgs::msg::Pose2D>()
        .x(12.1453)
        .y(-11.1404)
        .theta(0.0))
      .size_x(0.6)
      .size_y(0.6);

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    CHECK((how_much - 4.314) == Approx(0.0).margin(1e-1));
    std::cout <<"how_much: " << how_much << std::endl;
  }
}