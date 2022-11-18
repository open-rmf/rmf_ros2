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
    const double expected = 1.0;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(4.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("OBB geometries are not intersecting and 1m apart")
  {
    double how_much;
    const double expected = 0.586;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(4.0)
      .y(0.0)
      .theta(radians(45.0)),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("AABB geometries are overlapping along X-Axis")
  {
    double how_much;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(2.5)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are overlapping along Y-Axis")
  {
    double how_much;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(1.5)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are overlapping along X & Y-Axis")
  {
    double how_much;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(2.5)
      .y(0.5)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("AABB geometries are touching")
  {
    double how_much;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(3.0)
      .y(0.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("OBB geometries are overlapping along X & Y-Axis")
  {
    double how_much;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(0.0)
      .y(0.0)
      .theta(radians(45.0)),
      2.0,
      2.0
    );


    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(1.414)
      .y(1.0)
      .theta(0.0),
      2.0,
      2.0
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    CHECK(intersect);
  }

  WHEN("Test #1")
  {
    double how_much;
    const double expected = 2.344;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(8.6824)
      .y(-10.9616)
      .theta(0.25553),
      1.43478,
      0.5);


    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(12.002)
      .y(-10.1094)
      .theta(0.0),
      0.6,
      0.6
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("Test #2")
  {
    double how_much;
    const double expected = 6.593;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(11.6892)
      .y(-3.52843)
      .theta(0.29391),
      3.01193,
      0.5
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(12.002)
      .y(-10.9738)
      .theta(0.0),
      0.6,
      0.6
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("Test #3")
  {
    double how_much;
    const double expected = 4.292;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(9.57985)
      .y(-4.6367)
      .theta(1.1626),
      3.36581,
      0.5
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(12.1453)
      .y(-11.1404)
      .theta(0.0),
      0.6,
      0.6
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("Test #4")
  {
    double how_much;
    const double expected = 7.702;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(-3.7801)
      .y(-2.48618)
      .theta(-2.9587),
      4.76905,
      0.734582
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(6.81831)
      .y(-1.99772)
      .theta(0.99046),
      0.6,
      0.6
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }

  WHEN("Test #5")
  {
    double how_much;
    const double expected = 17.126;
    const auto ob1 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(-9.12337)
      .y(2.63674)
      .theta(7.0577),
      4.92557,
      1.66422
    );

    const auto ob2 = CollisionGeometry(
      geometry_msgs::build<geometry_msgs::msg::Pose2D>()
      .x(8.87474)
      .y(-5.78416)
      .theta(-1.9070),
      0.7,
      1.1
    );

    const bool intersect = IntersectionChecker::between(
      ob1, ob2, how_much);
    REQUIRE_FALSE(intersect);
    std::cout <<"expected: " << expected << std::endl;
    std::cout <<"how_much: " << how_much << std::endl;
    CHECK((how_much - expected) == Approx(0.0).margin(1e-3));
  }
}
