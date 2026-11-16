/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <rmf_utils/catch.hpp>

#include <make_trajectory.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <cmath>

namespace {
//==============================================================================
rmf_fleet_msgs::msg::Location location(
  double x, double y, double yaw,
  rmf_traffic::Time t = rmf_traffic::Time())
{
  rmf_fleet_msgs::msg::Location l;
  l.t = rmf_traffic_ros2::convert(t);
  l.x = x;
  l.y = y;
  l.yaw = yaw;
  l.level_name = "L1";
  return l;
}

//==============================================================================
double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a.head<2>() - b.head<2>()).norm();
}
} // anonymous namespace

//==============================================================================
SCENARIO("Checkpoint reached along a read-only robot's route")
{
  using namespace std::chrono_literals;

  const rmf_traffic::agv::VehicleTraits traits{
    {0.5, 0.5}, {0.6, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.5)}
  };

  // An L-shaped route that stops at its corners and turns in place at (4, 2)
  // and (6, 2). (2, 0) lies on a straight line, so Interpolate::positions
  // leaves it out.
  const auto start = rmf_traffic::Time(10s);
  rmf_fleet_msgs::msg::RobotState state;
  state.location = location(0.0, 0.0, 0.0, start);
  state.path = {
    location(2.0, 0.0, 0.0),
    location(4.0, 0.0, 0.0),
    location(4.0, 2.0, M_PI/2.0),
    location(6.0, 2.0, 0.0)
  };

  bool sitting = true;
  const auto trajectory = make_trajectory(state, traits, sitting);
  REQUIRE_FALSE(sitting);

  // A path index is not a checkpoint
  CHECK(trajectory.size() > state.path.size() + 1);

  WHEN("The robot drives along its route")
  {
    const auto motion = rmf_traffic::Motion::compute_cubic_splines(trajectory);
    const auto finish = *trajectory.finish_time();
    std::size_t last_checkpoint = 0;
    for (auto t = start; t <= finish; t += 100ms)
    {
      const Eigen::Vector3d p = motion->compute_position(t);

      // The robot's path is the points it has not reached yet. The route only
      // heads in +x and +y, so a point is ahead if x + y is larger.
      rmf_fleet_msgs::msg::RobotState later;
      later.location = location(p[0], p[1], p[2], t);
      for (const auto& l : state.path)
      {
        if (!later.path.empty() || (l.x - p[0]) + (l.y - p[1]) > 1e-3)
          later.path.push_back(l);
      }

      const auto c = reached_checkpoint(trajectory, later);
      REQUIRE(c.has_value());
      CAPTURE(rmf_traffic::time::to_seconds(t - start), *c);

      // Never a waypoint that the robot has not reached yet (within 1 mm)
      CHECK((trajectory[*c].time() <= t
        || distance(trajectory[*c].position(), p) < 1e-3));

      // Nor behind where the robot arrived at its current stop
      std::size_t j = 0;
      while (j + 1 < trajectory.size() && trajectory[j+1].time() <= t)
        ++j;
      std::size_t arrived = j;
      while (arrived > 0 && distance(
          trajectory[arrived-1].position(), trajectory[j].position()) < 1e-3)
      {
        --arrived;
      }
      CHECK(*c >= arrived);

      CHECK(*c >= last_checkpoint);
      last_checkpoint = *c;
    }

    // At the end of the route, the robot has reached it
    CHECK(trajectory[last_checkpoint].position().head<2>().isApprox(
        Eigen::Vector2d(6.0, 2.0), 1e-6));
  }

  WHEN("The robot has passed (2, 0), which is not a waypoint")
  {
    rmf_fleet_msgs::msg::RobotState later;
    later.location = location(2.5, 0.0, 0.0);
    later.path.assign(state.path.begin() + 1, state.path.end());

    const auto c = reached_checkpoint(trajectory, later);
    REQUIRE(c.has_value());
    CHECK(trajectory[*c].position()[0] <= 2.5);
    REQUIRE(*c + 1 < trajectory.size());
    CHECK(trajectory[*c+1].position()[0] > 2.5);
  }

  WHEN("The robot is past the next point in its path")
  {
    // It has not reported reaching (4, 0) yet, so (4, 0) is not reached
    rmf_fleet_msgs::msg::RobotState later;
    later.location = location(4.0, 1.0, M_PI/2.0);
    later.path.assign(state.path.begin() + 1, state.path.end());

    const auto c = reached_checkpoint(trajectory, later);
    REQUIRE(c.has_value());
    CHECK(trajectory[*c].position()[0] < 4.0 - 1e-3);
  }

  WHEN("The trajectory is too short")
  {
    rmf_traffic::Trajectory single;
    single.insert(start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    CHECK_FALSE(reached_checkpoint(single, state).has_value());
  }
}
