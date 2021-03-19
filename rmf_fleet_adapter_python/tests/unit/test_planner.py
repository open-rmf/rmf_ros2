#! /usr/bin/env python3 

import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.graph.lane as lane
import rmf_adapter.plan as plan

import datetime

"""
                  10
                   |
                   |
                   8------9
                   |      |
                   |      |
     3------4------5------6------7
                   |      |
                   |      |
                   1------2
                   |
                   |
                   0
"""
map_name = "test_map"
nav_graph = graph.Graph()
nav_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
nav_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
nav_graph.add_waypoint(map_name, [5.0, -5.0])  # 2
nav_graph.add_waypoint(map_name, [-10.0, 0])  # 3
nav_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
nav_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
nav_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
nav_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
nav_graph.add_waypoint(map_name, [0.0, 5.0]) \
    .set_holding_point(True)  # 8
nav_graph.add_waypoint(map_name, [5.0, 5.0]) \
    .set_passthrough_point(True)  # 9
nav_graph.add_waypoint(map_name, [0.0, 10.0]) \
    .set_parking_spot(True)  # 10

lanes = [[0, 1],
         [1, 2],
         [1, 5],
         [2, 6],
         [3, 4],
         [4, 5],
         [5, 6],
         [6, 7],
         [5, 8],
         [6, 9],
         [8, 9],
         [8, 10]]
for _lane in lanes:
    nav_graph.add_bidir_lane(*_lane)
assert nav_graph.num_lanes == 24

profile = traits.Profile(geometry.make_final_convex_circle(1.0),
    geometry.make_final_convex_circle(1.5))
vehicle_traits = traits.VehicleTraits(
    linear=traits.Limits(0.5, 0.3),
    angular=traits.Limits(0.3, 0.4),
    profile=profile)
vehicle_traits.differential.reversible = False

config = plan.Configuration(nav_graph, vehicle_traits)
planner = plan.Planner(config)
assert(planner)

start = plan.Start(
    datetime.datetime.today(),
    0,
    0.0)
goal = plan.Goal(7)
waypoints = planner.get_plan_waypoints(start, goal)
assert(waypoints)
