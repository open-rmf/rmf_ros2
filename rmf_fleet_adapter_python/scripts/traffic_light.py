#!/usr/bin/env python3

# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.nodes as nodes

import rmf_adapter.easy_traffic_light as traffic_light

import time
from threading import Thread

# assertion var
visited_waypoints = []

def init_graph():
    bot1_path = [1, 2, 3, 4, 5]
    bot2_path = [0, 3, 4, 6]

    test_graph_vis = \
        """
                             6 (bot2 end)
                             |
        (bot1 start)         |
        1------2------3------4-----5 (bot1 end)
                      |
                      |
                      0 (bot2 start)
        """
    print(test_graph_vis)
    print(f"dummybot1 path {bot1_path}")
    print(f"dummybot2 path {bot2_path}")
    print("=================================================\n")
    return bot1_path, bot2_path


class MockTrafficLightHandle:
    def __init__(self, name):
        print(f" [{name}] Start Mock EZ traffic light")
        self.name = name
        self.handler = None
        self.path_checkpoints = []
        self.coordinates = [[3, 0, 0], [1, 1, 0], [2, 1, 0],
                            [3, 1, 0], [4, 1, 0], [5, 1, 0], [4, 3, 0]]
        pass

    def traffic_light_cb(self, ez_traffic_light):
        print("Add traffic light handler!")
        self.handler = ez_traffic_light

    def pause_cb(self):
        print("I was told to pause")
        pass

    def resume_cb(self):
        print("I was told to resume")
        pass

    def pub_follow_path(self, path=[]):
        print(f"[{self.name}] Update RMF robot to follow a new path")
        self.path_checkpoints = path
        _path = []
        for i in path:
            _path.append(adpt.Waypoint(
                "test_map", self.coordinates[i], 0.0, True))
        self.handler.follow_new_path(_path)

    def move_robot(self, curr_checkpoint):
        print(f"[{self.name}] wait at checkpoint {curr_checkpoint}")
        for _ in range(100):
            result = self.handler.waiting_at(curr_checkpoint)
            time.sleep(0.5)
            if (result == traffic_light.WaitingInstruction.Resume):
                break

        time.sleep(0.2)
        nxt_wp = self.path_checkpoints[curr_checkpoint + 1]
        nxt_wp_coor = self.coordinates[nxt_wp]
        result = self.handler.moving_from(curr_checkpoint, nxt_wp_coor)
        print(f"[{self.name}] moved to {nxt_wp}, result: {result}")
        if result == traffic_light.MovingInstruction.MovingError:
            return False
        else:
            # for assertion
            visited_waypoints.append((self.name, nxt_wp))
            return True


def main():
    rclpy.init()
    adpt.init_rclcpp()
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(2.5, 1.0),
                                        angular=traits.Limits(2.5, 1.0),
                                        profile=profile)

    print("Test Run Easy Traffic Light")
    path1, path2 = init_graph()

    adapter = adpt.Adapter.make("TestTrafficLightAdapter")

    # Add robot 1
    mock_tl = MockTrafficLightHandle("dummybot1")
    adapter.add_easy_traffic_light(
        mock_tl.traffic_light_cb,
        "dummybot_fleet",
        mock_tl.name,
        robot_traits,
        mock_tl.pause_cb,
        mock_tl.resume_cb)

    # Add robot 2
    mock_tl2 = MockTrafficLightHandle("dummybot2")
    adapter.add_easy_traffic_light(
        mock_tl2.traffic_light_cb,
        "dummybot_fleet",
        mock_tl2.name,
        robot_traits,
        mock_tl2.pause_cb,
        mock_tl2.resume_cb)

    adapter.start()

    # this is crucial to wait for traffic light to start
    time.sleep(1)

    # inform rmf that the robot is following these path
    mock_tl.pub_follow_path(path1)
    mock_tl2.pub_follow_path(path2)

    # Move robot along the designated path.
    def control_robot(mod, path):
        for cp in range(len(path)-1):
            if not mod.move_robot(cp):
                print("Error! Robot Failed to move")
                break
            time.sleep(0.2)

    th1 = Thread(target=control_robot, args=(mock_tl, path1))
    th2 = Thread(target=control_robot, args=(mock_tl2, path2))

    th1.start()
    th2.start()
    th1.join()
    th2.join()

    print(f"Sequence of visited waypoints: \n{visited_waypoints}")
    assert visited_waypoints == [
        ("dummybot2", 3), 
        ("dummybot2", 4), 
        ("dummybot1", 2), 
        ("dummybot2", 6), 
        ("dummybot1", 3), 
        ("dummybot1", 4), 
        ("dummybot1", 5)], "Sequence of visited waypoints is incorrect."

    print("Done Traffic Light Tutorial!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
