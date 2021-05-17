#!/usr/bin/env python3

import rclpy
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.battery as battery
import rmf_adapter.plan as plan
import rmf_adapter.type as Type

from test_utils import MockRobotCommand
from test_utils import MockDispenser, MockIngestor
from test_utils import TaskSummaryObserver

from functools import partial


test_name = 'test_loop'  # aka task_id
map_name = "test_map"
fleet_name = "test_fleet"

start_name = "start"
finish_name = "finish"
loop_count = 2


def main():
    # INIT RCL ================================================================
    rclpy.init()

    try:
        adpt.init_rclcpp()
    except RuntimeError:
        # Continue if it is already initialized
        pass

    # INIT GRAPH ==============================================================
    test_graph = graph.Graph()

    test_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    test_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
    test_graph.add_waypoint(map_name, [5.0, -5.0]).set_holding_point(True)  # 2
    test_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    test_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
    test_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    test_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    test_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    test_graph.add_waypoint(map_name, [0.0, 5.0])  # 8
    test_graph.add_waypoint(map_name, [5.0, 5.0]).set_holding_point(True)  # 9
    test_graph.add_waypoint(map_name, [0.0, 10.0])  # 10

    assert test_graph.get_waypoint(2).holding_point
    assert test_graph.get_waypoint(9).holding_point
    assert not test_graph.get_waypoint(10).holding_point

    test_graph_legend = \
        """
        D : Dispenser
        I : Ingestor
        H : Holding Point
        K : Dock
        Numerals : Waypoints
        ---- : Lanes
        """

    test_graph_vis = \
        test_graph_legend + \
        """
                         10(I,K)
                          |
                          |
                          8------9(H)
                          |      |
                          |      |
            3------4------5------6------7(D,K)
                          |      |
                          |      |
                          1------2(H)
                          |
                          |
                          0
       """

    test_graph.add_bidir_lane(0, 1)  # 0   1
    test_graph.add_bidir_lane(1, 2)  # 2   3
    test_graph.add_bidir_lane(1, 5)  # 4   5
    test_graph.add_bidir_lane(2, 6)  # 6   7
    test_graph.add_bidir_lane(3, 4)  # 8   9
    test_graph.add_bidir_lane(4, 5)  # 10 11
    test_graph.add_bidir_lane(5, 6)  # 12 13
    test_graph.add_dock_lane(6, 7, "A")  # 14 15
    test_graph.add_bidir_lane(5, 8)  # 16 17
    test_graph.add_bidir_lane(6, 9)  # 18 19
    test_graph.add_bidir_lane(8, 9)  # 20 21
    test_graph.add_dock_lane(8, 10, "B")  # 22 23

    assert test_graph.num_lanes == 24

    test_graph.add_key(start_name, 7)
    test_graph.add_key(finish_name, 10)

    assert len(test_graph.keys) == 2 and start_name in test_graph.keys \
        and finish_name in test_graph.keys

    # INIT FLEET ==============================================================
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(0.7, 0.3),
                                        angular=traits.Limits(1.0, 0.45),
                                        profile=profile)

    # Manages loop requests
    adapter = adpt.MockAdapter("TestLoopAdapter")
    fleet = adapter.add_fleet(fleet_name, robot_traits, test_graph)

    # Set up task request callback function
    # we will only accept Loop task here
    def task_request_cb(task_profile):
        from rmf_task_msgs.msg import TaskType	
        if(task_profile.description.task_type == TaskType.TYPE_LOOP):
            return True
        else:
            return False

    fleet.accept_task_requests(task_request_cb)

    # Set fleet battery profile
    battery_sys = battery.BatterySystem.make(24.0, 40.0, 8.8)
    mech_sys = battery.MechanicalSystem.make(70.0, 40.0, 0.22)
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_power_sys = battery.PowerSystem.make(20.0)
    ambient_sink = battery.SimpleDevicePowerSink(battery_sys, ambient_power_sys)
    tool_power_sys = battery.PowerSystem.make(10.0)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    b_success = fleet.set_task_planner_params(
        battery_sys, motion_sink, ambient_sink, tool_sink, 0.2, 1.0, False)

    assert b_success, "set battery param failed"

    cmd_node = Node("RobotCommandHandle")

    # Test compute_plan_starts, which tries to place the robot on the navgraph
    # Your robot MUST be near a waypoint or lane for this to work though!
    starts = plan.compute_plan_starts(test_graph,
                                      map_name,
                                      [[-10.0], [0.0], [0.0]],
                                      adapter.now())
    assert [x.waypoint for x in starts] == [3], [x.waypoint for x in starts]

    # Alternatively, if you DO know where your robot is, place it directly!
    starts = [plan.Start(adapter.now(),
                         0,
                         0.0)]

    # Lambda to insert an adapter
    def updater_inserter(handle_obj, updater):
        updater.update_battery_soc(1.0)
        handle_obj.updater = updater

    # Manages and executes robot commands
    robot_cmd = MockRobotCommand(cmd_node, test_graph)

    fleet.add_robot(robot_cmd,
                    "T0",
                    profile,
                    starts,
                    partial(updater_inserter, robot_cmd))

    # INIT TASK SUMMARY OBSERVER ==============================================
    # Note: this is used for assertation check on TaskSummary.Msg
    observer = TaskSummaryObserver()

    # FINAL PREP ==============================================================
    rclpy_executor = SingleThreadedExecutor()
    rclpy_executor.add_node(cmd_node)
    rclpy_executor.add_node(observer)

    # GO! =====================================================================
    adapter.start()

    print("\n")
    print("# SENDING SINGLE LOOP REQUEST ####################################")
    print(test_graph_vis)

    observer.reset()
    observer.add_task(test_name)

    # Create a task to dispatch
    task_desc = Type.CPPTaskDescriptionMsg()
    # this is the time when the robot reaches the start waypoint for loop
    task_desc.start_time_sec = int(time.time()) + 50
    task_desc.loop = adpt.type.CPPLoopMsg(fleet_name,
                                          loop_count,
                                          start_name,
                                          finish_name)
    task_profile = Type.CPPTaskProfileMsg()
    task_profile.description = task_desc
    task_profile.task_id = test_name
    adapter.dispatch_task(task_profile)

    for i in range(1000):
        if observer.all_tasks_complete():
            print("Tasks Complete.")
            break
        rclpy_executor.spin_once(1)
        # time.sleep(0.2)

    results = observer.count_successful_tasks()
    print("\n== DEBUG TASK REPORT ==")
    print("Visited waypoints:", robot_cmd.visited_waypoints)
    print(f"Sucessful Tasks: {results[0]} / {results[1]}")

    assert results[0] == results[1], "Not all tasks were completed."

    error_msg = "Robot did not take the expected route"
    assert robot_cmd.visited_waypoints == [
        0, 0, 5, 5, 6, 6, 7,
        6, 5, 5, 8, 8, 10,
        8, 5, 5, 6, 6, 7,
        6, 5, 5, 8, 8, 10], error_msg

    cmd_node.destroy_node()
    observer.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
