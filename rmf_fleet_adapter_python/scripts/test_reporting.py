#!/usr/bin/env python3

from concurrent.futures import thread
import rclpy
import time
import json
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.battery as battery
import rmf_adapter.plan as plan
import rmf_adapter.type as Type

from rmf_adapter.robot_update_handle import Tier

import datetime
import asyncio
import threading
from itertools import groupby

from test_utils import MockRobotCommand
from test_utils import update_observer
from rmf_msg_observer import RmfMsgType

from functools import partial

test_task_id = "patrol.direct.001"  # aka task_id
map_name = "test_map"
fleet_name = "test_fleet"
rmf_server_uri = "ws://localhost:7878"  # random port

start_name = "start_wp"  # 7
finish_name = "finish_wp"  # 10
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
    # Copied from test_loop.py
    test_graph = graph.Graph()

    test_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    test_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
    test_graph.add_waypoint(map_name, [5.0, -5.0]) # 2
    test_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    test_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
    test_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    test_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    test_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    test_graph.add_waypoint(map_name, [0.0, 5.0])  # 8
    test_graph.add_waypoint(map_name, [5.0, 5.0]) # 9
    test_graph.add_waypoint(map_name, [0.0, 10.0]).set_holding_point(
        True).set_charger(True)  # 10

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

    test_graph.add_key(start_name, 7)
    test_graph.add_key(finish_name, 10)

    # INIT FLEET ==============================================================
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(
        linear=traits.Limits(0.7, 0.3),
        angular=traits.Limits(1.0, 0.45),
        profile=profile
    )

    adapter = adpt.MockAdapter("TestReportingAdapter")
    fleet = adapter.add_fleet(
        fleet_name, robot_traits, test_graph, rmf_server_uri
    )

    # Set fleet battery profile
    battery_sys = battery.BatterySystem.make(24.0, 40.0, 8.8)
    mech_sys = battery.MechanicalSystem.make(70.0, 40.0, 0.22)
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_power_sys = battery.PowerSystem.make(20.0)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_power_sys = battery.PowerSystem.make(10.0)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    b_success = fleet.set_task_planner_params(
        battery_sys, motion_sink, ambient_sink, tool_sink, 0.2, 1.0, False)

    assert b_success, "set task planner params failed"

    cmd_node = Node("RobotCommandHandle")

    start = plan.Start(adapter.now(), 0, 0.0)

    def updater_inserter(handle_obj, updater):
        updater.update_battery_soc(1.0)
        handle_obj.updater = updater

    robot_cmd = MockRobotCommand(cmd_node, test_graph)

    fleet.add_robot(
        robot_cmd, "R0", profile, [start], partial(updater_inserter, robot_cmd)
    )

    fleet.fleet_state_update_period(datetime.timedelta(milliseconds=5))

    # FINAL PREP ==============================================================
    rclpy_executor = SingleThreadedExecutor()
    rclpy_executor.add_node(cmd_node)

    # GO! =====================================================================
    adapter.start()

    fleet_state = None
    logs = []
    cv_update = threading.Condition()
    def receive_fleet_update(type, data):
        nonlocal fleet_state
        nonlocal logs
        with cv_update:
            if type == RmfMsgType.FleetState:
                fleet_state = data
            elif type == RmfMsgType.FleetLog:
                for log in data['robots']['R0']:
                    logs.append(log)
            cv_update.notify_all()

    # INIT TASK STATE OBSERVER ==============================================
    print("spawn observer thread")
    fut = asyncio.Future()
    observer_th = threading.Thread(
        target=update_observer,
        args=(
            receive_fleet_update,
            fut,
            {
                RmfMsgType.FleetState: [],
                RmfMsgType.FleetLog: []
            }))
    observer_th.start()

    counter = 0
    while counter < 10:
        with cv_update:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive an update')

            if fleet_state is None:
                raise RuntimeError('Failed to receive an update')

            for robot, state in fleet_state['robots'].items():
                assert robot == 'R0', f'Wrong name of robot: {robot}'
                assert len(state['issues']) == 0

        counter += 1

    # We successfully found that the robot R0 has no issues!

    first_issue_ticket = robot_cmd.updater.create_issue(
        Tier.Warning,
        'Something bad is happening',
        {
            'This object': 'has details',
            'about': ['the', 'bad', 'thing']
        }
    )

    counter = 0
    counter_limit = 1000
    noticed_the_issue = False
    while counter < counter_limit:
        with cv_update:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive a fleet state')

            num_issues = len(fleet_state['robots']['R0']['issues'])
            if num_issues > 0:
                if not noticed_the_issue:
                    print(f'Noticed the first issue:\n{fleet_state["robots"]["R0"]["issues"][0]}')
                    counter_limit = counter + 50
                noticed_the_issue = True
                assert num_issues == 1
            else:
                assert not noticed_the_issue, \
                    'The issue disappeared earlier than it should have'

        counter += 1

    assert noticed_the_issue, 'The first issue was never noticed!'
    assert len(logs) == 1, f'Wrong number of logs: {len(logs)}'

    second_issue_ticket = robot_cmd.updater.create_issue(
        Tier.Error,
        'Something even worse is happening',
        {
            'This problem': 'is even worse',
            'so': {
                'we': 'labeled it',
                'as': 'an error'
            }
        }
    )

    counter = 0
    counter_limit = 1000
    noticed_both_issues = False
    while counter < counter_limit:
        with cv_update:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive an update')

            num_issues = len(fleet_state['robots']['R0']['issues'])
            if num_issues > 1:
                if not noticed_both_issues:
                    print('Noticed both issues:')
                    for issue in fleet_state['robots']['R0']['issues']:
                        print(f'{issue}')
                    counter_limit = counter + 50
                noticed_both_issues = True
            else:
                assert num_issues == 1
                assert not noticed_both_issues, \
                    'One of the issues disappeared earlier than it should have'

        counter += 1

    assert noticed_both_issues, 'The second issue was never noticed!'
    assert len(logs) == 2, f'Wrong number of logs: {len(logs)}'

    first_issue_ticket.resolve(
        {
            'This': 'issue',
            'is': ['now', 'fixed']
        }
    )

    first_issue_gone = False
    counter = 0
    counter_limit = 1000
    while counter < counter_limit:
        with cv_update:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive an update')

            num_issues = len(fleet_state['robots']['R0']['issues'])
            if num_issues < 2:
                if not first_issue_gone:
                    print('Noticed that an issue was resolved')
                    counter_limit = counter + 50
                first_issue_gone = True
                assert num_issues == 1
            else:
                assert num_issues == 2, f'Wrong number of issues: {num_issues}'
                assert not first_issue_gone, 'Somehow an issue came back??'

        counter += 1

    assert first_issue_gone
    assert len(logs) == 3, f'Wrong number of logs: {len(logs)}'

    # Delete the second ticket, causing its issue to get dropped
    del second_issue_ticket

    both_issues_gone = False
    counter = 0
    counter_limit = 1000
    while counter < counter_limit:
        with cv_update:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive an update')

            num_issues = len(fleet_state['robots']['R0']['issues'])
            if num_issues == 0:
                if not both_issues_gone:
                    print('Noticed that both issues are gone')
                    counter_limit = counter + 50
                both_issues_gone = True
            else:
                assert not both_issues_gone, 'Somehow an issue came back??'

        counter += 1

    assert both_issues_gone
    assert len(logs) == 4, f'Wrong number of logs: {len(logs)}'

    robot_cmd.updater.log_info('I am logging some info')
    robot_cmd.updater.log_warning('I am logging a warning')
    robot_cmd.updater.log_error('I am logging an error')

    counter = 0
    counter_limit = 1000
    received_all_logs = False
    with cv_update:
        while counter < counter_limit and len(logs) < 7:
            if not cv_update.wait(timeout=10.0):
                raise RuntimeError('Failed to receive an update')

            if not received_all_logs and len(logs) == 7:
                print('Received all the logs')
                received_all_logs = True
                counter_limit = counter + 50

            counter += 1

    print(f'Noticed {len(logs)} logs:')
    for log in logs:
        print(f'{log}')

    assert len(logs) == 7, f'Received the wrong number of logs: {len(logs)}'

    fut.set_result(True)


if __name__ == "__main__":
    main()
