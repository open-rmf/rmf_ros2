#!/usr/bin/env python3

import rmf_dispenser_msgs.msg as dispenser_msgs
import rmf_ingestor_msgs.msg as ingestor_msgs
from rmf_task_msgs.msg import TaskSummary

from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.graph as graph

import time
import datetime
from functools import partial
import asyncio
from rmf_msg_observer import AsyncRmfMsgObserver, RmfMsgType

class Constants:
    # Static Definitions
    dispenser_results_topic = 'dispenser_results'
    dispenser_states_topic = 'dispenser_states'
    dispenser_requests_topic = 'dispenser_requests'

    ingestor_results_topic = 'ingestor_results'
    ingestor_states_topic = 'ingestor_states'
    ingestor_requests_topic = 'ingestor_requests'

    task_summary_topic = 'task_summaries'


class MockDispenser(Node):
    def __init__(self, name, dispense_duration_sec=1.0,
                 publish_states=True, publish_results=True):
        super().__init__(name)

        # Variables
        self.dispense_duration_sec = float(dispense_duration_sec)
        self.publish_states = publish_states
        self.publish_results = publish_results
        self.reset(name)

        self.current_request_idx = 0  # Points to current task
        # time ordered queue of received requests
        # We never pop this queue. Instead, we use
        # self.current_request_idx to determine which tasks are pending
        self.tasks = []

        # Currently only one dispense task can be processed at a time
        self.timer = self.create_timer(
            0.1,
            self._timer_cb
        )

        # Pub-sub
        self.result_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            Constants.dispenser_results_topic,
            1
        )

        self.state_pub = self.create_publisher(
            dispenser_msgs.DispenserState,
            Constants.dispenser_states_topic,
            1
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            Constants.dispenser_requests_topic,
            self._process_request_cb,
            1
        )

    def reset(self, name=None):
        # reset sets a new name if provided
        if name is not None:
            self.name = name
        self.tasks = []

    def _process_request_cb(self, msg):
        # Process request only if addressed to this dispenser
        if msg.target_guid != self.name:
            return

        task_already_received = [
            x for x in self.tasks if x.request_guid == msg.request_guid]

        if task_already_received:
            # print("[MockDispenser] DUPLICATE REQUEST")
            pass
        else:
            # New request to process
            print("[MockDispenser] NEW REQUEST RECEIVED")
            self.tasks.append(msg)

    def _timer_cb(self):
        # Report Dispenser State if publish_states is set
        if self.publish_states:
            state = dispenser_msgs.DispenserState()
            state.time = self.get_clock().now().to_msg()
            state.guid = self.name
            if self.tasks:
                # the queue contains only unfinished tasks
                state.request_guid_queue = [
                    x.request_guid for x in self.tasks][
                        self.current_request_idx:]
            if state.request_guid_queue:
                state.mode = dispenser_msgs.DispenserState.BUSY
            else:
                state.mode = dispenser_msgs.DispenserState.IDLE
            state.seconds_remaining = self.dispense_duration_sec
            self.state_pub.publish(state)

        if self.tasks:
            if self.current_request_idx >= len(self.tasks):
                # No more new tasks, do nothing
                return
        else:
            # Tasks is empty, return
            return

        # If enough time has elapsed, consider dispensing done
        time_now = self.get_clock().now().to_msg().sec
        time_elapsed = time_now - self.tasks[
            self.current_request_idx].time.sec
        if time_elapsed > self.dispense_duration_sec:
            if self.publish_results:
                # Send a success message
                result = dispenser_msgs.DispenserResult()
                result.time = self.get_clock().now().to_msg()
                result.status = dispenser_msgs.DispenserResult.SUCCESS
                result.source_guid = self.name
                result.request_guid = self.tasks[
                    self.current_request_idx].request_guid
                self.result_pub.publish(result)
            self.current_request_idx += 1


class MockIngestor(Node):
    def __init__(self, name, ingest_duration_sec=1,
                 publish_states=True, publish_results=True):
        super().__init__(name)

        # Variables
        self.ingest_duration_sec = float(ingest_duration_sec)
        self.publish_states = publish_states
        self.publish_results = publish_results
        self.reset(name)

        self.current_request_idx = 0  # Points to current task
        # time ordered queue of received requests
        # We never pop this queue. Instead, we use
        # self.current_request_idx to determine which tasks are pending
        self.tasks = []

        # Currently only one ingest task can be processed at a time
        self.timer = self.create_timer(
            0.1,
            self._timer_cb
        )

        # Pub-sub
        self.result_pub = self.create_publisher(
            ingestor_msgs.IngestorResult,
            Constants.ingestor_results_topic,
            1
        )
        self.state_pub = self.create_publisher(
            ingestor_msgs.IngestorState,
            Constants.ingestor_states_topic,
            1
        )
        self.request_sub = self.create_subscription(
            ingestor_msgs.IngestorRequest,
            Constants.ingestor_requests_topic,
            self._process_request_cb,
            1
        )

    def reset(self, name=None):
        # reset sets a new name if provided
        if name is not None:
            self.name = name
        self.tasks = []

    def _process_request_cb(self, msg):
        # Process request only if addressed to this Ingestor
        if msg.target_guid != self.name:
            return

        task_already_received = [
            x for x in self.tasks if x.request_guid == msg.request_guid]

        if task_already_received:
            # print("[MockIngestor] DUPLICATE REQUEST")
            pass
        else:
            # New request to process
            print("[MockIngestor] NEW REQUEST RECEIVED")
            self.tasks.append(msg)

    def _timer_cb(self):
        # Report Ingestor State
        if self.publish_states:
            state = ingestor_msgs.IngestorState()
            state.time = self.get_clock().now().to_msg()
            state.guid = self.name
            if self.tasks:
                # the queue contains only unfinished tasks
                state.request_guid_queue = [
                    x.request_guid for x in self.tasks][
                        self.current_request_idx:]
            if state.request_guid_queue:
                state.mode = ingestor_msgs.IngestorState.BUSY
            else:
                state.mode = ingestor_msgs.IngestorState.IDLE
            state.seconds_remaining = self.ingest_duration_sec
            self.state_pub.publish(state)

        if self.tasks:
            if self.current_request_idx >= len(self.tasks):
                # No more new tasks, do nothing
                return
        else:
            # Tasks is empty, return
            return

        # If enough time has elapsed, consider ingesting done
        time_now = self.get_clock().now().to_msg().sec
        time_elapsed = time_now - self.tasks[
            self.current_request_idx].time.sec
        if time_elapsed > self.ingest_duration_sec:
            if self.publish_results:
                # Send a success message
                result = ingestor_msgs.IngestorResult()
                result.time = self.get_clock().now().to_msg()
                result.status = ingestor_msgs.IngestorResult.SUCCESS
                result.source_guid = self.name
                result.request_guid = self.tasks[
                    self.current_request_idx].request_guid
                self.result_pub.publish(result)
            self.current_request_idx += 1


class MockRobotCommand(adpt.RobotCommandHandle):
    class EventListener(graph.lane.Executor):
        def __init__(self, dock_to_wp, wp):
            graph.lane.Executor.__init__(self)

            self.dock_to_wp = dock_to_wp
            self.wp = wp

        def dock_execute(self, dock):
            self.dock_to_wp[dock.dock_name] = self.wp
            # print("DOCK EVENT EXECUTED FOR DOCK:", dock.dock_name)

        # And these are all overrided but meant to do nothing
        def door_open_execute(self, door_open):
            return

        def door_close_execute(self, door_close):
            return

        def lift_door_open_execute(self, lift_door_open):
            return

        def lift_door_close_execute(self, lift_door_close):
            return

        def lift_move_execute(self, lift_move):
            return

    def __init__(self, node, graph):
        adpt.RobotCommandHandle.__init__(self)

        self.node = node
        self.reset()

        for i in range(graph.num_lanes):
            lane = graph.get_lane(i)

            # lane.entry and lane.exit are a Lane::Node wrappers
            if lane.entry.event:
                executor = self.EventListener(self.dock_to_wp,
                                              lane.exit.waypoint_index)
                try:
                    lane.entry.event.execute(executor)
                except Exception:
                    print(type(lane.entry.event))
                    print("EVENT EXECUTE FOR LANE", i, "NOT IMPLEMENTED")

        # print("Registered Docks:", self.dock_to_wp, "\n")

    def reset(self):
        self.updater = None
        self.active = False
        self.timer = None
        self.current_waypoint_target = 0
        self.visited_waypoints = []
        self.dock_to_wp = {}

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,  # function!
                        path_finished_callback):
        print("\n[RobotCommandHandle] Setting new path of %d waypoints..."
              % len(waypoints))
        print("Waypoints:", [x.graph_index for x in waypoints])

        self.stop()

        self.current_waypoint_target = 0
        self.active = True
        self.timer = self.node.create_timer(
            0.01,
            partial(self._timer_cb,
                    waypoints=waypoints,
                    next_arrival_estimator=next_arrival_estimator,
                    path_finished_callback=path_finished_callback)
        )

    def stop(self):
        try:
            self.timer.reset()
            self.timer.cancel()
        except Exception:
            # For when a timer does not exist yet
            pass

    def dock(self, dock_name, docking_finished_callback):
        assert dock_name in self.dock_to_wp

        waypoint = self.dock_to_wp[dock_name]
        self.visited_waypoints.append(waypoint)

        docking_finished_callback()
        print(f"[RobotCommandHandle] DOCKING FINISHED to waypoint {waypoint}")

    def _timer_cb(self,
                  waypoints,
                  next_arrival_estimator,
                  path_finished_callback):
        if not self.active:
            return

        if self.current_waypoint_target < len(waypoints):
            self.current_waypoint_target += 1

        if self.updater:
            previous_waypoint = waypoints[self.current_waypoint_target - 1]

            previous_waypoint_graph_idx = previous_waypoint.graph_index
            print("[RobotUpdateHandle] UPDATING ROBOT POSITION:",
                  previous_waypoint_graph_idx)

            self.updater.update_current_waypoint(
                previous_waypoint_graph_idx,
                previous_waypoint.position[2]
            )

            if previous_waypoint_graph_idx is not None:
                self.visited_waypoints.append(previous_waypoint_graph_idx)

        if self.current_waypoint_target < len(waypoints):
            # Again, this waypoint is a plan waypoint! NOT a graph waypoint!!
            waypoint = waypoints[self.current_waypoint_target]
            test_delay = (datetime.timedelta(milliseconds=750)
                          * self.current_waypoint_target)

            node_time = self.node.get_clock().now().nanoseconds / 1e9
            now = datetime.datetime.fromtimestamp(node_time)

            delayed_arrival_time = waypoint.time + test_delay

            # Note: next_arrival_estimator
            # expects a std::chrono::duration,
            # not a std::chrono::steady_clock::time_point

            # The duration represents the time delay from
            # self.node.get_clock().now() till the arrival time,
            # and NOT the time_point that represents that arrival time!!!
            next_arrival_estimator(self.current_waypoint_target,
                                   delayed_arrival_time - now)
        else:
            self.active = False
            self.timer.reset()

            path_finished_callback()
            print("[RobotCommandHandle] PATH FINISHED")


"""
Function which runs a async websocket listener to listen to rmf task
state msg. with the input ayncio.Future, it will set a result when the
provided 'task_id' is completed. Note that this is a blocking function.
Thus it is recommended to run this on a different thread,

:param fut:
    future will also be set to done internally when a task is completed.
:param task_id:
    the target task id to check on completion state
"""
def task_state_observer_fn(fut: asyncio.Future, target_id: str):
    print("Starting task state observer")

    # sample function for user to provide
    def checkstate_callback(msg_type, data):
        print(f" [Observer] Received [{msg_type}] :: "
              f"{data['booking']['id']}: {data['status']}")
        if (data['status'] == 'completed' and
                data['booking']['id'] == target_id):
            print(f" {target_id} COMPLETED, set as DONE!")
            fut.set_result(True)

    print("creating sync rmf_observer class")
    observer = AsyncRmfMsgObserver(
        checkstate_callback,
        msg_filters={
            RmfMsgType.TaskState: []
        }
    )
    observer.spin(fut)
    print("Exit observer function")

def update_observer(callback, fut: asyncio.Future, msg_filters):
    observer = AsyncRmfMsgObserver(callback, msg_filters=msg_filters)
    observer.spin(fut)
