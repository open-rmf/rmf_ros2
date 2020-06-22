import rmf_dispenser_msgs.msg as dispenser_msgs
from rmf_task_msgs.msg import TaskSummary

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from functools import partial
import datetime
import time

###############################################################################
# PARAMS
###############################################################################

pickup_name = "pickup"
dropoff_name = "dropoff"

quiet_dispenser_name = "quiet"
flaky_dispenser_name = "flaky"

###############################################################################
# CONSTS
###############################################################################

DISPENSER_RESULT_ACKNOWLEDGED = 0
DISPENSER_RESULT_SUCCESS = 1
DISPENSER_RESULT_FAILED = 2

DISPENSER_STATE_IDLE = 0
DISPENSER_STATE_BUSY = 1
DISPENSER_STATE_OFFLINE = 2

TASK_STATE_QUEUED = 0
TASK_STATE_ACTIVE = 1
TASK_STATE_COMPLETED = 2
TASK_STATE_FAILED = 3


###############################################################################
# CLASSES
###############################################################################

class MockQuietDispenser(Node):
    """
    This mock dispenser will not publish any states; it will only publish a
    successful result and nothing else.

    Note: This dispenser is just using pure rclpy, there is no usage of
    rmf_adapter here
    """
    def __init__(self, name):
        super().__init__(name)

        # Variables
        self.reset(name)

        # Pub-sub
        self.result_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            'dispenser_results',
            1
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            'dispenser_requests',
            self._process_request_cb,
            1
        )

    def reset(self, name=None):
        if name is not None:
            self.name = name

        self.tasks = {}
        self.success_flag = False

        try:
            self.timer.reset()
            self.timer.cancel()
        except Exception:
            pass

        self.timer = None

    def _process_request_cb(self, msg):
        # Process request if addressed to this dispenser
        if msg.target_guid != self.name:
            return

        print("[MockQuietDispenser] REQUEST RECEIVED")
        if self.tasks.get(msg.request_guid):
            print("QUIET DISPENSER SUCCESS")
            status = DISPENSER_RESULT_SUCCESS
        else:
            self.tasks[msg.request_guid] = False
            status = DISPENSER_RESULT_ACKNOWLEDGED

            self.timer = self.create_timer(
                0.01,
                partial(self._timer_cb, msg=msg)
            )

        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = status
        result.source_guid = self.name
        result.request_guid = msg.request_guid

        self.result_pub.publish(result)

    def _timer_cb(self, msg):
        if not self.timer:
            return

        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = DISPENSER_RESULT_SUCCESS
        result.source_guid = self.name
        result.request_guid = msg.request_guid

        self.result_pub.publish(result)
        self.success_flag = True
        self.timer.cancel()


class MockFlakyDispenser(Node):
    """
    This mock dispenser will not publish any results; it will only publish
    states. This is representative of network issues where a result might not
    actually arrive, but the state heartbeats can still get through.

    Note: This dispenser is just using pure rclpy, there is no usage of
    rmf_adapter here
    """
    class RequestEntry():
        def __init__(self, request, publish_count):
            self.request = request
            self.publish_count = publish_count

    def __init__(self, name):
        super().__init__(name)

        # Variables
        self.name = name
        self.request_queue = []

        self.success_flag = False
        self._fulfilled_flag = False

        # Pub-sub
        self.state_pub = self.create_publisher(
            dispenser_msgs.DispenserState,
            'dispenser_states',
            1
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            'dispenser_requests',
            self._process_request_cb,
            1
        )

        self._timer = self.create_timer(
            0.1,
            self._timer_cb
        )

    def reset(self, name=None):
        if name is not None:
            self.name = name

        self.request_queue = []

        self.success_flag = False
        self._fulfilled_flag = False

    def _process_request_cb(self, msg):
        # Add requests to queue if addressed to this dispenser
        if msg.target_guid != self.name:
            return

        print("[MockFlakyDispenser] REQUEST RECEIVED")
        self.request_queue.append(self.RequestEntry(msg, 0))

    def _timer_cb(self):
        msg = dispenser_msgs.DispenserState()
        msg.guid = self.name

        if not self.request_queue:  # If empty
            msg.mode = DISPENSER_STATE_IDLE
        else:
            msg.mode = DISPENSER_STATE_BUSY

        msg.time = self.get_clock().now().to_msg()
        msg.seconds_remaining = 0.1

        # Increment publish_count of all requests in queue
        for req in self.request_queue:
            msg.request_guid_queue.append(req.request.request_guid)
            req.publish_count += 1

        initial_count = len(self.request_queue)

        # Remove all requests with publish count > 2
        self.request_queue = list(filter(lambda x: x.publish_count > 2,
                                  self.request_queue))

        # If any requests were removed, set flags to True
        if len(self.request_queue) < initial_count:
            if not self._fulfilled_flag:
                self._fulfilled_flag = True
                self.success_flag = True

        self.state_pub.publish(msg)


class MockRobotCommand(adpt.RobotCommandHandle):
    class EventListener(graph.lane.Executor):
        def __init__(self, dock_to_wp, wp):
            graph.lane.Executor.__init__(self)

            self.dock_to_wp = dock_to_wp
            self.wp = wp

        def dock_execute(self, dock):
            self.dock_to_wp[dock.dock_name] = self.wp
            print("DOCK EVENT EXECUTED FOR DOCK:", dock.dock_name)

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

        self.updater = None

        self.active = False
        self.node = node
        self.timer = None
        self.current_waypoint_target = 0
        self.dockings = {}
        self.visited_waypoints = {}
        self.dock_to_wp = {}

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

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,  # function!
                        path_finished_callback):
        print("\n[RobotCommandHandle] Setting new path of %d waypoints..."
              % len(waypoints))

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
        self.timer.reset()
        self.timer.cancel()

    def dock(self, dock_name, docking_finished_callback):
        assert dock_name in self.dock_to_wp

        # For both dockings and visited waypoints, increment the associated
        # keyed values by 1. Or start it off at 1 if it doesn't exist yet.
        self.dockings[dock_name] = self.dockings.get(dock_name, 0) + 1

        waypoint = self.dock_to_wp[dock_name]
        self.visited_waypoints[waypoint] = (
            self.visited_waypoints.get(waypoint, 0) + 1)

        docking_finished_callback()
        print("[RobotCommandHandle] DOCKING FINISHED")

    def _timer_cb(self,
                  waypoints,
                  next_arrival_estimator,
                  path_finished_callback):
        if not self.active:
            return

        if self.current_waypoint_target < len(waypoints):
            self.current_waypoint_target += 1

        if self.updater:
            # This waypoint is a plan waypoint, NOT graph waypoint!!
            previous_waypoint = waypoints[self.current_waypoint_target - 1]

            if previous_waypoint.graph_index.has_value:
                print("[RobotUpdateHandle] UPDATING ROBOT POSITION:",
                      previous_waypoint.graph_index.value)

                self.updater.update_position(
                    previous_waypoint.graph_index.value,
                    previous_waypoint.position[2]
                )
                self.visited_waypoints[previous_waypoint.graph_index.value] = (
                    self.visited_waypoints
                        .get(previous_waypoint.graph_index.value, 0)
                    + 1
                )
            else:
                print("[RobotUpdateHandle] UPDATING ROBOT POSITION DEFAULT:",
                      previous_waypoint.position)
                # TODO(CH3): NOTE(CH3): Confirm this magic string is wanted
                self.updater.update_position("test_map",
                                             previous_waypoint.position)

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


def main():
    # INIT RCL ================================================================
    rclpy.init()
    adpt.init_rclcpp()

    # INIT GRAPH ==============================================================
    map_name = "test_map"
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

    """
                     10(D)
                      |
                      |
                      8------9
                      |      |
                      |      |
        3------4------5------6------7(D)
                      |      |
                      |      |
                      1------2
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

    test_graph.add_key(pickup_name, 7)
    test_graph.add_key(dropoff_name, 10)

    assert len(test_graph.keys) == 2 and pickup_name in test_graph.keys \
        and dropoff_name in test_graph.keys

    # INIT FLEET ==============================================================
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(0.7, 0.3),
                                        angular=traits.Limits(1.0, 0.45),
                                        profile=profile)

    # Manages delivery or loop requests
    adapter = adpt.MockAdapter("TestDeliveryAdapter")
    fleet = adapter.add_fleet("test_fleet", robot_traits, test_graph)
    fleet.accept_delivery_requests(lambda x: True)

    cmd_node = Node("RobotCommandHandle")

    starts = [plan.Start(adapter.now(),
                         0,
                         0.0)]

    # Lambda to insert an adapter
    def updater_inserter(handle_obj, updater):
        handle_obj.updater = updater

    # Manages and executes robot commands
    robot_cmd = MockRobotCommand(cmd_node, test_graph)

    fleet.add_robot(robot_cmd,
                    "T0",
                    profile,
                    starts,
                    partial(updater_inserter, robot_cmd))

    # INIT TASK OBSERVER ======================================================
    at_least_one_incomplete = False
    completed = False

    def task_cb(msg):
        nonlocal at_least_one_incomplete
        nonlocal completed

        if msg.state == TASK_STATE_COMPLETED:
            completed = True
            at_least_one_incomplete = False
        else:
            completed = False
            at_least_one_incomplete = True

    task_node = rclpy.create_node("task_summary_node")
    task_node.create_subscription(TaskSummary,
                                  "task_summaries",
                                  task_cb,
                                  10)

    # INIT DISPENSERS =========================================================
    quiet_dispenser = MockQuietDispenser(quiet_dispenser_name)
    flaky_dispenser = MockFlakyDispenser(flaky_dispenser_name)

    # FINAL PREP ==============================================================
    rclpy_executor = SingleThreadedExecutor()
    rclpy_executor.add_node(task_node)
    rclpy_executor.add_node(cmd_node)
    rclpy_executor.add_node(quiet_dispenser)
    rclpy_executor.add_node(flaky_dispenser)

    last_quiet_state = False
    last_flaky_state = False

    # GO! =====================================================================
    adapter.start()

    print("# SENDING NEW REQUEST ############################################")
    request = adpt.type.CPPDeliveryMsg("test_delivery",
                                       pickup_name,
                                       quiet_dispenser_name,
                                       dropoff_name,
                                       flaky_dispenser_name)
    quiet_dispenser.reset()
    flaky_dispenser.reset()
    adapter.request_delivery(request)
    rclpy_executor.spin_once(1)

    for i in range(1000):
        if quiet_dispenser.success_flag != last_quiet_state:
            last_quiet_state = quiet_dispenser.success_flag
            print("== QUIET DISPENSER FLIPPED SUCCESS STATE ==",
                  last_quiet_state)

        if flaky_dispenser.success_flag != last_flaky_state:
            last_flaky_state = flaky_dispenser.success_flag
            print("== FLAKY DISPENSER FLIPPED SUCCESS STATE ==",
                  last_flaky_state)

        if quiet_dispenser.success_flag and flaky_dispenser.success_flag:
            rclpy_executor.spin_once(1)
            break

        rclpy_executor.spin_once(1)
        time.sleep(0.2)

    print("\n== DEBUG TASK REPORT ==")
    print("Visited waypoints:", robot_cmd.visited_waypoints)
    print("At least one incomplete:", at_least_one_incomplete)
    print("Completed:", completed)
    print()

    assert len(robot_cmd.visited_waypoints) == 6
    assert all([x in robot_cmd.visited_waypoints for x in [0, 5, 6, 7, 8, 10]])
    assert at_least_one_incomplete

    # Uncomment this to send a second request.

    # TODO(CH3):
    # But note! The TaskManager has to be fixed first to allow task pre-emption
    # print("# SENDING NEW REQUEST ##########################################")
    # request = adpt.type.CPPDeliveryMsg("test_delivery_two",
    #                                    dropoff_name,
    #                                    flaky_dispenser_name,
    #                                    pickup_name,
    #                                    quiet_dispenser_name)
    # quiet_dispenser.reset()
    # flaky_dispenser.reset()
    # adapter.request_delivery(request)
    # rclpy_executor.spin_once(1)
    #
    # for i in range(1000):
    #     if quiet_dispenser.success_flag != last_quiet_state:
    #         last_quiet_state = quiet_dispenser.success_flag
    #         print("== QUIET DISPENSER FLIPPED SUCCESS STATE ==",
    #               last_quiet_state)
    #
    #     if flaky_dispenser.success_flag != last_flaky_state:
    #         last_flaky_state = flaky_dispenser.success_flag
    #         print("== FLAKY DISPENSER FLIPPED SUCCESS STATE ==",
    #               last_flaky_state)
    #
    #     if quiet_dispenser.success_flag and flaky_dispenser.success_flag:
    #         rclpy_executor.spin_once(1)
    #         break
    #
    #     rclpy_executor.spin_once(1)
    #     time.sleep(0.2)
    #
    # print("\n== DEBUG TASK REPORT ==")
    # print("Visited waypoints:", robot_cmd.visited_waypoints)
    # print("At least one incomplete:", at_least_one_incomplete)
    # print("Completed:", completed)
    # print()

    task_node.destroy_node()
    cmd_node.destroy_node()
    quiet_dispenser.destroy_node()
    flaky_dispenser.destroy_node()

    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
