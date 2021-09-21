import numpy as np
import datetime

import rmf_adapter.graph as graph
import rmf_adapter.graph.lane as lane

duration = datetime.timedelta(seconds=5)
new_duration = datetime.timedelta(seconds=10)

# Note: Arguments sometimes deliberately called out of order
# to test keyword args


# DOORS =======================================================================
def test_doors():
    lane.Door("door_name", duration)
    lane.DoorOpen("door_name", duration)
    lane.DoorClose("door_name", duration)

    door = lane.Door(duration=duration, name="door_name")

    assert door.name == "door_name"
    assert door.duration == duration

    door.name = "door!"
    door.duration = new_duration
    assert door.name == "door!"
    assert door.duration == new_duration


# LIFTS =======================================================================
# Doors
def test_lift_doors():
    lane.LiftSessionBegin("lift_name", "floor_name", duration)
    lane.LiftDoorOpen("lift_name", "floor_name", duration)
    lane.LiftSessionEnd("lift_name", "floor_name", duration)

    lift_door = lane.LiftSessionBegin(duration=duration,
                              lift_name="lift_name",
                              floor_name="floor_name")
    assert lift_door.lift_name == "lift_name"
    assert lift_door.floor_name == "floor_name"
    assert lift_door.duration == duration

    lift_door.lift_name = "lift!"
    lift_door.floor_name = "lift!"
    lift_door.duration = new_duration
    assert lift_door.lift_name == "lift!"
    assert lift_door.floor_name == "lift!"
    assert lift_door.duration == new_duration


# Move
def test_lift_moves():
    lane.LiftMove("lift_name", "destination_floor", duration)
    lift_move = lane.LiftMove(lift_name="lift_name",
                              floor_name="floor_name",
                              duration=duration)

    assert lift_move.lift_name == "lift_name"
    assert lift_move.floor_name == "floor_name"
    assert lift_move.duration == duration

    lift_move.lift_name = "lift!"
    lift_move.floor_name = "lift!"
    lift_move.duration = new_duration
    assert lift_move.lift_name == "lift!"
    assert lift_move.floor_name == "lift!"
    assert lift_move.duration == new_duration


# DOCK ========================================================================
dock = lane.Dock(duration=duration, dock_name="dock_name")


def test_dock():
    lane.Dock("dock_name", duration)

    assert dock.dock_name == "dock_name"
    assert dock.duration == duration

    dock.dock_name = "dock!"
    dock.duration = new_duration
    assert dock.dock_name == "dock!"
    assert dock.duration == new_duration


# EXECUTOR ====================================================================
class TestExecutor(lane.Executor):
    __test__ = False

    def door_open_execute(self, DoorOpen):
        return DoorOpen.name

    def door_close_execute(self, DoorClose):
        return DoorClose.name

    def lift_session_begin_execute(self, LiftSessionBegin):
        return LiftSessionBegin.lift_name

    def lift_door_open_execute(self, LiftDoorOpen):
        return LiftDoorOpen.lift_name

    def lift_door_close_execute(self, LiftDoorClose):
        return LiftDoorClose.lift_name

    def lift_session_end_execute(self, LiftSessionEnd):
        return LiftSessionEnd.lift_name

    def lift_move_execute(self, LiftMove):
        return LiftMove.lift_name

    def dock_execute(self, Dock):
        return Dock.dock_name

    def wait_execute(Self, Wait):
        return Wait.duration


executor = TestExecutor()


def test_executor():
    assert executor.door_open_execute(
        lane.DoorOpen("door_open_name", duration)
    ) == "door_open_name"

    assert executor.door_close_execute(
        lane.DoorClose("door_close_name", duration)
    ) == "door_close_name"

    assert executor.lift_session_begin_execute(
        lane.LiftSessionBegin(
            "lift_session_begin_name", "floor_name", duration)
    ) == "lift_session_begin_name"

    assert executor.lift_door_open_execute(
        lane.LiftDoorOpen("lift_session_end_name", "floor_name", duration)
    ) == "lift_session_end_name"

    assert executor.lift_session_end_execute(
        lane.LiftSessionEnd("lift_session_end_name", "floor_name", duration)
    ) == "lift_session_end_name"

    assert executor.lift_move_execute(
        lane.LiftSessionEnd("lift_move_name", "floor_name", duration)
    ) == "lift_move_name"

    assert executor.dock_execute(
        lane.Dock("dock_name", duration)
    ) == "dock_name"

    assert executor.wait_execute(
        lane.Wait(duration)
    ) == duration


# EVENT =======================================================================
class TestEvent(lane.Event):
    """Create a custom event."""
    __test__ = False

    def duration(self):
        return "duration_ok"

    def execute(self, Executor):
        return type(Executor)

    def clone(self):
        return "clone_ok"


event = TestEvent()

# We can also instantiate EventPtrs for existing events
# e.g. DoorOpen/Close, LiftDoorOpen/Close, LiftMove, Dock
event_ptr = event.dock_make(dock)


def test_event():
    assert event.duration() == "duration_ok"
    assert event.execute(executor) is type(executor)
    assert event.clone() == "clone_ok"

# LANE ========================================================================
# NO TESTS SINCE WE DON'T HAVE EXPLICIT CONSTRUCTION

# WAYPOINT ====================================================================
# NO TESTS SINCE WE DON'T HAVE EXPLICIT CONSTRUCTION


# ORIENTATION_CONSTRAINT ======================================================
class TestOrientationConstraint(graph.OrientationConstraint):
    """
    Use this to create new implementations of orientation constraints
    beyond the ones already implemented in C++!

    (e.g. acceptable_orientation_constraint, direction_constraint)
    """

    __test__ = False

    def apply(self, position, course_vector):
        """
        Return True or False depending on if the constraint is satisfied.
        """
        return True

    def clone(self, OrientationConstraint):
        return True


three_d = np.reshape([1, 2, 3], [3, 1])
two_d = np.reshape([1, 2], [2, 1])

# You can also instantiate constraints for two C++ implemented constraints
# The bindings resolve overloaded function calls!
acceptable_orientation_constraint_ptr = (
    graph.OrientationConstraint.make([1, 2, 3]))
direction_constraint_ptr = (
    graph.OrientationConstraint.make(graph.Direction.Forward, two_d))


def test_orientation_constraint():
    orientation_constraint = TestOrientationConstraint()
    assert orientation_constraint.apply(three_d, two_d)
    assert orientation_constraint.clone(orientation_constraint)

    assert (type(acceptable_orientation_constraint_ptr)
            == graph.OrientationConstraintPtr)
    assert (type(direction_constraint_ptr)
            == graph.OrientationConstraintPtr)


# NODE ========================================================================
node_one = lane.Node(1, acceptable_orientation_constraint_ptr)
node_two = lane.Node(2,
                     event_ptr,
                     acceptable_orientation_constraint_ptr)


# Once you've instantiated your nodes, you can manipulate their members
# as you wish
def test_node():
    node_one.orientation_constraint.clone()
    node_one.orientation_constraint.apply(three_d, two_d)


# GRAPH =======================================================================
# Finally!
map_name = "rawr_map"
rawr_graph = graph.Graph()


def test_graph():
    # Test waypoints
    rawr_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    rawr_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
    rawr_graph.add_waypoint(map_name, [5.0, -5.0])  # 2
    rawr_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    rawr_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
    rawr_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    rawr_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    rawr_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    rawr_graph.add_waypoint(map_name, [0.0, 5.0]) \
        .set_holding_point(True)  # 8
    rawr_graph.add_waypoint(map_name, [5.0, 5.0]) \
        .set_passthrough_point(True)  # 9
    rawr_graph.add_waypoint(map_name, [0.0, 10.0]) \
        .set_parking_spot(True)  # 10

    assert rawr_graph.get_waypoint(8).holding_point
    assert rawr_graph.get_waypoint(9).passthrough_point
    assert rawr_graph.get_waypoint(10).parking_spot

    assert not rawr_graph.get_waypoint(10).holding_point
    assert not rawr_graph.get_waypoint(8).parking_spot

    # Remember waypoint 0 counts as one waypoint also!
    assert rawr_graph.num_waypoints == 11

    # Test keys
    assert not rawr_graph.keys and type(rawr_graph.keys) is dict

    assert rawr_graph.add_key("rawr", 0)
    assert rawr_graph.add_key("rawr!", 0)
    assert not rawr_graph.add_key("rawr!", 1)
    assert len(rawr_graph.keys) == 2 and "rawr" in rawr_graph.keys

    assert rawr_graph.remove_key("rawr!")
    assert not rawr_graph.remove_key("rawr!")
    assert len(rawr_graph.keys) == 1 and "rawr!" not in rawr_graph.keys

    assert rawr_graph.set_key("rawr!", 1)
    assert rawr_graph.set_key("rawr!", 0)
    assert rawr_graph.set_key("rawr", 1)
    assert len(rawr_graph.keys) == 2 and "rawr!" in rawr_graph.keys

    # Test key retrieval
    assert rawr_graph.find_waypoint("rawr!") is rawr_graph.get_waypoint(0)
    assert rawr_graph.find_waypoint("rawr") is rawr_graph.get_waypoint(1)

    # Waypoints 1, 2, 4, 8, 9 are holding positions
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

    # You must first add enough waypoints so as to avoid the assertion error
    # when calling add_lane!!
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

    # test speed limit std::optional lane property
    properties = lane.Properties()
    properties.speed_limit = 10.0
    # Test add_lane
    added_lane = rawr_graph.add_lane(lane.Node(0),
                        lane.Node(1), properties)
    assert added_lane.properties.speed_limit == 10.0
    assert rawr_graph.num_lanes == 1

    # Test add_bidir_lane
    for _lane in lanes:
        rawr_graph.add_bidir_lane(*_lane)
    assert rawr_graph.num_lanes == 25

    # Test add_dock_lane
    for _lane in lanes:
        rawr_graph.add_dock_lane(*_lane, "test")

    assert rawr_graph.num_lanes == 49
