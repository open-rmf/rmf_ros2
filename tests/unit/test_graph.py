from typing import Callable
import datetime

def init():  # Hacky way to avoid flake8 errors
    import os
    import numpy as np

    try:
        rmf_adapter_path = ("~/Desktop/"
                            "pybind_ws/install/rmf_mock_adapter_python/"
                            "lib/python3.6/site-packages")
        os.chdir(os.path.expanduser(rmf_adapter_path))
    except Exception:
        pass

    import rmf_adapter as adpt
    import rmf_adapter.graph as graph
    import rmf_adapter.graph.lane as lane
    global adpt, graph, lane, np


init()

duration = datetime.timedelta(seconds=5)
new_duration = datetime.timedelta(seconds=10)

# Note: Arguments deliberately called out of order to test keyword args

# DOORS =======================================================================
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
lane.LiftDoor("lift_name", "floor_name", duration)
lane.LiftDoorOpen("lift_name", "floor_name", duration)
lane.LiftDoorClose("lift_name", "floor_name", duration)
lift_door = lane.LiftDoor(duration=duration,
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
lane.LiftMove("lift_name", "destination_floor", duration)
lift_move = lane.LiftMove(duration=duration,
                          lift_name="lift_name",
                          destination_floor="destination_floor")

assert lift_move.lift_name == "lift_name"
assert lift_move.destination_floor == "destination_floor"
assert lift_move.duration == duration

lift_move.lift_name = "lift!"
lift_move.destination_floor = "lift!"
lift_move.duration = new_duration
assert lift_move.lift_name == "lift!"
assert lift_move.destination_floor == "lift!"
assert lift_move.duration == new_duration

# DOCK ========================================================================
lane.Dock("dock_name", duration)

dock = lane.Dock(duration=duration, dock_name="dock_name")

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

    def lift_door_open_execute(self, LiftDoorOpen):
        return LiftDoorOpen.lift_name

    def lift_door_close_execute(self, LiftDoorClose):
        return LiftDoorClose.lift_name

    def lift_move_execute(self, LiftMove):
        return LiftMove.lift_name

    def dock_execute(self, Dock):
        return Dock.dock_name


executor = TestExecutor()

assert executor.door_open_execute(
    lane.DoorOpen("door_open_name", duration)
) == "door_open_name"

assert executor.door_close_execute(
    lane.DoorOpen("door_close_name", duration)
) == "door_close_name"

assert executor.lift_door_open_execute(
    lane.LiftDoorOpen("lift_door_open_name", "floor_name", duration)
) == "lift_door_open_name"

assert executor.lift_door_close_execute(
    lane.LiftDoorClose("lift_door_close_name", "floor_name", duration)
) == "lift_door_close_name"

assert executor.lift_move_execute(
    lane.LiftDoorClose("lift_name", "destination_floor", duration)
) == "lift_name"

assert executor.dock_execute(
    lane.Dock("dock_name", duration)
) == "dock_name"


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

assert event.duration() == "duration_ok"
assert event.execute(executor) is type(executor)
assert event.clone() == "clone_ok"

# We can also instantiate EventPtrs for existing events
# e.g. DoorOpen/Close, LiftDoorOpen/Close, LiftMove, Dock
event_ptr = event.dock_make(dock)

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
        return "clone_ok"


three_d = np.reshape([1, 2, 3], [3, 1])
two_d = np.reshape([1, 2], [2, 1])
orientation_constraint = TestOrientationConstraint()
orientation_constraint.apply(three_d, two_d)
orientation_constraint.clone(orientation_constraint)

# You can also instantiate constraints for two C++ implemented constraints
# The bindings resolve overloaded function calls!
acceptable_orientation_constraint_ptr = \
    graph.OrientationConstraint.make([1, 2, 3])
direction_constraint_ptr = \
    graph.OrientationConstraint.make(graph.Direction.Forward, two_d)

assert type(acceptable_orientation_constraint_ptr) == \
    graph.OrientationConstraintPtr
assert type(acceptable_orientation_constraint_ptr) \
    == graph.OrientationConstraintPtr

# VELOCITY_CONSTRAINT =========================================================
class TestVelocityConstraint(graph.VelocityConstraint):
    """
    Use this to create new implementations of velocity constraints!
    You must create a subclass because the original is an abstract class.

    Unfortunately there aren't any that are implemented in C++ currently.
    """
    __test__ = False

    def apply(self, position, course_vector):
        return position, course_vector

    def clone(self):
        # Unfortunately the clone method won't work until we can
        # access the private _ptr member directly
        return self
        # return "clone_ok"

velocity_constraint = TestVelocityConstraint()
velocity_constraint.apply(three_d, two_d)
velocity_constraint_ptr = velocity_constraint.clone()

velocity_constraint_ptr

# NODE ========================================================================
node_one = lane.Node(1, acceptable_orientation_constraint_ptr)
node_two = lane.Node(2,
                     event_ptr,
                     acceptable_orientation_constraint_ptr,
                     # Note, the velocity constrain there might as well be null.
                     graph.VelocityConstraintPtr())

# Once you've instantiated your nodes, you can manipulate their members
# as you wish

node_one.orientation_constraint.clone()
node_one.orientation_constraint.apply(three_d, two_d)


# GRAPH =======================================================================
# Finally!
map_name = "rawr_map"
rawr_graph = graph.Graph()

rawr_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
rawr_graph.add_waypoint(map_name, [0.0, -5.0], True)  # 1
rawr_graph.add_waypoint(map_name, [5.0, -5.0], True)  # 2
rawr_graph.add_waypoint(map_name, [-10.0, 0])  # 3
rawr_graph.add_waypoint(map_name, [-5.0, 0.0], True)  # 4
rawr_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
rawr_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
rawr_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
rawr_graph.add_waypoint(map_name, [0.0, 5.0], True)  # 8
rawr_graph.add_waypoint(map_name, [5.0, 5.0], True)  # 9
rawr_graph.add_waypoint(map_name, [0.0, 10.0])  # 10

# Remember waypoint 0 counts as one waypoint also!
assert rawr_graph.num_waypoints == 11

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

# Normal graph.add_lane() also works!
for _lane in lanes:
    rawr_graph.add_bidir_lane(*_lane)

assert rawr_graph.num_lanes == 24
