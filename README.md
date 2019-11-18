# rmf\_core
The `rmf_core` packages provide the centralized functions of
the Robotics Middleware Framework (RMF). These include task
queuing, conflict-free resource scheduling, utilities to help
create robot fleet adapters, and so on.

All packages in this repo will be written in ROS 2.

To create a useful deployment, `rmf_core` must be connected
to many other subsystems, as shown in the following diagram.

![integration-diagram](/docs/rmf_core_integration_diagram.png)

## Interfacing with rmf\_core

There are several interface points with RMF core, as shown in the arrows
between the blue central box and the orange boxes in the diagram above. The
goal of these interfaces is to create a "narrow" and simple set of messages
that allow `rmf_core` to integrate with the following elements of a
deployment:

### Robot fleet integration

The `rmf_fleet_msgs` package contains four messages and is
intended to carry the interactions between `rmf_core` and a vendor-provided
(typically proprietary) fleet manager for a collection of robots. It is
expected that a RMF deployment will consist of multiple robot fleets, often
operating at different levels of RMF integration: for example, one fleet may
only be willing to supply `FleetState` messages (observation-only) whereas
another fleet in the same facility may be willing to follow RMF
`DestinationRequest` messages. RMF is specifically designed to allow this
type of "mixed levels of control" and to plan accordingly.

 * `rmf_fleet_msgs/FleetState` consists of a list of `rmf_fleet_msgs/RobotState` messages, which contains the state of a particular robot. This includes the level of the facility the robot is on, its X- and Y- offset (in meters) from the origin of that level's map, its current destination and path (if known), and so on.
 * `rmf_fleet_msgs/DestinationRequest` is a request for a particular robot
to go to a particular destination.
 * `rmf_fleet_msgs/ModeRequest` is a request for a particular robot to change modes, for example, from `MOVING` to `PAUSED`, in order to preserve spatial separation between robots of different fleets.
 * `rmf_fleet_msgs/PathRequest` is a request for a particular robot to follow a particular path.

### Door integration

The `rmf_door_msgs` package contains two messages. This interface allows
RMF to open and close motorized doors for robots as they move throughout a
facility.
 * `rmf_door_msgs/DoorState` messages are periodically sent by door controllers
to `rmf_core`. These messages express the current mode of the door as `CLOSED`, `MOVING`, or `OPEN`
 * `rmf_door_msgs/DoorRequest` messages are sent from `rmf_core` to doors when
they need to open or close for robot operations.
