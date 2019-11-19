# Example - running a simulated read-only fleet

A read-only fleet does not allow any of the `rmf_core` systems to control it in any way. It only reports its status through `rmf_fleet_msgs/FleetState`, expecting other `rmf_core` nodes to divert other fleets' traffic away from its intended routes.

## Instructions

After building `rmf_core`, we start the scheduler node.

```
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

Next, in a separate terminal, we start a fleet adapter with the name `fake_fleet` and also provide the path to a generated graph file in `.yaml` format. The fleet adapter is read-only by default, unless specified with the flag `-c`.

```
ros2 run rmf_fleet_adapter rmf_fleet_adapter -g <PATH_TO_GRAPH> -f fake_fleet
```

In the last terminal, we start the fake fleet driver/manager, which simulates a round robot forever going diagonally in space.

```
ros2 run rmf_fleet_adapter fake_fleet
```
