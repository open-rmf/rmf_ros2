A simple node that takes in a yaml file that describes a schedule for charger
usage in an RMF scenario. The node will watch the clock and publish the
appropriate commands to change the chargers of the robots.

The format for the schedule looks like this:
```
"my_fleet_name":
    "00:00": { "robot_1": "charger_A", "robot_2": "charger_B", "robot_3": "queue_A" }
    "01:55": { "robot_1": "queue_B" }
    "02:00": { "robot_3": "charger_A", "robot_1": "queue_A" }
    "03:55": { "robot_2": "queue_B" }
    "04:00": { "robot_1": "charger_B", "robot_2": "queue_A" }
parking: ["queue_A", "queue_B"]
```

The time format is `"HH:MM"` where `HH` ranges from `00` to `23` and `MM` ranges
from `00` to `59`. Note that quotes are important because otherwise the yaml
format may confuse the meaning of the colon `:`.

The schedule will cycle every 24 hours.

For each timestamp, only robots that are explicitly mentioned will have their
dedicated charger changed. **It is the responsibility of the schedule file author
to make sure that two robots are never assigned the same charger at the same
time.** Failing to ensure this may cause traffic and task management to misbehave.

When run in simulation mode (`--ros-args --use-sim-time`), the time `00:00` in
the schedule will correspond to `t=0.0` in simulation time.

When run without sim time on, the hours and minutes will correspond to the local
timezone of the machine that the node is run on. To choose a specific timezone
instead of using the system's local timzeone, use the `--timezone` argument and
provide the desired [TZ identifier](https://en.wikipedia.org/wiki/List_of_tz_database_time_zones)
string.

It is advisable that you always put a `00:00` entry that indicates **all** of
the intended charger assignments at midnight. When the node is launched, it will
move through the schedule from the earliest entry up until the last relevant one
and issue an initial charger assignment message based on what the assignments
would have been if the schedule had run from `00:00`.

If any of the waypoints are parking points instead of charging points, put them
into a list called `parking`. Note that this node does not support the existence
of a fleet with the name `parking`.
