# Requirements

gcc >= 8, There is a bug in gcc 7 that causes segfault when using rxcpp under certain scenarios.


# Testing read\_only\_fleet\_adapter

The read_only adapter is tested for four scenarios:
1. `test_insert` : the first FleetState message is sent to the adapter which should create an entry in its ScheduleEntries and trigger a submit_trajectory service to the schedule. The fleet is initialized with a single robot at the origin and a `path` with two `locations`. The robot is predicted to reach the first path location which is 70m along x-axis after 100s. The second location is at 140m along x-axis following another 100s. 
2. `test_advance` : simulates a scenario where the robot is ahead of the previously known schedule. 
3. `test_delay` : simulates a scenario where the robot is delayed by 20s
4. `test_replace` : simulates a scenario where two additonal locations are added to the path 

## Test Setup

Testing the read_only_fleet_adapter requires 5 terminals and each command run in the same numbered sequence.

1. Start the rmf_schedule node 
```ros2 run rmf_traffic_ros2 rmf_traffic_schedule```

2. Start the [rmf_schedule_visualizer](https://github.com/osrf/rmf_schedule_visualizer) node to print out trajectory data in a mirror. 
```ros2 run rmf_schedule_visualizer schedule_visualizer -n viz```

3. Start the read_only fleet adapter node
```ros2 run rmf_fleet_adapter read_only -f fleet1```

4. Start the testing node 
```ros2 run rmf_fleet_adapter test_read_only_adapter```

5. The tests
```ros2 topic pub /test_adapter_fleet1/cmd std_msgs/msg/String "{data: <TEST_NAME>}" --once ```

where <TEST_NAME> is either test_insert, test_advance, test_delay or test_replace

## Test Procedure

With the first four terminals setup, the desired test can be run by executing the command in the fifth terminal. 

```Note: To run tests 2-4, a test_insert command has to be issued first through the same terminal```

