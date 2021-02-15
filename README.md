# rmf_fleet_adapter_python
Python bindings for the fleet adapters for [rmf_core](https://github.com/osrf/rmf_core)
( Specifically [rmf_fleet_adapter](https://github.com/osrf/rmf_core/tree/develop/rmf_fleet_adapter) )

For API Docs, see [the documentation](https://osrf.github.io/rmf_fleet_adapter_python).

## Pre-Requisites

- ROS2
- [rmf_core](https://github.com/osrf/rmf_core) must be in your workspace


## Installation

```bash
source install/setup.bash
colcon build --packages-select rmf_fleet_adapter_python
```

## Running Examples

Integration test script for rmf loop and delivery task:

```shell
ros2 run rmf_adapter_python test_adapter
```

You may then probe the effects on the ROS2 graph by subscribing to the following topics with `ros2 topic echo <TOPIC_NAME>`:
- <TOPIC_NAME>: `/dispenser_requests`, `/dispenser_results`, `ingestor_requests`, `ingestor_results`, `/task_summaries`

###  Traffic Light Example
This will showcase an example of having 2 "traffic light" robots using RMF.

```bash
# First Terminal
ros2 run rmf_fleet_adapter_python schedule_blockade_nodes

# Second Terminal
ros2 run rmf_fleet_adapter_python traffic_light
```

For more details, please read [this README](/rmf_fleet_adapter_python/README.md).

### Contributions

- Before making a PR, please make sure that the doc is up to date, generate docs via: `cd docs && bash gen_docs.sh`
- More bindings, tests, and examples are welcome!
