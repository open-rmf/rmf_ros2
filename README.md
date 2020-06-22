# rmf_fleet_adapter_python
Python bindings for the fleet adapters for [rmf_core](https://github.com/osrf/rmf_core)

Specifically:

- [rmf_fleet_adapter](https://github.com/osrf/rmf_core/tree/develop/rmf_fleet_adapter)



## Pre-Requisites

- ROS2
- [rmf_core](https://github.com/osrf/rmf_core) must be in your workspace



## Installation

- Place the `pybind_ament` and `rmf_fleet_adapter_python` packages in your ROS2 workspace, and run `colcon build` as per normal, remembering to source the workspace.
- The bindings will then be available as importable `Python3` packages!

Do read the individual `README.md` files in each of the relevant packages for writeups on using the packages.



## Description

> Fleet adapters allow for interactions between `rmf_core` and robot fleets.
>
> High level pathing or task assignments can be issued to robots from `rmf_core`, and a fleet adapter will take those assignments and handle it in an implementation specific manner to direct individual robots to execute certain actions while monitoring and updating `rmf_core` on their high-level states via `rmf_core` robot update handles.
>
> For more information it is helpful to look at the docs for [rmf_core](https://github.com/osrf/rmf_core)

### Usage

The individual `README.md` files in the relevant packages will go into more detail or talk about certain gotchas that should be kept in mind when using these bindings.

But barring that, you may also peruse the existing scripts in each package's `scripts` directory, or the unit tests in `test`.

The bound APIs are almost identical to the `rmf_core` ones as well, so if the written documentation for these packages, and the Python usage examples in the scripts or tests are insufficient, you can also look into the relevant C++ implementations as a last resort.

Alternatively, you may also explore any interfaces with:

```python
# To list any available members or methods
dir(adpt.<WHATEVER_CLASS_YOU_WANT_TO_INSPECT>)

# To see function signatures
help(adpt.<WHATEVER_CLASS_YOU_WANT_TO_INSPECT>)
```



### Binding Details

This package contains Python bindings to C++ code that is found in `rmf_core` and its associated packages. It was implemented using [Pybind11](https://pybind11.readthedocs.io/).

Do note that as a result of that, if one needs to inspect the source, there are no Python implementations of any of the bound code, and it would be advisable to look directly into the appropriate [rmf_fleet_adapter](https://github.com/osrf/rmf_core/tree/develop/rmf_fleet_adapter) package, that contains the C++ code.. Additionally, C++ objects will get instantiated (generally) as references that can be used or called using the appropriate bound methods on the Python side.

Some of these bindings allow you to **override virtual methods that would normally be called in the C++ side with Python code** that you write in your Python scripts!



### Features

There are enough functionalities bound to implement your own fleet adapters to communicate with the rest of the `rmf_core` systems in the ROS2 graph. Allowing you to:

- `Adapter/MockAdapter`: **Communicate with rmf_core by adding fleets or requesting deliveries**
- `RobotCommandHandle`: **Specify custom robot command handles** that can take commands from the `rmf_core` packages and implement or execute the following robot behaviors:
  - Docking
  - Path assignment
  - Starting
  - Stopping
  - RobotUpdateHandle: **Update robot statuses** 
- `Executor`: **Implement new event handlers** for:
  - Docking
  - Door operation
  - Lift door operation
  - Lift moving
- `Graph`: **Specify map geometries as they relate to waypoints and lanes**
- `VehicleTraits`: **Specify robot vehicle traits**, including:
  - Footprint
  - Kinematic limits

Additionally, when you pair this with `rclpy`, you can interact with the other ROS2 message interfaces provided by `rmf_core`, and essentially write your own fleet adapter in Python!