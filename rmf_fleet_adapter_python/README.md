# rmf_fleet_adapter_python
Python bindings for [rmf_fleet_adapter](https://github.com/open-rmf/rmf_ros2/tree/master/rmf_fleet_adapter)



## Introduction

The `rmf_fleet_adapter` package defines C++ classes that can interact with `rmf_core`. Specifically an `Adapter` and a `MockAdapter` class. This package provides bindings for both and most of their dependencies.

The main difference between `rmf_fleet_adapter` and `rmf_mock_adapter` is the fact that `rmf_fleet_adapter` objects **can interact with the ROS2 graph**.

The `MockAdapter` and `Adapter` both do this. But the difference is that `MockAdapter` can be instantiated with no issues, whereas `Adapter` will block to wait for a scheduler node from `rmf_core` to make its presence known.



## Installation

- Place the `pybind_ament` and `rmf_fleet_adapter_python` packages in your ROS2 workspace, and run `colcon build` as per normal.



## Running Examples

Integration test script for rmf loop and delivery task:

```shell
ros2 run rmf_fleet_adapter_python test_adapter.py
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

## Notes
- The py api bindings are mainly experimental. Use with caution.
- Current CI and docs gen are using rolling release of RMF
- More bindings, tests, and examples are welcome!

## Using the Bindings

Ensure that you have built this package and sourced its environment. Then it is as simple as importing the module containing the Python bindings!

```python
import rmf_adapter as adpt

# You may then check the available bindings
print(dir(adpt))
```

## Description

> Fleet adapters allow for interactions between `rmf_core` and robot fleets.
>
> High level pathing or task assignments can be issued to robots from `rmf_core`, and a fleet adapter will take those assignments and handle it in an implementation specific manner to direct individual robots to execute certain actions while monitoring and updating `rmf_core` on their high-level states via `rmf_core` robot update handles.
>
> For more information it is helpful to look at the docs for [rmf_core](https://github.com/osrf/rmf_core)

### Usage

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

---

## Creating your own RobotCommandHandle

### Introduction

A RobotCommandHandle allows you to define routines that should be executed when `rmf_core` issues a command to the robot the RobotCommandHandle is responsible for to:

- `dock()`: Dock
- `follow_new_path()`: Follow a new path
- `stop()` Stop

Python bindings have been written for the `rmf_fleet_adapter::RobotCommandHandle` abstract class that allows you to implement it in Python and have it communicate with the C++ code, as well as other Python bound `rmf_mock_adapter` classes and methods.

That is, **you can write Python code as overrides to C++ RobotCommandHandle methods and have `rmf_core` execute it in the C++ side**! This allows you to effectively write robot routines in Python that are callable from `rmf_core`!

All the standard functionalities for Python classes are open to you. You **may define new attributes or members** as you wish.



### Caveats

- The class methods `follow_new_path`, `dock`, and `stop` methods must be implemented if you want to use them, since the underlying C++ definitions are pure `virtual` method which **requires** an override
  - These methods will be called in from the C++ side by `rmf_core`! `rmf_core` will call the methods with the relevant arguments, so it is **absolutely essential that you ensure that the same number of arguments are exposed in any of the core methods that you define**.
- You **must not** declare an `__init__` method, as that will override the binding
  - If you still need to, look at the **Using `__init__`** section

Also:

- You may pass instances of your implemented RobotCommandHandles into bound C++ methods that take in `std::shared_ptr<rmf_mock_adapter::RobotCommandHandle>` arguments. This is because the bound class inherits from that type! It's very convenient.



### Very Minimal Template

```python
import rmf_adapter as adpt

class RobotCommandHandle(adpt.RobotCommandHandle):
    # If you want to declare class attributes you do it here or in an init
    new_member = "Rawr"
    
    # The argument names do not need to be the same
    # But they are declared here to match the C++ interface for consistency
    def follow_new_path(self,
                        waypoints: str,
                        path_finished_callback: Callable) -> None:
        # Your implementation goes here.
        # You may replace the following lines!
        print(self.new_member)  # We use the instance variable here!
        path_finished_callback()
    
    def dock(self,
             dock_name: str,
             docking_finished_callback: Callable) -> None:
        # Implementation here too!
        print(dock_name)
        docking_finished_callback()
        
# Then you may simply instantiate it like any other Python class!
command_handler = RobotCommandHandle()

## Testing Instance Attributes
print(command_handler.new_member)  # Directly
command_handler.follow_new_path("", lambda: None)  # Via class method

command_handler.newer_member = "Rer"  # A new one!
print(command_handler.newer_member)

## Testing Class Methods
# And the methods can get called from the Python side
command_handler.dock(
    "", lambda: print("Dock callback works!")
) 

# But also to a C++ method that takes in a std::shared_ptr argument!
adpt.test_shared_ptr(command_handler,
                     "wow",
                     lambda: print("wow"))
# With default args!
adpt.test_shared_ptr(command_handler,
                     docking_finish_callback=lambda: print("wow"))
```



## Using `__init__`

If, however, you still want to define an `__init__` magic method, ensure that you **explicitly** call the required bound C++ constructor.

```python
class RobotCommandHandleInit(adpt.RobotCommandHandle):
    def __init__(self, new_member="rawr"):
        adpt.RobotCommandHandle.__init__(self)
        self.new_member = new_member
    
    # The argument names do not need to be the same
    # But they are declared here to match the C++ interface for consistency
    def follow_new_path(self,
                        waypoints: str,
                        path_finished_callback: Callable) -> None:
        # Your implementation goes here.
        # You may replace the following lines!
        print(self.new_member)  # We use the instance variable here!
        path_finished_callback()
    
    def dock(self,
             dock_name: str,
             docking_finished_callback: Callable) -> None:
        # Implementation here too!
        print(dock_name)
        docking_finished_callback()
```



## Creating Your Own Event Executor

An event executor simply executes some routing upon receipt of an event.

In this case, the `graph.lane.Executor` binding class allows you to override the various execution routines for the following event types (defined and used by `rmf_core` on the C++ side):

- `Dock`
- `DoorOpen`
- `DoorClose`
- `LiftDoorOpen`
- `LiftDoorClose`
- `LiftMove`

The corresponding bound event objects in Python can be found defined in `graph.lane`. You can print information about them out using `dir` and `help` as usual.

```python
import rmf_adapter.graph as graph

class EventListener(graph.lane.Executor):
    def __init__(self):
        graph.lane.Executor.__init__(self)

    def dock_execute(self, dock):
        return

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

```



### Using Executors

Simply pass it into the `execute()` method for a given `Entry` object, which you can find attached to either the `entry` or `exit` members of a given graph `Lane` object.

The appropriate event execute method from the executor will be called, with the event being passed in, allowing you to access the relevant event attributes.

```python
executor = self.EventListener()

# Where lane_obj is a lane from graph.get_lane()
lane_obj.entry.event.execute(executor)
```

So, for example, if the event in the lane_obj's entry node was docking event, and our executor had this `dock_execute` defined:

```python
def dock_execute(self, dock):
    self.dock_to_wp[dock.dock_name] = self.wp
    print("DOCK EVENT EXECUTED FOR DOCK:", dock.dock_name)
```

Then it will get called! (Notice how the passed in dock's `dock_name` attribute is accessible.)



## Interacting with `rmf_core`

Check out the `test_adapter.py` script in `scripts` for an integration example with `rmf_core`! These steps are **much easier understood in context**.

However, if you have already done that, relevant points are to:

1. Init `rclpy` if needed (`rclpy.init()`), and init `rclcpp` (`adpt.init_rclcpp()`)
2. Define your own `ChildRobotCommandHandle` subclass of `adpt.RobotCommandHandle`
   - Making sure to define any of the core methods you intend for the `rmf_core`  to be able to call
   - **Ensure that the method argument signatures are the same as the template though**!
   - If you want the ability to update `rmf_core` on the robot's status, also be sure to expose a class member for a bound `RobotUpdateHandle` instance, that can be used to notify `rmf_core` of any **updates to robot position** (`update_position()`), **interruptions** (`interrupted()`), or additional **delays** (`add_delay()`)
3. Optionally define custom `graph.lane.Executor`executor objects to handle events
4. Define your map's graph with `graph.Graph()`
5. Adding **map waypoints**: `add_waypoint()`
   - You can also specify if the waypoints are holding_points (`set_holding_point()`) or passthrough_points (`set_passthrough_point()`)
6. And **map lanes**: `add_lane()`, `add_bidir_lane()`, `add_dock_lane()
7. Add keys to the graph waypoints, which gives the waypoints names that are relevant when requests are sent via the `MockAdapter/Adapter` to `rmf_core`
8. Define your robot
   - By defining its profile: `traits.Profile()`
   - And instantiating its traits: `traits.VehicleTraits()`
9. Create an adapter: `adpt.Adapter()` or `adpt.MockAdapter()`
10. Attach a new fleet: `add_fleet()`
11. Define a callback function for task acceptance: `accept_task_requests()`
12. Set task planner parameters: `set_task_planner_params()`
13. Create a list of Plan Starts: `[plan.Start()]`
    - You will need to pass in a C++ starting time object, which you can obtain using the `now()` method from your instantiated `Adapter/MockAdapter`
14. And add your robots to your fleet: `add_robot()`
15. Then simply request your deliveries!: `request_delivery()`
16. Then spin your `rclpy` nodes if you have any!: `spin()`, `spin_once()`

T​hen you're done! :tada:

![](https://media.giphy.com/media/woDjSSLgqys8yLiJaP/giphy.gif)



## Running tests

Unit tests have been written for the individual components.

You may invoke `pytest` directly in the appropriate directory.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <rmf_fleet_adapter_python_dir>/tests

# Invoke the tests
$ pytest -v
```

Or use `colcon test` with console output redirection.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <workspace_dir>

# Invoke the tests
$ colcon test --packages-select rmf_fleet_adapter_python --event-handlers console_direct+
```



## Gotchas

### Pointer Indirection Gotchas

- `clone_ptr` indirection does not seem to work correctly currently! Be **very careful**!
- The only way surefire way to do pointer indirection is to do it via the objects that manage them. Unfortunately there isn't much of a workaround given that most of the pointers point to abstract classes.
- For most of the other pointers, you must make them using the various factory functions. Do not instantiate them directly since you will not be able to configure their internal members, even if those members are public.
  - No explicit bound methods exist for them as they are meant to be pointers to implementations that might vary widely.



### Update Handles Should NOT be Directly Instantiated (Segfault risk!)

- The `RobotUpdateHandle` and `FleetUpdateHandle` classes must **only be instantiated via their factory methods**! (Their init members have been disabled as a result.)
  - `FleetUpdateHandle` should be instantiated via `TestScenario` (via `add_fleet`)
  - `RobotUpdateHandle` should be instantiated via `FleetUpdateHandle` (via `add_robot`)
  - Doing otherwise will cause their members to contain null values which will **lead to segmentation faults**!!!



### Different Kinds of Waypoints

- The `graph` and `plan` submodules have their own internal `Waypoint` classes with different, but related interfaces!



### Two Kinds of ROS2 Nodes

Seeing how we are binding C++ objects and methods from `rmf_core`, we inevitably will end up binding an `rclcpp` `Node` object. (This will normally come from the `Adapter` or `MockAdapter` node members.)

However, if we wanted to, for instance, use `rclpy` and define our own pub-sub functionalities in our Python script, we'll have an `rclpy` `Node` object that we can play around with as well.

**The two node types are NOT interchangeable**! Keep a VERY strong handle on which nodes are which, and treat them with respect.

The reason why the `rclcpp` Node has been exposed for you to use, is for an interface to access `rmf_core`'s `MockAdapter/Adapter` time, as it pertains to the `rclcpp` ROS2 clock, which is helpfully converted into a Python manipulatable `datetime` object.



### Time Shenanigans

There are **three kinds of time** that you should be concerned with, owing to different types of time used on the C++ side. 

However, confusing things can happen due to how `pybind11` provides conversions between those C++ types and Python `datetime` types. Let me explain...

The three kinds of time that the C++ side uses are:

- Steady clock timepoint: `std::chrono::steady_clock::time_point`
  - This gets converted into a `datetime.timedelta`, where the timedelta represents the time elapsed since the start of the `steady_clock`'s time (that is, since its epoch)
- System clock timepoint: `std::chrono::time_point`
  - This gets converted into a `datetime.datetime`, simple.
- Duration: `std::chrono::Duration`
  - This gets converted into a `datetime.timedelta`, which represents the quantity of time that the `Duration` represents.

Caught the confusion? Three types of C++ time are being converted into **TWO** types of Python time, with no way to distinguish between the `datetime.timedelta` objects that represent `Duration` objects as opposed to `std::chrono::steady_clock::time_point` objects. 

The conversions back and forth work fine. But you **must** be very aware of which `timedelta` objects refer to which.

In an effort to make it simpler, the bindings automatically convert `steady_clock::time_point` objects to system_clock `time_point` objects. So that what is meant by the `datetime.timedelta` objects is clearer.

#### **Caveats**

There is also an additional caveat though. The `steady_clock::time_point` values are **relative** to the ROS2 clock. That is, it uses the ROS2 epoch.

So when we convert them to system_clock `time_point` objects, just note that that relative time be used to construct the `datetime.datetime` object.

So, for example, if the steady_clock `time_point` was relative to a ROS2 clock with epoch start on 1 Jan 2020 00:00:00, and represented 1 second, and it gets converted automatically by the bindings, the returned `datetime.datetime` object will actually represent... 1 Jan 1970 00:00:01! Because system clocks are relative to the UNIX epoch!

All that is important is the relative time values. If you keep that as a constant, and pass the `datetime` objects back into bound methods that use them, there are no additional implications on the correct functioning of the C++ calls that those bound methods will invoke.
