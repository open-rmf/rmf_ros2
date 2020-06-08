# rmf_mock_adapter_python
Python bindings for https://github.com/osrf/rmf_mock_adapter



## Using the Bindings

Ensure that you have built this package and sourced its environment. Then it is as simple as importing the module containing the Python bindings!

```python
import rmf_adapter as adpt

# You may then check the available bindings
print(dir(adpt))
```



## Running the Integration Test

```shell
ros2 run rmf_mock_adapter_python test_adapter
```



## Creating your own RobotCommandHandle

Python bindings have been written for the `rmf_mock_adapter::RobotCommandHandle` abstract class that allows you to implement it in Python and have it communicate with the C++ code, as well as other Python bound `rmf_mock_adapter` classes and methods.

There are some **caveats** though.

- Instance attributes are declared as static attributes, but they are actually instanced
- The `follow_new_path` and `dock` methods must be implemented, since the underlying C++ definition is a pure `virtual` method which requires an override
- You **must not** declare an `__init__` method, as that will override the binding
  - If you still need to, look at the **Using `__init__`** section

Also:

- You may pass instances of your implemented RobotCommandHandles into bound C++ methods that take in `std::shared_ptr<rmf_mock_adapter::RobotCommandHandle>` arguments. This is because the bound class inherits from that type! It's very convenient.

Using this template is a good idea.

```python
import rmf_adapter

class RobotCommandHandle(adpt.RobotCommandHandle):
    # If you want to declare instance attributes you do it here
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

# adpt.test_shared_ptr binds:
# [](std::shared_ptr<rmf_mock_adapter::RobotCommandHandle> handle,
#    std::string dock_name = "DUMMY_DOCK_NAME",
#    std::function<void()> docking_finished_callback = [&](){})
# {
#   handle->dock(dock_name, docking_finished_callback);
# }
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



## Running tests

You may invoke `pytest` directly in the appropriate directory.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <rmf_mock_adapter_python_dir>/tests

# Invoke the tests
$ pytest -v
```

Or use `colcon test` with console output redirection.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <workspace_dir>

# Invoke the tests
$ colcon test --packages-select rmf_mock_adapter_python --event-handlers console_direct+
```



## Gotchas

### Pointer Indirection Gotchas

- `clone_ptr` indirection does not seem to work correctly currently! Be **very careful**!
- The only way surefire way to do pointer indirection is to do it via the objects that manage them. Unfortunately there isn't much of a workaround given that most of the pointers point to abstract classes.
- For most of the other pointers, you must make them using the various factory functions. Do not instantiate them directly since you will not be able to configure their internal members, even if those members are public.
  - No explicit bound methods exist for them as they are meant to be pointers to implementations that might vary widely.

### Missing Implementations

- Unfortunately, since there is no way to instantiate VelocityConstraint pointers yet, you cannot pass them into `Node`s yet.
  - `VelocityConstraint::clone` is not bound. And overriding it in Python doesn't expose the underlying pointer.

### Update Handles Cannot be Directly Instantiated (Segfault risk!)

- The `RobotUpdateHandle` and `FleetUpdateHandle` classes must **only be instantiated via their factory methods**!
  - `FleetUpdateHandle` should be instantiated via `TestScenario` (via `add_fleet`)
  - `RobotUpdateHandle` should be instantiated via `FleetUpdateHandle` (via `add_robot`)
  - Doing otherwise will cause their members to contain null values which will **lead to segmentation faults**!!!

### Different Kinds of Waypoints

- The `graph` and `plan` submodules have their own internal `Waypoint` classes with different, but related interfaces!

### Memory Leak

- The destructor for `TestScenario` will be disabled soon to avoid a segfault on object deallocation. When that happens, be **very wary** of repeatedly creating or re-instantiating multiple `Scenario` objects since their memory will not be cleared as long as the program does not exit.
  - This also means that you should preferably not be continuously and needlessly adding fleets and robots over and over repeatedly, since Scenarios will not get deallocated.


