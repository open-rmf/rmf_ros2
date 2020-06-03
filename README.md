# rmf_mock_adapter_python
Python bindings for https://github.com/osrf/rmf_mock_adapter



## Using the Bindings

Ensure that you have built this package and sourced its environment. Then it is as simple as importing the module containing the Python bindings!

```python
import rmf_adapter as adpt
```



## Creating your own RobotCommandHandle

Python bindings have been written for the `rmf_mock_adapter::RobotCommandHandle` abstract class that allows you to implement it in Python and have it communicate with the C++ code, as well as other Python bound `rmf_mock_adapter` classes and methods.

There are some **caveats** though.

- Instance attributes are declared as static attributes, but they are actually instanced
- The `follow_new_path` and `dock` methods must be implemented, since the underlying C++ definition is a pure `virtual` method which requires an override
- You **must not** declare an `__init__` method, as that will override the binding

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