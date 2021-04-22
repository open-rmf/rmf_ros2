import rmf_adapter as adpt
from typing import Callable


# Class under test
class TestHandle(adpt.RobotCommandHandle):
    __test__ = False

    def __init__(self):
        adpt.RobotCommandHandle.__init__(self)
        self.test_member = "rawr"

    def follow_new_path(self,
                        waypoints: str,
                        path_finished_callback: Callable) -> None:
        print(self.test_member)
        path_finished_callback()

    def dock(self,
             dock_name: str,
             docking_finished_callback: Callable) -> None:
        print(dock_name)
        docking_finished_callback()


def test_dynamic_attr():
    test_obj = TestHandle()
    assert test_obj.test_member == "rawr", (
        "Dynamic assignment on init failing!")

    test_obj.newer_member = "rer"
    assert test_obj.newer_member == "rer", "Dynamic assignment failing!"

    test_obj.test_func = lambda x: x
    assert test_obj.test_func(5) == 5, "Dynamic function assignment failing!"

    test_obj_new = TestHandle()
    assert not hasattr(test_obj_new, "test_func"), (
        "Changing instance attributes changes class attributes!")


def test_python_side_methods(capsys):
    test_obj = TestHandle()

    test_obj.follow_new_path("", lambda: print("follow callback works!"))
    captured = capsys.readouterr()
    assert captured.out == "rawr\nfollow callback works!\n", (
        "Method call failed")

    test_obj.dock("dock_rawr", lambda: print("dock callback works!"))
    captured = capsys.readouterr()
    assert captured.out == "dock_rawr\ndock callback works!\n", (
        "Method argument pass failed")


def test_cpp_side_methods(capsys):
    """
    adpt.test_shared_ptr binds:

    [](std::shared_ptr<rmf_adapter::RobotCommandHandle> handle,
        std::string dock_name = "DUMMY_DOCK_NAME",
        std::function<void()> docking_finished_callback = [&](){})
    {
        handle->dock(dock_name, docking_finished_callback);
    }

    Notice that it takes in a shared_ptr of the RobotCommandHandle,
    and uses that pointer to call a pointed instance's dock method.
    """

    test_obj = TestHandle()

    adpt.test_shared_ptr(test_obj,
                         "dock_rawr",
                         lambda: print("dock callback works!"))
    captured = capsys.readouterr()
    assert captured.out == "dock_rawr\ndock callback works!\n", (
        "cpp argument pass failed")

    test_obj.dock = lambda s, f: print("overridden")
    adpt.test_shared_ptr(test_obj,
                         "",
                         lambda: print("not overridden"))
    captured = capsys.readouterr()
    assert captured.out == "overridden\n", "Python method override failed"
