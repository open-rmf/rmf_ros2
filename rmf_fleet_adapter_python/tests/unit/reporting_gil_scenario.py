"""Integration scenario for the real Open-RMF reporting mutex."""

import sys

import threading
import time

import rmf_adapter as adpt
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.vehicletraits as traits
import rmf_reporting_gil_test as bridge


class MockRobotCommand(adpt.RobotCommandHandle):
    def __init__(self):
        super().__init__()

    def follow_new_path(self, waypoints, path_finished_callback):
        pass

    def dock(self, dock_name, docking_finished_callback):
        pass


def main(method_name):
    adpt.init_rclcpp()

    navigation_graph = graph.Graph()
    navigation_graph.add_waypoint("test_map", [0.0, 0.0])
    navigation_graph.add_waypoint(
    "test_map", [1.0, 0.0]
    ).set_charger(True)
    navigation_graph.add_bidir_lane(0, 1)

    profile = traits.Profile(
        geometry.make_final_convex_circle(0.5)
    )

    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(0.7, 0.3),
        angular=traits.Limits(1.0, 0.45),
        profile=profile,
    )

    adapter = adpt.MockAdapter("ReportingGILTestAdapter")
    fleet = adapter.add_fleet(
        "reporting_test_fleet",
        vehicle_traits,
        navigation_graph,
    )

    robot_command = MockRobotCommand()
    robot_ready = threading.Event()
    robot_handles = []

    def on_robot_added(handle):
        robot_handles.append(handle)
        robot_ready.set()

    starts = [plan.Start(adapter.now(), 0, 0.0)]

    fleet.add_robot(
        robot_command,
        "test_robot",
        profile,
        starts,
        on_robot_added,
    )

    adapter.start()

    assert robot_ready.wait(timeout=10), (
        "RobotUpdateHandle callback did not complete"
    )

    handle = robot_handles[0]
    fixture = bridge.ReportingMutexFixture(handle)

    fixture.wait_until_locked()

    reporting_started = threading.Event()
    release_finished = threading.Event()
    reporting_finished = threading.Event()
    errors = []

    def reporting_call():
        reporting_started.set()
        try:
            if method_name == "create_issue":
                ticket = handle.create_issue(
                    adpt.robot_update_handle.Tier.Info,
                    "gil_regression_test",
                    {"message": "Testing reporting mutex contention"},
                )
                ticket.resolve({"message": "Regression test completed"})
            else:
                getattr(handle, method_name)(
                    "Reporting GIL regression test"
                )
        except Exception as exc:
            errors.append(exc)
        finally:
            reporting_finished.set()

    def release_mutex():
        assert reporting_started.wait(timeout=5)
        time.sleep(0.2)
        fixture.release()
        release_finished.set()

    reporter = threading.Thread(target=reporting_call)
    releaser = threading.Thread(target=release_mutex)

    try:
        releaser.start()
        reporter.start()

        assert reporting_finished.wait(timeout=5), (
            "Reporting call did not finish"
        )

        reporter.join(timeout=5)
        releaser.join(timeout=5)

        assert release_finished.is_set(), (
            "Python releaser could not execute"
        )

        assert not errors, f"Reporting call raised: {errors}"

        print(f"PASS: {method_name} completed under reporting mutex contention")

    finally:
        fixture.release()
        adapter.stop()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: reporting_gil_scenario.py METHOD")
    main(sys.argv[1])
