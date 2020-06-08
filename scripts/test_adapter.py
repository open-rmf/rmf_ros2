import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter as adpt

import random


class MockRobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self):
        adpt.RobotCommandHandle.__init__(self)
        self.updater = None
        self.current_path_index = None
        self.waypoints = None
        self.path_finished_callback = None

    def follow_new_path(self, waypoints, path_finished_callback):
        self.current_path_index = 0
        self.waypoints = waypoints
        self.path_finished_callback = path_finished_callback

    def dock(self):
        pass

    def step(self):
        if not self.waypoints:
            return

        if self.current_path_index < len(self.waypoints):
            self.current_path_index += 1

        if self.current_path_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_path_index]

            if self.updater:
                if waypoint.graph_index:
                    if waypoint.graph_index.has_value:
                        self.updater.update_position(
                            waypoint.graph_index.value,
                            0.0
                        )
                    else:
                        self.updater.update_position(waypoint.position, [])
                else:
                    self.updater.update_position(waypoint.position, [])

            if self.interruped():
                self.updater.interrupted()

        if self.current_path_index == len(self.waypoints) and \
                self.path_finished_callback:
            self.path_finished_callback()
            self.path_finished_callback = None

    def interruped(self):
        # 40% chance of interruption at each step
        return random.random() < 0.4


def main():
    map_name = "test_map"
    test_graph = graph.Graph()

    # Waypoints 1, 2, 4, 8, 9 are holding positions
    """
                      10
                       |
                       |
                       8------9
                       |      |
                       |      |
         3------4------5------6------7
                       |      |
                       |      |
                       1------2
                       |
                       |
                       0
    """

    test_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    test_graph.add_waypoint(map_name, [0.0, -5.0], True)  # 1
    test_graph.add_waypoint(map_name, [5.0, -5.0], True)  # 2
    test_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    test_graph.add_waypoint(map_name, [-5.0, 0.0], True)  # 4
    test_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    test_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    test_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    test_graph.add_waypoint(map_name, [0.0, 5.0], True)  # 8
    test_graph.add_waypoint(map_name, [5.0, 5.0], True)  # 9
    test_graph.add_waypoint(map_name, [0.0, 10.0])  # 10

    lanes = [[0, 1],
             [1, 2],
             [1, 5],
             [2, 6],
             [3, 4],
             [4, 5],
             [5, 6],
             [6, 7],
             [5, 8],
             [6, 9],
             [8, 9],
             [8, 10]]

    # Normal graph.add_lane() also works!
    for _lane in lanes:
        test_graph.add_bidir_lane(*_lane)

    test_profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    test_traits = traits.VehicleTraits(traits.Limits(0.7, 0.3),
                                       traits.Limits(1.0, 0.45),
                                       test_profile)

    scenario = adpt.TestScenario()
    for i in range(1, 5000 + 1):
        fleet_update_handle = scenario.add_fleet("mock_fleet",
                                                 test_graph,
                                                 test_traits)

        robot_handles = []

        for wp in [3, 7]:
            robot_handles.append(MockRobotCommandHandle())
            robot_handles[-1].updater = fleet_update_handle.add_robot(
                robot_handles[-1],
                "ROBOT_" + str(wp), test_profile, wp, 0.0
            )

        scenario.test(
            [adpt.Condition(robot_handles[0].updater, plan.Goal(10)),
             adpt.Condition(robot_handles[-1].updater, plan.Goal(0))]
        )

        while not scenario.finished:
            for handle in robot_handles:
                handle.step()

        print("\n== COMPLETE %d ==\n" % i)


if __name__ == "__main__":
    main()
