import sys
import yaml
import datetime
import argparse
from collections import OrderedDict
import bisect
from functools import total_ordering
from zoneinfo import ZoneInfo

from icecream import ic

import rclpy
from rclpy.node import Node, Publisher
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)

from rmf_fleet_msgs.msg import ChargingAssignment, ChargingAssignments

@total_ordering
class ScheduleTimePoint:
    hour: int
    minute: int

    def __init__(self, hour: int, minute: int):
        self.hour = hour
        self.minute = minute

    def parse(text: str):
        segments = text.split(":")
        assert len(segments) == 2, (
            f'Time point text [{text}] does not have the correct HH:MM format'
        )
        hh = int(segments[0])
        mm = int(segments[1])
        assert 0 <= hh and hh < 24, (
            f'Time point text [{text}] has an hour value which is outside the '
            f'valid range of 0 -> 23.'
        )
        assert 0 <= mm and mm < 60, (
            f'Time point text [{text}] has a minute value which is outside the '
            f'valid range of 0 -> 59'
        )
        return ScheduleTimePoint(hh, mm)

    def __eq__(self, other):
        return self.hour == other.hour and self.minute == other.minute

    def __lt__(self, other):
        if self.hour < other.hour:
            return True
        elif self.hour > other.hour:
            return False
        return self.minute < other.minute

    def __hash__(self):
        return hash((self.hour, self.minute))


class Assignment:
    fleet: str
    robot: str
    charger: str

    def __init__(self, fleet, robot, charger):
        self.fleet = fleet
        self.robot = robot
        self.charger = charger


def publish_assignments(
    publisher: Publisher,
    assignments: dict[dict[str]],
    parking: list[str]
):
    for fleet, robots in assignments.items():
        msg = ChargingAssignments()
        msg.fleet_name = fleet
        for robot, charger in robots.items():
            assignment = ChargingAssignment()
            assignment.robot_name = robot
            assignment.waypoint_name = charger

            if charger in parking:
                assignment.mode = ChargingAssignment.MODE_WAIT
            else:
                assignment.mode = ChargingAssignment.MODE_CHARGE
            msg.assignments.append(assignment)

        publisher.publish(msg)

def update_assignments(
    last_update_index: int | None,
    next_update_index: int,
    sorted_times: list,
    schedule: dict,
    assignments: dict,
    parking: list[str],
    publisher: Publisher,
    node: Node,
):
    for key in sorted_times[last_update_index:next_update_index]:
        changes: list[Assignment] = schedule[key]
        for change in changes:
            assignments.setdefault(change.fleet, {})[change.robot] = change.charger
            node.get_logger().info(
                f'Sending {change.fleet}/{change.robot} to {change.charger} at '
                f'{key.hour:02d}:{key.minute:02d}'
            )
    publish_assignments(publisher, assignments, parking)


def simulation_time(node: Node) -> ScheduleTimePoint:
    seconds, _ = node.get_clock().now().seconds_nanoseconds()
    minutes: int = int(seconds/60)
    hour: int = int((minutes/60) % 24)
    minute = minutes % 60
    return ScheduleTimePoint(hour, minute)


def real_time(node: Node, timezone: ZoneInfo) -> ScheduleTimePoint:
    nanoseconds = float(node.get_clock().now().nanoseconds)
    seconds = nanoseconds / 1e9
    dt = datetime.datetime.fromtimestamp(seconds, timezone)
    return ScheduleTimePoint(dt.hour, dt.minute)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = Node("rmf_charging_schedule")
    use_sim_time = node.get_parameter('use_sim_time').value

    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='rmf_charging_schedule',
        description='Manage a fixed 24-hour charger schedule rotation',
    )
    parser.add_argument(
        'schedule', type=str,
        help=(
            'A .yaml file representing the schedule. See README for the '
            'expected format.'
        )
    )
    parser.add_argument(
        '-z', '--timezone', type=str, required=False,
        help=(
            'Timezone that the 24-hour rotation will be based on. If not '
            'provided, the system\'s local timezone will be used.'
        )
    )
    parser.add_argument(
        '-t', '--test-time', action='store_true',
        help=(
            'Use this option to test the real time calculation by printing the '
            'current HH:MM based on your settings. This may be used in '
            'conjunction with the --timezone option and sim time. The node '
            'will immediately quit after printing the time, so this will not '
            'publish any assignment messages.'
        )
    )

    args = parser.parse_args(args_without_ros[1:])
    schedule_file = args.schedule

    if args.timezone is not None:
        timezone = ZoneInfo(args.timezone)
    else:
        timezone = None

    if use_sim_time:
        get_time = lambda: simulation_time(node)
    else:
        get_time = lambda: real_time(node, timezone)

    if args.test_time:
        t = get_time()
        print(f'{t.hour:02d}:{t.minute:02d}')
        return

    with open(schedule_file, 'r') as f:
        schedule_yaml = yaml.safe_load(f)

    unsorted_schedule = {}
    parking = []
    for fleet, change in schedule_yaml.items():
        if fleet == 'parking':
            # We treat the parking entry as a special case that simply lists
            # which waypoints are parking spots
            parking = change
            continue

        for time_text, assignments in change.items():
            time = ScheduleTimePoint.parse(time_text)
            entry: list[Assignment] = unsorted_schedule.get(time, list())
            for robot, charger in assignments.items():
                entry.append(Assignment(fleet, robot, charger))
            unsorted_schedule[time] = entry

    schedule = {}
    sorted_times = []
    for time in sorted(unsorted_schedule.keys()):
        sorted_times.append(time)
        schedule[time] = unsorted_schedule[time]

    num_fleets = len(schedule_yaml.keys())
    transient_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2*num_fleets,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(
        ChargingAssignments, 'charging_assignments', transient_qos
    )

    # fleet -> robot -> charger
    assignments = {}
    last_update_index = bisect.bisect_right(sorted_times, get_time())
    update_assignments(
        None, last_update_index,
        sorted_times, schedule, assignments, parking, publisher, node,
    )

    def update():
        nonlocal last_update_index
        nonlocal sorted_times
        nonlocal schedule
        nonlocal assignments
        nonlocal publisher
        nonlocal parking

        next_update_index = bisect.bisect_right(sorted_times, get_time())
        if last_update_index < next_update_index:
            update_assignments(
                last_update_index, next_update_index,
                sorted_times, schedule, assignments, parking, publisher, node,
            )
            last_update_index = next_update_index

        elif next_update_index < last_update_index:
            # The cycle must have restarted, e.g. passing midnight
            update_assignments(
                None, next_update_index,
                sorted_times, schedule, assignments, parking, publisher, node,
            )
            last_update_index = next_update_index

    node.create_timer(10.0, update)

    rclpy.spin(node)

if __name__ == '__main__':
    main(sys.argv)
