#!/bin/bash
set -e

on_exit() {
  wait
}

ros2 run rmf_scheduler_ros2 rmf_scheduler_ros2 e2e.sqlite3 &
pid=$!
trap on_exit EXIT

{{TEST_EXE}} "$@"
