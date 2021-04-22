#!/usr/bin/env python3

# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import rmf_adapter as adpt
import rmf_adapter.nodes as nodes


def main():
    adpt.init_rclcpp()

    print("Creating Sceduler and Blockade Node")
    blockade_node = nodes.make_blockade(adpt.NodeOptions())
    schedule_node = nodes.make_schedule(adpt.NodeOptions())

    print("spinning rclcpp")
    while True:
        try:
            adpt.spin_some_rclcpp(blockade_node)
            adpt.spin_some_rclcpp(schedule_node)
            time.sleep(0.1)
        except RuntimeError as e:
            # TODO: needa fix runtime error during deregistration
            print('Error on spining: ', e)
    print("Exiting")


if __name__ == "__main__":
    main()
