# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import rmf_adapter.type as Type
import rmf_adapter.nodes as Nodes

from threading import Thread


def submit_task_thread(mod):
    time.sleep(2)
    task_desc = Type.CPPTaskDescriptionMsg()
    task_desc.delivery = adpt.type.CPPDeliveryMsg()
    print("Sumbiting 2 sample delivery tasks")
    id1 = mod.submit_task(task_desc)
    id2 = mod.submit_task(task_desc)
    print(f" active list >>  {mod.get_active_task_ids()}")

    time.sleep(3)
    print("---- Assertion ----")
    state1 = mod.get_task_state(id1)
    state2 = mod.get_task_state(id2)
    state3 = mod.get_task_state("null_id")
    print(f" {id1}: {state1}   \n {id2}: {state2} \n null_id: {state3}")

    assert state1 == Nodes.TaskState.Failed, "Fail due to absence of a bid"
    assert state2 == Nodes.TaskState.Pending, "state should be pending"
    assert state3 == None, "state should be none"
    assert mod.cancel_task(id2), "failed to cancel task"

    # check if task is canceled
    state2 = mod.get_task_state(id2)
    print(f" Canceled:: {id2}: {state2} \n")
    assert state2 == Nodes.TaskState.Canceled, "task should be canceled"
    print("Done Check")


def main():
    print("Starting Simple Dispatcher Node")
    adpt.init_rclcpp()
    dispatcher = Nodes.DispatcherNode.make_node("sample_dispatcher_node")

    th1 = Thread(target=submit_task_thread, args=(dispatcher,))
    th1.start()

    while True:
        adpt.spin_some_rclcpp(dispatcher.node())
        time.sleep(0.2)

    print("Exiting")


if __name__ == "__main__":
    main()
