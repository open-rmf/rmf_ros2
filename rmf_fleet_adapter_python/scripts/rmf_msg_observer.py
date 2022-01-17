#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import asyncio
import websockets
import json
from typing import Callable, Optional, List, Dict, Tuple

# Test script
import threading
import time

#####################################################################


class RmfMsgType:
    FleetState = "fleet_state_update"
    TaskState = "task_state_update"
    TaskLog = "task_log_update"
    FleetLog = "fleet_log_update"


"""
This helper function filters the messy msg from the websocket
and only return the useful data according to the provided filter args
:param json_str:     input json string data
:param msg_type:     type of msg which is to be filter out
:param data_filter:  detailed filter different levels of the data obj
:return:             return the filtered data in dictionary format
"""


def filter_rmf_msg(
        json_str: str,
        filters: Dict[RmfMsgType, List[str]] = {}
) -> Optional[Tuple[RmfMsgType, Dict]]:
    # msg_type: Optional[RmfMsgType] = None,
    # data_filter: List[str] = []) -> dict:
    obj = json.loads(json_str)
    if "type" not in obj:
        print("ERRORRRR: type is not avail as json key")
        return None

    if obj["type"] not in filters:
        return None

    msg_type = obj["type"]
    data = obj["data"]
    if not filters[msg_type]:  # empty list
        return msg_type, data

    for filter in filters[msg_type]:
        if filter not in data:
            print(f" Key ERROR!!, indicated data_filter: "
                  "[{filter}] is not avail in {data}")
            return None

        data = data[filter]
    return msg_type, data


#####################################################################
class AsyncRmfMsgObserver:
    """
    This helper class filters the messy msg from RMF server
    and only return the useful data according to the provided filter args
    Note that this is a blocking class since spin is done internally.
    :param callback_fn:  function callback when a filtered msg is received
    :param server_url:   websocket server address
    :param server_port:  websocket server port number
    :param json_str:     input json string data
    :param msg_type:     type of msg which is to be filter out
    :param data_filter:  detailed filter different levels of the data obj
    """

    def __init__(self,
                 callback_fn: Callable[[dict], None],
                 server_url: str = "localhost",
                 server_port: str = "7878",
                 msg_filters: Dict[RmfMsgType, List[str]] = {}
                 ):
        print("Starting Websocket Server")
        self.callback_fn = callback_fn
        self.server_url = server_url
        self.server_port = server_port
        self.msg_filters = msg_filters

    """
    This is a blocking function, which will spin the RmfMsgObserver.
    By default this server will spin forever. User can provide an
    optional future arg to indicate when to end this observer
    """

    def spin(self, future=asyncio.Future()):
        self.future = future
        asyncio.run(self.__internal_spin())

    async def __msg_handler(self, websocket, path):  # PATH?
        try:
            async for message in websocket:
                ret_data = filter_rmf_msg(
                    message, self.msg_filters)
                if ret_data:
                    # call the provided callback function
                    msg_type, data = ret_data
                    self.callback_fn(msg_type, data)
        except:
            print('Waiting for reconnection')

    async def __check_future(self):
        while not self.future.done():
            # print("is done? ", self.future.done())
            await asyncio.sleep(1)  # arbitrary loop freq check
        print("Received exit signal")
        # TODO: debug the reason why future is not awaitable outside
        # of the loop problem. Thread safe?
        # return await asyncio.wrap_future(self.future)

    async def __internal_spin(self):
        async with websockets.serve(
                self.__msg_handler, self.server_url, self.server_port):
            await self.__check_future()


#####################################################################


# Provide future to indicate when to stop the thread,
# :param: task_id, the target task id to check on completion state
# future will also be set to done internally when a task is completed.
def task_state_observer_thread(fut: asyncio.Future, target_id):
    print("starting observer thread")
    start_time = time.time()

    # sample function for user to provide
    def checkstate_callback(msg_type, data):
        print(f" \nReceived [{msg_type}] :: \n   "
              f"{data['booking']['id']}: {data['status']}")
        if (data['status'] == 'completed' and
                data['booking']['id'] == target_id):
            print("COMPLETED, set future as DONE! from observer")
            fut.set_result(True)

    print("creating sync rmf_observer class")
    observer = AsyncRmfMsgObserver(
        checkstate_callback,
        msg_filters={
            RmfMsgType.TaskState: []}
    )
    print("Done")
    observer.spin(fut)


def main():
    # TODO: Create a script with --argsparser for usage CLI
    print("spawn observer thread")
    fut = asyncio.Future()
    th = threading.Thread(target=task_state_observer_thread, args=(fut,))
    th.start()
    print("start the thread")
    th.join()
    print("Thread join")
    print("exit everything")


if __name__ == "__main__":
    main()
