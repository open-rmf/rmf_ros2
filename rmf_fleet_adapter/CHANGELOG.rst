^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_fleet_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.1 (2025-07-21)
-------------------
* Fix a race condition that can cause an endless loop when waiting for a lift (`#449 <https://github.com/open-rmf/rmf_ros2/issues/449>`_)
* Fix orientation mismatch when !skip_rotation_commands (`#422 <https://github.com/open-rmf/rmf_ros2/issues/422>`_)
* Migrate fleet_adapter to target_link_libraries (`#438 <https://github.com/open-rmf/rmf_ros2/issues/438>`_)
* Contributors: Grey, Luca Della Vedova

2.11.0 (2025-06-09)
-------------------

2.10.1 (2025-05-13)
-------------------
* Add dependency on rclcpp_action (`#431 <https://github.com/open-rmf/rmf_ros2/issues/431>`_)

2.10.0 (2025-05-09)
-------------------
* Add a timer to log the size of planner caches (`#427 <https://github.com/open-rmf/rmf_ros2/issues/427>`_)
* Fix goal orientation constraint (`#426 <https://github.com/open-rmf/rmf_ros2/issues/426>`_)
* Log reason for rejecting dynamic event goals (`#419 <https://github.com/open-rmf/rmf_ros2/issues/419>`_)
* Completely Excise Reservation Cancellation pathways (`#416 <https://github.com/open-rmf/rmf_ros2/issues/416>`_)
* Reduce churn in lift supervisor (`#418 <https://github.com/open-rmf/rmf_ros2/issues/418>`_)
* Dynamic Event Action Server (`#410 <https://github.com/open-rmf/rmf_ros2/issues/410>`_)
* Improve arrival time predictions for lanes with speed limits (`#400 <https://github.com/open-rmf/rmf_ros2/issues/400>`_)
* Ensure correct docking orientation (`#414 <https://github.com/open-rmf/rmf_ros2/issues/414>`_)
* Direct robot task request estimation feature (`#408 <https://github.com/open-rmf/rmf_ros2/issues/408>`_)
* Giving options to separate fire alarm by fleet names (`#413 <https://github.com/open-rmf/rmf_ros2/issues/413>`_)
* Update documentation for RobotCommandHandle (`#409 <https://github.com/open-rmf/rmf_ros2/issues/409>`_)
* Remove bad optional access (`#406 <https://github.com/open-rmf/rmf_ros2/issues/406>`_)
* Fix potential issues around tasks starting (`#403 <https://github.com/open-rmf/rmf_ros2/issues/403>`_)
* Fix the criteria for replanning (`#405 <https://github.com/open-rmf/rmf_ros2/issues/405>`_)
* Do not update assignments if bid notice is for a dry run (`#401 <https://github.com/open-rmf/rmf_ros2/issues/401>`_)
* Use unique name for exported targets and avoid exporting binary targets (`#396 <https://github.com/open-rmf/rmf_ros2/issues/396>`_)
* Do not skip waypoint if orientation is not aligned (`#398 <https://github.com/open-rmf/rmf_ros2/issues/398>`_)
* Contributors: Arjo Chakravarty, Chen Bainian, Cheng-Wei Chen, Grey, Jun, Luca Della Vedova, Xiyu, lkw303, yadunund

2.9.0 (2024-11-27)
------------------
* Add reached API to read-only fleet adapter (`#387 <https://github.com/open-rmf/rmf_ros2/issues/387>`_)
* Fix lift disruption issue (`#393 <https://github.com/open-rmf/rmf_ros2/issues/393>`_)
* Allow automatic action cancellation to be toggled (`#392 <https://github.com/open-rmf/rmf_ros2/issues/392>`_)
* Adds a simple parking spot management system.  (`#325 <https://github.com/open-rmf/rmf_ros2/issues/325>`_)
* Publish fleet and task updates over ROS 2 if websocket is not provided (`#383 <https://github.com/open-rmf/rmf_ros2/issues/383>`_)
* Allow robot-specific finishing request and specify parking spots (`#379 <https://github.com/open-rmf/rmf_ros2/issues/379>`_)
* Issue relocalization command if the robot is on the wrong map (`#316 <https://github.com/open-rmf/rmf_ros2/issues/316>`_)
* set request_time in lift end session request. (`#385 <https://github.com/open-rmf/rmf_ros2/issues/385>`_)
* Update _last_active_task_time if robot is executing task (`#384 <https://github.com/open-rmf/rmf_ros2/issues/384>`_)
* Populate `Booking.Priority`, added schema to validate `Priority` (`#381 <https://github.com/open-rmf/rmf_ros2/issues/381>`_)
* Emergency trigger to use transient local qos (`#341 <https://github.com/open-rmf/rmf_ros2/issues/341>`_)
* Add a timeout before automatically releasing lift (`#369 <https://github.com/open-rmf/rmf_ros2/issues/369>`_)
* Configure strict lanes (`#367 <https://github.com/open-rmf/rmf_ros2/issues/367>`_)
* Only fail on_cancel deserializations when none of the activities are possible (`#378 <https://github.com/open-rmf/rmf_ros2/issues/378>`_)
* Fix a minor typo in fleet adapter error log (`#307 <https://github.com/open-rmf/rmf_ros2/issues/307>`_)
* Provide an API that says the robot's lift destination (`#376 <https://github.com/open-rmf/rmf_ros2/issues/376>`_)
* Populate LiftProperties for Destination in localize callbacks (`#373 <https://github.com/open-rmf/rmf_ros2/issues/373>`_)
* Revert "Event based lift / door logic (`#320 <https://github.com/open-rmf/rmf_ros2/issues/320>`_)" (`#372 <https://github.com/open-rmf/rmf_ros2/issues/372>`_)
* Update with new RobotMode field (`#345 <https://github.com/open-rmf/rmf_ros2/issues/345>`_)
* Quiet cancel API (`#357 <https://github.com/open-rmf/rmf_ros2/issues/357>`_)
* Contributors: Aaron Chong, Arjo Chakravarty, Grey, Luca Della Vedova, Xiyu, cwrx777

2.8.0 (2024-06-12)
------------------

2.7.1 (2024-06-11)
------------------
* Fix charging status (`#347 <https://github.com/open-rmf/rmf_ros2/pull/347>`_)
* Contributors: Grey

2.7.0 (2024-06-01)
------------------
* Fix race condition for ingesting/dispensing and disable uncrustify tests by default (`#362 <https://github.com/open-rmf/rmf_ros2/pull/362>`_)
* Event based lift / door logic (`#320 <https://github.com/open-rmf/rmf_ros2/pull/320>`_)
* Filter DoorOpen insertion by map name (`#353 <https://github.com/open-rmf/rmf_ros2/pull/353>`_)
* Fix schema dictionary used during robot status override (`#349 <https://github.com/open-rmf/rmf_ros2/pull/349>`_)
* Add fleet-level reassign dispatched tasks API (`#348 <https://github.com/open-rmf/rmf_ros2/pull/348>`_)
* Automatically begin or cancel idle behavior when commission changes (`#346 <https://github.com/open-rmf/rmf_ros2/pull/346>`_)
* Disable automatic retreat (`#330 <https://github.com/open-rmf/rmf_ros2/pull/330>`_)
* Manual release of mutex groups (`#339 <https://github.com/open-rmf/rmf_ros2/pull/339>`_)
* Stabilize commissioning feature (`#338 <https://github.com/open-rmf/rmf_ros2/pull/338>`_)
* Release other mutexes if robot started charging (`#334 <https://github.com/open-rmf/rmf_ros2/pull/334>`_)
* Support labels in booking information (`#328 <https://github.com/open-rmf/rmf_ros2/pull/328>`_)
* Fix interaction between emergency pullover and finishing task (`#333 <https://github.com/open-rmf/rmf_ros2/pull/333>`_)
* Contributors: Aaron Chong, Grey, Luca Della Vedova, Xiyu, Yadunund

2.6.0 (2024-03-13)
------------------
* Removes a line of dead code (`#322 <https://github.com/open-rmf/rmf_ros2/pull/322>`_)
* include cstdint header (`#331 <https://github.com/open-rmf/rmf_ros2/pull/331>`_)
* Add Backward-ROS for improved logging in event of segfaults (`#327 <https://github.com/open-rmf/rmf_ros2/pull/327>`_)
* Explicitly specify all qos depth (`#323 <https://github.com/open-rmf/rmf_ros2/pull/323>`_)
* Add support of fleet-level task (`#317 <https://github.com/open-rmf/rmf_ros2/pull/317>`_)
* Fix minor logging error (`#318 <https://github.com/open-rmf/rmf_ros2/pull/318>`_)
* Contributors: Arjo Chakravarty, Teo Koon Peng, Yadunund, cwrx777

2.5.0 (2023-12-22)
------------------
* Fix edge case when starting on a lane (`#312 <https://github.com/open-rmf/rmf_ros2/pull/312>`_)
* Update `GoToPlace` to allow finding nearest spot (`#308 <https://github.com/open-rmf/rmf_ros2/pull/308>`_)
* Contributors: Arjo Chakravarty, Grey

2.4.0 (2023-12-15)
------------------
* Mutex Groups, localization hook, dynamic charging, and new graph elements (`#310 <https://github.com/open-rmf/rmf_ros2/pull/310>`_)

2.3.2 (2023-08-28)
------------------
* Improve linking time (`#297 <https://github.com/open-rmf/rmf_ros2/pull/297>`_)
* EasyFullControl API (`#235 <https://github.com/open-rmf/rmf_ros2/pull/235>`_)
* Contributors: Grey, Luca Della Vedova, Xiyu, Yadunund

2.3.1 (2023-08-10)
------------------
* Remove duplicate task schemas (`#294 <https://github.com/open-rmf/rmf_ros2/pull/294>`_)
* Fix comparator for direct assignment ordering (`#288 <https://github.com/open-rmf/rmf_ros2/pull/288>`_)
* Adding initiator and request time to booking (`#267 <https://github.com/open-rmf/rmf_ros2/pull/267>`_)
* Contributors: Aaron Chong, Omar Hamza, Yadunund

2.3.0 (2023-06-08)
------------------

2.2.0 (2023-06-06)
------------------
* Fix race condition related to the ``finished`` callback of ``perform_action`` events: (`#273 <https://github.com/open-rmf/rmf_ros2/pull/273>`_)
* Switch to rst changelogs (`#276 <https://github.com/open-rmf/rmf_ros2/pull/276>`_)
* Contributors: Grey, Yadunund

2.1.5 (2023-05-20)
------------------
* Reformat code to meet expectations of uncrustify-0.72.0: (`#274 <https://github.com/open-rmf/rmf_ros2/pull/274>`_)
* Contributors: Yadunund

2.1.4 (2023-04-27)
------------------

2.1.3 (2023-04-26)
------------------
* Fix emergency response for waiting robots: (`#253 <https://github.com/open-rmf/rmf_ros2/pull/253>`_)
* Properly cleanup emergency pullover task: (`#258 <https://github.com/open-rmf/rmf_ros2/pull/258>`_)
* Fix priority assignment when parsing tasks: (`#265 <https://github.com/open-rmf/rmf_ros2/pull/265>`_)
* Link Threads to fix build errors on certain platforms: (`#204 <https://github.com/open-rmf/rmf_ros2/pull/204>`_)
* Contributors: decada-robotics, Luca Della Vedova, Grey, Yadunund

2.1.2 (2022-10-10)
------------------

2.1.0 (2022-10-03)
------------------
* Add API to update speed limits for lanes: (`#217 <https://github.com/open-rmf/rmf_ros2/pull/217>`_)
* Make async behaviors more robust: (`#228 <https://github.com/open-rmf/rmf_ros2/pull/228>`_)
* Allow fleet adapters to change schedule participant profiles: (`#229 <https://github.com/open-rmf/rmf_ros2/pull/229>`_)
* Allow robots to be decommissioned from the task dispatch system: (`#233 <https://github.com/open-rmf/rmf_ros2/pull/233>`_)
* Allow manual toggling of stubborn negotiation: (`#196 <https://github.com/open-rmf/rmf_ros2/pull/196>`_)
* Allow users to specify a custom update listener: (`#198 <https://github.com/open-rmf/rmf_ros2/pull/198>`_)
* Introduce `WaitUntil` activity and use it in the `ResponsiveWait`: (`#199 <https://github.com/open-rmf/rmf_ros2/pull/199>`_)
* Better support for patrol behaviors: (`#205 <https://github.com/open-rmf/rmf_ros2/pull/205>`_)
* Allow `ResponsiveWait` to be enabled and disabled: (`#209 <https://github.com/open-rmf/rmf_ros2/pull/209>`_)
* Publish the navigation graph of the fleet adapter: (`#207 <https://github.com/open-rmf/rmf_ros2/pull/207>`_)
* Allow robot status to be overridden by the user: (`#191 <https://github.com/open-rmf/rmf_ros2/pull/191>`_)
* Add API to report status for `perform_action`: (`#190 <https://github.com/open-rmf/rmf_ros2/pull/190>`_)
* Add APIs for cancelling and killing tasks from the `RobotUpdateHandle`: (`#205 <https://github.com/open-rmf/rmf_ros2/pull/205>`_)
* Add a WaitUntil event and use it for ResponsiveWait: (`#199 <https://github.com/open-rmf/rmf_ros2/pull/199>`_)

2.0.0 (2022-03-18)
------------------
* Update to traffic dependency system: (`#188 <https://github.com/open-rmf/rmf_ros2/pull/188>`_)

1.5.0 (2022-02-14)
------------------
* Support flexible task definitions (`#168 <https://github.com/open-rmf/rmf_ros2/pull/168>`_)
* Add lane speed limit to graph parsing function (`#124 <https://github.com/open-rmf/rmf_ros2/pull/124>`_)
* Support for geojson graphs (`#142 <https://github.com/open-rmf/rmf_ros2/pull/142>`_)

1.4.0 (2021-09-01)
------------------
* Add read_only_blockade adapter: (`#110 <https://github.com/open-rmf/rmf_ros2/pull/110>`_)
* Accommodate finishing tasks: (`#108 <https://github.com/open-rmf/rmf_ros2/pull/109>`_)
* Check if lane request's fleet_name is equal to the fleet's fleet_name: (`#95 <https://github.com/open-rmf/rmf_ros2/pull/95>`_)
* Find nearest waypoint among starts: (`#98 <https://github.com/open-rmf/rmf_ros2/pull/98>`_)

1.3.0 (2021-06-07)
------------------
* Add API for opening and closing lanes: (`#15 <https://github.com/open-rmf/rmf_ros2/pull/15>`_)
    * Added `open_lanes` and `close_lanes` CLI tools for issuing requests
* Allow Traffic Light APIs to update the location of a robot while it is idle: (`#270 <https://github.com/osrf/rmf_core/pull/270>`_)
* Allow TrafficLight and EasyTrafficLight API to update battery level: (`#263 <https://github.com/osrf/rmf_core/pull/263>`_)
* Migrating to a task dispatcher framework: (`#21 <https://github.com/osrf/rmf_core/pull/21>`_)
    * The `rmf_fleet_adapter::agv` component interacts with a dispatcher node over topics with `rmf_task` prefix as specified in `rmf_fleet_adapter/StandardNames.hpp`
    * Support for executing tasks at specified timepoints
    * Support for `Loop`, `Delivery`, `Clean` and `ChargeBattery` tasks
* Introduce ResponsiveWait: (`#308 <https://github.com/osrf/rmf_core/pull/308>`_)
    * The new ResponsiveWait task phase can be used to have idle/waiting robots respond to schedule conflicts
    * Idle robots (robots that do not have an assigned task) will automatically enter ResponsiveWait mode


1.2.0 (2021-01-05)
------------------
* Automatically publish fleet states from the fleet adapter API: (`#232 <https://github.com/osrf/rmf_core/pull/232>`_)
* Easy Traffic Light API: (`#226 <https://github.com/osrf/rmf_core/pull/226>`_)
* Gridlock-proof Traffic Light Implementation: (`#226 <https://github.com/osrf/rmf_core/pull/226>`_)

1.1.0 (2020-09-24)
------------------
* Traffic Light API: (`#147 <https://github.com/osrf/rmf_core/pull/147>`_) (`#176 <https://github.com/osrf/rmf_core/pull/176>`_) (`#180 <https://github.com/osrf/rmf_core/pull/180>`_)
* Allow fleet adapters to adjust the maximum delay: (`#148 <https://github.com/osrf/rmf_core/pull/148>`_)
* Full Control Fleet Adapters respond to emergency alarm topic: (`#162 <https://github.com/osrf/rmf_core/pull/162>`_)
* Migrating to ROS2 Foxy: (`#133 <https://github.com/osrf/rmf_core/pull/133>`_)
* Contributors: Chen Bainian, Grey, Kevin_Skywalker, Marco A. Gutiérrez, Rushyendra Maganty, Yadu

1.0.2 (2020-07-27)
------------------
* Always respond to negotiations: (`#138 <https://github.com/osrf/rmf_core/pull/138>`_)

1.0.1 (2020-07-20)
------------------
* Interrupt dangling negotiation planning efforts to reduce memory usage: (`#130 <https://github.com/osrf/rmf_core/pull/130>`_)
* Trim the amount of system memory that is committed to a fleet adapter after each task: (`#130 <https://github.com/osrf/rmf_core/pull/130>`_)

1.0.0 (2020-06-23)
------------------
* Provides `rmf_fleet_adapter` library
    * The `rmf_fleet_adapter::agv` component can be used to develop a custom "Full Control" fleet adapter
    * `rmf_fleet_adapter/StandardNames.hpp` specifies topic names that are used for RMF integration
* Provides a prototype `read_only` fleet adapter implementation
    * This will be deprecated in the future in favor of a C++ API
    * To use this fleet adapter, you must implement a "read-only fleet driver" to talk to the fleet adapter using `rmf_fleet_msgs`
* Provides a deprecated `full_control` fleet adapter implementation
    * This is made to be backwards compatible with "full-control fleet drivers" that were developed in the early stages of RMF
    * New users should prefer to implement their own fleet adapter using the `rmf_fleet_adapter::agv` API
* Uses rxcpp to make the fleet adapters reactive and multi-threaded
* Has a known memory leak issue which will be resolved in a later release
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Marco A. Gutiérrez, Grey, Yadu, Yadunund, koonpeng, methylDragon
