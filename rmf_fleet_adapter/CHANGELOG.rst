^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_fleet_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
