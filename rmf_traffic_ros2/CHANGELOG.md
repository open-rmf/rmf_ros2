## Changelog for package rmf_traffic_ros2

2.0.0 (2022-03-18)
------------------
* Update to the traffic dependency system: [#188](https://github.com/open-rmf/rmf_ros2/pull/188)

1.5.0 (2022-02-14)
------------------
* Allow participants to sync up with remote databases when discrepancies arise [#145](https://github.com/open-rmf/rmf_ros2/pull/142)
* Support for geojson graphs [#142](https://github.com/open-rmf/rmf_ros2/pull/142)

1.4.0 (2021-09-01)
------------------
* Make traffic schedule updates more efficient: [#86](https://github.com/open-rmf/rmf_ros2/pull/86)
* Add redundancy to the traffic schedule node: [#61](https://github.com/open-rmf/rmf_ros2/pull/61)

1.3.0 (2021-06-07)
------------------
* Use topics to update schedule mirrors: [#17](https://github.com/open-rmf/rmf_ros2/pull/17)
* Allow participant descriptions to update: [#17](https://github.com/open-rmf/rmf_ros2/pull/17)
* Add persistence to Traffic Schedule Participant IDs: [#242](https://github.com/osrf/rmf_core/pull/242)

1.2.0 (2021-01-05)
------------------
* Adding distributed blockade system hooks: [#226](https://github.com/osrf/rmf_core/pull/226)

1.1.0 (2020-09-24)
------------------
* Add a schedule node factory to the public API: [#147](https://github.com/osrf/rmf_core/pull/147)
* Allow the Negotiation class to accept callbacks for Table updates: [#140](https://github.com/osrf/rmf_core/pull/140/)
* Allow the Negotiation class to provide views for existing Tables: [#140](https://github.com/osrf/rmf_core/pull/140/)
* Allow the Negotiation class to store up to a certain number of completed negotiations: [#140](https://github.com/osrf/rmf_core/pull/140/)
* Migrating to ROS2 Foxy: [#133](https://github.com/osrf/rmf_core/pull/133)
* Contributors: Aaron Chong, Grey, Yadu, ddengster

1.0.2 (2020-07-27)
------------------
* Always respond to negotiations: [#138](https://github.com/osrf/rmf_core/pull/138)

1.0.0 (2020-06-23)
------------------
* Provides `rmf_traffic_ros2` library which offers utilities to wrap `rmf_traffic` into `ros2` APIs
    * `rmf_traffic_ros2::convert(T)` functions convert between `rmf_traffic` API data structures and `rmf_traffic_msgs` message structures
    * `rmf_traffic_ros2::schedule` utilities help to connect `rmf_traffic` objects across distributed ROS2 systems
        * `MirrorManager` - Object that maintains a `rmf_traffic::schedule::Mirror` across ROS2 connections
        * `Writer` - Factory for `rmf_traffic::schedule::Participant` objects that can talk to a database across ROS2 connections
        * `Negotiation` - Object that manages a set of traffic negotiations across ROS2 connections
* `rmf_traffic_schedule` - a ROS2 node that manages a traffic schedule service and judges the outcomes of traffic negotiations
* Contributors: Aaron Chong, Grey, Marco A. Gutiérrez, Morgan Quigley, Yadu, Yadunund, koonpeng
