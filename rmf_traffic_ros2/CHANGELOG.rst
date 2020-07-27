^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
