^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#276 <https://github.com/open-rmf/rmf_ros2/pull/276>`_)
* Contributors: Yadunund

2.1.5 (2023-05-20)
------------------
* Reformat code to meet expectations of uncrustify-0.72.0: (`#274 <https://github.com/open-rmf/rmf_ros2/pull/274>`_)
* Contributors: Yadunund

2.1.4 (2023-04-27)
------------------

2.1.3 (2023-04-26)
------------------

2.1.2 (2022-10-10)
------------------

2.1.0 (2022-10-03)
------------------
* Make async behaviors more robust: (`#228 <https://github.com/open-rmf/rmf_ros2/pull/228>`_)
* Make schedule failover more robust: (`#232 <https://github.com/open-rmf/rmf_ros2/pull/232>`_)
* Ignore conflicts between any plans that have a dependency: (`#205 <https://github.com/open-rmf/rmf_ros2/pull/205>`_)
* Add support for docking in lanes with entry events: (`#226 <https://github.com/open-rmf/rmf_ros2/pull/226>`_)
* Add message conversion functions for the navigation graph: (`#207 <https://github.com/open-rmf/rmf_ros2/pull/207>`_)
* Changes for humble compatibility: (`#215 <https://github.com/open-rmf/rmf_ros2/pull/215>`_)

2.0.0 (2022-03-18)
------------------
* Update to the traffic dependency system: (`#188 <https://github.com/open-rmf/rmf_ros2/pull/188>`_)

1.5.0 (2022-02-14)
------------------
* Allow participants to sync up with remote databases when discrepancies arise (`#145 <https://github.com/open-rmf/rmf_ros2/pull/145>`_)
* Support for geojson graphs (`#142 <https://github.com/open-rmf/rmf_ros2/pull/142>`_)

1.4.0 (2021-09-01)
------------------
* Make traffic schedule updates more efficient: (`#86 <https://github.com/open-rmf/rmf_ros2/pull/86>`_)
* Add redundancy to the traffic schedule node: (`#61 <https://github.com/open-rmf/rmf_ros2/pull/61>`_)

1.3.0 (2021-06-07)
------------------
* Use topics to update schedule mirrors: (`#17 <https://github.com/open-rmf/rmf_ros2/pull/17>`_)
* Allow participant descriptions to update: (`#17 <https://github.com/open-rmf/rmf_ros2/pull/17>`_)
* Add persistence to Traffic Schedule Participant IDs: (`#242 <https://github.com/osrf/rmf_core/pull/242>`_)

1.2.0 (2021-01-05)
------------------
* Adding distributed blockade system hooks: (`#22 <https://github.com/osrf/rmf_core/pull/22>`_)

1.1.0 (2020-09-24)
------------------
* Add a schedule node factory to the public API: (`#147 <https://github.com/osrf/rmf_core/pull/147>`_)
* Allow the Negotiation class to accept callbacks for Table updates: (`#140 <https://github.com/osrf/rmf_core/pull/140>`_)
* Allow the Negotiation class to provide views for existing Tables: (`#140 <https://github.com/osrf/rmf_core/pull/140>`_)
* Allow the Negotiation class to store up to a certain number of completed negotiations: (`#140 <https://github.com/osrf/rmf_core/pull/140>`_)
* Migrating to ROS2 Foxy: (`#133 <https://github.com/osrf/rmf_core/pull/13>`_)
* Contributors: Aaron Chong, Grey, Yadu, ddengster

1.0.2 (2020-07-27)
------------------
* Always respond to negotiations: (`#138 <https://github.com/osrf/rmf_core/pull/138>`_)

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
