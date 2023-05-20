## Changelog for package rmf_fleet_adapter_python

2.1.5 (2023-05-20)
------------------

2.1.4 (2023-04-27)
------------------

2.1.3 (2023-04-26)
------------------

2.1.2 (2022-10-10)
------------------

2.1.0 (2022-10-03)
------------------
* Make async behaviors more robust: [#228](https://github.com/open-rmf/rmf_ros2/pull/228)
* Allow fleet adapters to change schedule participant profiles: [#229](https://github.com/open-rmf/rmf_ros2/pull/229)
* Allow robots to be decommissioned from the task dispatch system: [#233](https://github.com/open-rmf/rmf_ros2/pull/233)
* Allow manual toggling of stubborn negotiation: [#196](https://github.com/open-rmf/rmf_ros2/pull/196)
* Allow users to specify a custom update listener: [#198](https://github.com/open-rmf/rmf_ros2/pull/198)
* Fix various segfaults related to pybind: [#205](https://github.com/open-rmf/rmf_ros2/pull/205)
* Allow `ResponsiveWait` to be enabled and disabled: [#209](https://github.com/open-rmf/rmf_ros2/pull/209)
* Allow robot status to be overridden by the user: [#191](https://github.com/open-rmf/rmf_ros2/pull/191)
* Add API to report status for `perform_action`: [#190](https://github.com/open-rmf/rmf_ros2/pull/190)
* Changes for humble compatibility: [#215](https://github.com/open-rmf/rmf_ros2/issues/215)

2.0.0 (2022-03-18)
------------------
No changes yet

1.5.0 (2022-02-14)
------------------
* Support flexible task definitions [#168](https://github.com/open-rmf/rmf_ros2/pull/168)
* Add lane speed limit to graph parsing function [#124](https://github.com/open-rmf/rmf_ros2/pull/124)

1.3.0 (2021-06-07)
------------------
* Modifications to support refactor of rmf_task (`#51 <https://github.com/open-rmf/rmf_ros2/issues/51>`_)
* Fix symlink-install compilation (`#32 <https://github.com/open-rmf/rmf_ros2/issues/32>`_)
* Updated package.xml (`#26 <https://github.com/open-rmf/rmf_ros2/issues/26>`_)
* Fix/rmf task ros2 cleanup (`#21 <https://github.com/open-rmf/rmf_ros2/issues/21>`_)
* Feature/python binding planner (`#11 <https://github.com/open-rmf/rmf_ros2/issues/11>`_)
* Adding reference_internal tag to function bindings that return raw pointers (`#6 <https://github.com/open-rmf/rmf_ros2/issues/6>`_)
* Feature/add unstable participant api (`#11 <https://github.com/open-rmf/rmf_ros2/issues/11>`_)
* Feature/add simple docs (`#9 <https://github.com/open-rmf/rmf_ros2/issues/9>`_)
* Support apis for task dispatcher (`#10 <https://github.com/open-rmf/rmf_ros2/issues/10>`_)
* differentiate functions to prevent overloading (`#8 <https://github.com/open-rmf/rmf_ros2/issues/8>`_)
* support ez traffic light (`#7 <https://github.com/open-rmf/rmf_ros2/issues/7>`_)
* Update/release 1.1 (`#6 <https://github.com/open-rmf/rmf_ros2/issues/6>`_)
* Implement binding for Duration optional
* Make integration test even stricter
* Add reference capture for posterity
* Add clarifying printouts and fix multi-timer bug
* Integrate compute_plan_starts into integration test
* Implement type tests
* Bind optional constructors and delivery msg interfaces
* Bind compute_plan_starts
* Add update_position overload
* Implement Python Bindings for rmf_fleet_adapter (`#1 <https://github.com/open-rmf/rmf_ros2/issues/1>`_)
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Grey, Marco A. Gutiérrez, Yadu, methylDragon, youliang
