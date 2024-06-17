^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.2 (2024-06-18)
------------------

2.7.1 (2024-06-11)
------------------

2.7.0 (2024-06-01)
------------------
* Fix race condition for ingesting/dispensing and disable uncrustify tests by default (`#362 <https://github.com/open-rmf/rmf_ros2/pull/362>`_)
* Dispatcher only use websockets when dispatch fails (`#355 <https://github.com/open-rmf/rmf_ros2/pull/355>`_)
* Contributors: Aaron Chong, Grey, Luca Della Vedova, Yadunund

2.6.0 (2024-03-13)
------------------
* Add Backward-ROS for improved logging in event of segfaults (`#327 <https://github.com/open-rmf/rmf_ros2/pull/327>`_)
* Explicitly specify all qos depth (`#323 <https://github.com/open-rmf/rmf_ros2/pull/323>`_)
* Adds an option to generate unique hex strings for new task ids, fix time stamp logic (`#319 <https://github.com/open-rmf/rmf_ros2/pull/319>`_)
* Contributors: Aaron Chong, Arjo Chakravarty, Teo Koon Peng

2.5.0 (2023-12-22)
------------------

2.4.0 (2023-12-15)
------------------

2.3.2 (2023-08-28)
------------------
* Improve linking time (`#297 <https://github.com/open-rmf/rmf_ros2/pull/297>`_)
* Contributors: Grey, Luca Della Vedova

2.3.1 (2023-08-10)
------------------
* Adding initiator and request time to booking (`#267 <https://github.com/open-rmf/rmf_ros2/pull/267>`_)
* Contributors: Aaron Chong

2.3.0 (2023-06-08)
------------------

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
* Link Threads to fix build errors on certain platforms: (`#204 <https://github.com/open-rmf/rmf_ros2/pull/204>`_)
* Contributors: decada-robotics, Grey

2.1.2 (2022-10-10)
------------------
* Add find_package for vendored project.
  The vendor package for nlohmann_json_schema_validator was previously
  exporting dependency info for that package, however that is not the
  recommended workflow for vendor packages which are ideally as
  transparent as possible.
* Contributors: Steven! Ragnarök


2.1.0 (2022-10-03)
------------------
* Change default task auction evaluator to `QuickestFinishEvaluator`: (`#211 <https://github.com/open-rmf/rmf_ros2/pull/211>`_)
* ws broadcast client in dispatcher node (`#212 <https://github.com/open-rmf/rmf_ros2/pull/212>`_)
* create unique task_id with timestamp (`#223 <https://github.com/open-rmf/rmf_ros2/pull/223>`_)
* Changes for humble compatibility: (`#215 <https://github.com/open-rmf/rmf_ros2/pull/215>`_)

2.0.0 (2022-03-18)
------------------
No changes yet

1.5.0 (2022-02-14)
------------------
* Support flexible task definitions (`#168 <https://github.com/open-rmf/rmf_ros2/pull/168>`_)

1.3.0 (2021-01-13)
------------------
* Introduce dispatcher node to facilitate dispatching of tasks: (`#217 <https://github.com/osrf/rmf_core/pull/217>`_)
