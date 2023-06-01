^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.6 (2023-06-02)
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
