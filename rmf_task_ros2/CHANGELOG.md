## Changelog for package rmf_task_ros2

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
* Change default task auction evaluator to `QuickestFinishEvaluator`: [#211](https://github.com/open-rmf/rmf_ros2/pull/211)
* ws broadcast client in dispatcher node [#212](https://github.com/open-rmf/rmf_ros2/pull/212)
* create unique task_id with timestamp [#223](https://github.com/open-rmf/rmf_ros2/pull/223)

2.0.0 (2022-03-18)
------------------
No changes yet

1.5.0 (2022-02-14)
------------------
* Support flexible task definitions [#168](https://github.com/open-rmf/rmf_ros2/pull/168)

1.3.0 (2021-01-13)
------------------
* Introduce dispatcher node to facilitate dispatching of tasks: [#217](https://github.com/osrf/rmf_core/pull/217)
