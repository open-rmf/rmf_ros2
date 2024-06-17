^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_websocket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.2 (2024-06-18)
------------------

2.7.1 (2024-06-11)
------------------

2.7.0 (2024-06-01)
------------------
* Fix race condition for ingesting/dispensing and disable uncrustify tests by default (`#362 <https://github.com/open-rmf/rmf_ros2/pull/362>`_)
* Fix deadlock in websocket server (`#342 <https://github.com/open-rmf/rmf_ros2/pull/342>`_)
* Lower debug level of some messages in rmf_websocket (`#340 <https://github.com/open-rmf/rmf_ros2/pull/340>`_)
* Refactors the socket broadcast client (`#329 <https://github.com/open-rmf/rmf_ros2/pull/329>`_)
* Contributors: Arjo Chakravarty, Grey, Luca Della Vedova, Yadunund

2.6.0 (2024-03-13)
------------------

2.5.0 (2023-12-22)
------------------

2.4.0 (2023-12-15)
------------------

2.3.2 (2023-08-28)
------------------

2.3.1 (2023-08-10)
------------------

2.3.0 (2023-06-08)
------------------

2.2.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#276 <https://github.com/open-rmf/rmf_ros2/pull/276>`_)
* Contributors: Yadunund

2.1.5 (2023-05-20)
------------------

2.1.4 (2023-04-27)
------------------

2.1.3 (2023-04-26)
------------------

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
* Initial release
