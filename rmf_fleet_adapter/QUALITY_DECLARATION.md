This document is a declaration of software quality for the `rmf_fleet_adapter` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_fleet_adapter` Quality Declaration

The package `rmf_fleet_adapter` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_fleet_adapter` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_fleet_adapter` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package.
Headers in any other folders are not installed and are considered private.

All launch files in the installed `launch` directory are considered part of the public API.

### API Stability Policy [1.iv]

`rmf_fleet_adapter` will not break public API within a major version number.

### ABI Stability Policy [1.v]

`rmf_fleet_adapter` will not break public ABI within a major version number.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`rmf_fleet_adapter` will not break public API or ABI within a released ROS distribution, i.e. no major releases into the same ROS distribution once that ROS distribution is released.

## Change Control Process [2]

`rmf_fleet_adapter` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_fleet_adapter` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_fleet_adapter` uses DCO as its confirmation of contributor origin policy.
More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.

The most recent CI results can be seen on [the workflow page](https://github.com/open-rmf/rmf_ros2/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_fleet_adapter` does not provide documentation.

### Public API Documentation [3.ii]

`rmf_fleet_adapter` documents its public API.
The documentation is not hosted.

### License [3.iii]

The license for `rmf_fleet_adapter` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmf_fleet_adapter`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

Each feature in `rmf_fleet_adapter` has corresponding tests which simulate typical usage.
They are located in the [`test`](https://github.com/open-rmf/rmf_ros2/tree/master/rmf_fleet_adapter/test) directory.
New features are required to have tests before being added.

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests are not run automatically.
They are located in the [`test`](https://github.com/open-rmf/rmf_ros2/tree/master/rmf_fleet_adapter/test) directory.

### Coverage [4.iii]

`rmf_fleet_adapter` does not track coverage statistics.

### Performance [4.iv]

`rmf_fleet_adapter` does not test performance.

### Linters and Static Analysis [4.v]

`rmf_fleet_adapter` does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

`rmf_fleet_adapter` uses a custom `uncrustify` configuration matching its coding style.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

Below are the required direct runtime ROS dependencies of `rmf_fleet_adapter` and their evaluations.

#### rmf_utils

`rmf_utils` is [**Quality Level 4**](https://github.com/open-rmf/rmf_utils/blob/main/rmf_utils/QUALITY_DECLARATION.md).

#### rmf_door_msgs

`rmf_door_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/QUALITY_DECLARATION.md).

#### rmf_ingestor_msgs

`rmf_ingestor_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_ingestor_msgs/QUALITY_DECLARATION.md).

#### rmf_dispenser_msgs

`rmf_dispenser_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_dispenser_msgs/QUALITY_DECLARATION.md).

#### rmf_fleet_msgs

`rmf_fleet_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_fleet_msgs/QUALITY_DECLARATION.md).

#### rmf_lift_msgs

`rmf_lift_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/QUALITY_DECLARATION.md).

#### rmf_task_msgs

`rmf_task_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_task_msgs/QUALITY_DECLARATION.md).

#### rmf_traffic

`rmf_traffic` is [**Quality Level 4**](https://github.com/open-rmf/rmf_traffic/blob/main/rmf_traffic/QUALITY_DECLARATION.md).

#### rmf_traffic_ros2

`rmf_traffic_ros2` is [**Quality Level 4**](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_traffic_ros2/QUALITY_DECLARATION.md).


### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_fleet_adapter` has no optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`rmf_fleet_adapter` uses the [`yaml-cpp` library](https://github.com/jbeder/yaml-cpp).
This is assumed to be **Quality Level 2** due to its wide use, provided documentation, use of testing, and version number above 1.0.0.

## Platform Support [6]

### Target platforms [6.i]

`rmf_fleet_adapter` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`rmf_fleet_adapter` supports ROS Foxy.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
