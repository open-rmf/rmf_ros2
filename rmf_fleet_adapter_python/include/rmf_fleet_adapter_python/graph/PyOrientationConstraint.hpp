/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef PYORIENTATIONCONSTRAINT_HPP
#define PYORIENTATIONCONSTRAINT_HPP

#include <Eigen/Dense>
#include <vector>

#include "rmf_fleet_adapter/agv/Adapter.hpp"
#include <rmf_utils/clone_ptr.hpp>

using Graph = rmf_traffic::agv::Graph;

// Trampoline rmf_traffic::agv::Graph::OrientationConstraint wrapper class
// to allow method overrides from Python
class PyOrientationConstraint :
  public Graph::OrientationConstraint
{
public:
  // Constructor
  using Graph::OrientationConstraint::OrientationConstraint;

  bool apply(Eigen::Vector3d& position,
    const Eigen::Vector2d& course_vector) const override
  {
    PYBIND11_OVERLOAD_PURE(
      bool,
      Graph::OrientationConstraint,
      apply,
      position,
      course_vector
    );
  }

  rmf_utils::clone_ptr<OrientationConstraint> clone() const override
  {
    PYBIND11_OVERLOAD_PURE(
      rmf_utils::clone_ptr<OrientationConstraint>,
      Graph::OrientationConstraint,
      clone,  // Trailing comma required
    );
  }
};

#endif // PYORIENTATIONCONSTRAINT_HPP
