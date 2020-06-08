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

#ifndef PYVELOCITYCONSTRAINT_HPP
#define PYVELOCITYCONSTRAINT_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include <rmf_mock_adapter/adapter.hpp>

using Graph = rmf_traffic::agv::Graph;

// Trampoline rmf_traffic::agv::Graph::VelocityConstraint wrapper class
// to allow method overrides from Python
class PyVelocityConstraint :
  public Graph::VelocityConstraint
{
public:
  // Constructor
  using Graph::VelocityConstraint::VelocityConstraint;

  bool apply(Eigen::Vector3d& position,
             const Eigen::Vector2d& course_vector) const override
  {
    PYBIND11_OVERLOAD_PURE(
      bool,
      Graph::VelocityConstraint,
      apply,
      position,
      course_vector
    );
  }

  // NOTE(CH3): TODO(CH3): Nothing satisfactory works here...
  // This obviously does not clone...
  std::unique_ptr<Graph::VelocityConstraint> clone() const override
  {
    std::unique_ptr<Graph::VelocityConstraint> out = \
        std::make_unique<PyVelocityConstraint>();
    return out;
  }
};

#endif // PYVELOCITYCONSTRAINT_HPP
