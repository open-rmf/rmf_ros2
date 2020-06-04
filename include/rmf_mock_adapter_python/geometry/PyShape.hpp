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

#ifndef PYSHAPE_HPP
#define PYSHAPE_HPP

#include <rmf_traffic/geometry/Shape.hpp>

namespace geometry = rmf_traffic::geometry;

// Trampoline rmf_traffic::geometry::Shape wrapper class
// to allow method overrides from Python and type instantiation
class PyShape : public geometry::Shape
{
public:
  // Constructor
  using geometry::Shape::Shape;

  geometry::FinalShape finalize() const override
  {
    PYBIND11_OVERLOAD_PURE(
      geometry::FinalShape,
      geometry::Shape,
      finalize,  // Trailing comma required to specify no args
    );
  }
};

// Py class to expose protected constructors
class PyFinalShape : public geometry::FinalShape
{
public:
  // Constructor
  using geometry::FinalShape::FinalShape;
};

#endif // PYSHAPE_HPP
