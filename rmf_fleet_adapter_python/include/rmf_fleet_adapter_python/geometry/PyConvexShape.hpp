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

#ifndef PYCONVEXSHAPE_HPP
#define PYCONVEXSHAPE_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

namespace geometry = rmf_traffic::geometry;

// Trampoline rmf_traffic::geometry::ConvexShape wrapper class
// to allow method overrides from Python and type instantiation
class PyConvexShape : public geometry::ConvexShape
{
public:
  // Constructor
  using geometry::ConvexShape::ConvexShape;

  geometry::FinalConvexShape finalize_convex() const override
  {
    PYBIND11_OVERLOAD_PURE(
      geometry::FinalConvexShape,
      geometry::ConvexShape,
      finalize_convex,  // Trailing comma required to specify no args
    );
  }
};

// Py class to expose protected constructors
class PyFinalConvexShape : public geometry::FinalConvexShape
{
public:
  // Constructor
  using geometry::FinalConvexShape::FinalConvexShape;
};

#endif // PYCONVEXSHAPE_HPP
