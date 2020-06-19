/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic_ros2/geometry/Shape.hpp>
//#include <rmf_traffic_ros2/geometry/Box.hpp>
#include <rmf_traffic_ros2/geometry/Circle.hpp>

#include "ShapeInternal.hpp"

namespace rmf_traffic_ros2 {
namespace geometry {

//==============================================================================
class ShapeContext::Implementation
  : public internal::ShapeContextImpl<
    rmf_traffic::geometry::FinalShape,
    rmf_traffic_msgs::msg::Shape,
    rmf_traffic_msgs::msg::ShapeContext>
{
public:

  Implementation()
  {
    if (!initialized)
    {
      // TODO(MXG): Reconsider this design. Static initialization of singletons
      // seems too error prone with too little benefit. A template-based
      // implementation may be preferable.
      std::lock_guard<std::mutex> lock(initialization_mutex);
      if (!initialized)
      {
//        add<rmf_traffic::geometry::Box>(rmf_traffic_msgs::msg::Shape::BOX);
        add<rmf_traffic::geometry::Circle>(rmf_traffic_msgs::msg::Shape::CIRCLE);
        initialized = true;
      }
    }

    shapes.resize(num_shape_types);
  }

  static const Implementation& get(const ShapeContext& parent)
  {
    return *parent._pimpl;
  }
};

//==============================================================================
ShapeContext::ShapeContext()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
rmf_traffic_msgs::msg::Shape ShapeContext::insert(
  rmf_traffic::geometry::ConstFinalShapePtr shape)
{
  return _pimpl->insert(std::move(shape));
}

//==============================================================================
rmf_traffic::geometry::ConstFinalShapePtr ShapeContext::at(
  const rmf_traffic_msgs::msg::Shape& shape) const
{
  return _pimpl->at(shape);
}

} // namespace geometry

//==============================================================================
geometry::ShapeContext convert(
  const rmf_traffic_msgs::msg::ShapeContext& from)
{
  using namespace rmf_traffic::geometry;

  geometry::ShapeContext context;
//  for (const auto& box : from.convex_shapes.boxes)
//    context.insert(make_final<Box>(convert(box)));

  for (const auto& circle : from.convex_shapes.circles)
    context.insert(make_final<Circle>(convert(circle)));

  // TODO(MXG): Add SimplePolygon here once we're ready to support it

  return context;
}

//==============================================================================
rmf_traffic_msgs::msg::ShapeContext convert(
  const geometry::ShapeContext& _from)
{
  using rmf_traffic_msgs::msg::Shape;
  using namespace rmf_traffic::geometry;

  const auto& from = geometry::ShapeContext::Implementation::get(_from);

  rmf_traffic_msgs::msg::ShapeContext context;

  // TODO(MXG): Consider how this can be refactored to share the same code as
  // convert(ConvexShapeContext)

//  for (const auto& box : from.shapes.at(Shape::BOX))
//  {
//    context.convex_shapes.boxes.emplace_back(
//      convert(static_cast<const Box&>(box->source())));
//  }

  for (const auto& circle : from.shapes.at(Shape::CIRCLE))
  {
    context.convex_shapes.circles.emplace_back(
      convert(static_cast<const Circle&>(circle->source())));
  }

  // TODO(MXG): Add SimplePolygon here once we're ready to support it

  return context;
}

} // namespace rmf_traffic_ros2
