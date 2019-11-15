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

#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic_ros2/geometry/Box.hpp>
#include <rmf_traffic_ros2/geometry/Circle.hpp>

#include "ShapeInternal.hpp"

namespace rmf_traffic_ros2 {
namespace geometry {

//==============================================================================
class ConvexShapeContext::Implementation
    : public internal::ShapeContextImpl<
        rmf_traffic::geometry::FinalConvexShape,
        rmf_traffic_msgs::msg::ConvexShape,
        rmf_traffic_msgs::msg::ConvexShapeContext>
{
public:

  Implementation()
  {
    if(!initialized)
    {
      add<rmf_traffic::geometry::Box>(rmf_traffic_msgs::msg::ConvexShape::BOX);
      add<rmf_traffic::geometry::Circle>(rmf_traffic_msgs::msg::ConvexShape::CIRCLE);
    }
  }

  static const Implementation& get(const ConvexShapeContext& parent)
  {
    return *parent._pimpl;
  }
};

//==============================================================================
ConvexShapeContext::ConvexShapeContext()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShape ConvexShapeContext::insert(
    rmf_traffic::geometry::ConstFinalConvexShapePtr shape)
{
  return _pimpl->insert(std::move(shape));
}

//==============================================================================
rmf_traffic::geometry::ConstFinalConvexShapePtr ConvexShapeContext::at(
    const rmf_traffic_msgs::msg::ConvexShape& shape) const
{
  return _pimpl->at(shape);
}

} // namespace geometry

//==============================================================================
geometry::ConvexShapeContext convert(
    const rmf_traffic_msgs::msg::ConvexShapeContext& from)
{
  using namespace rmf_traffic::geometry;

  geometry::ConvexShapeContext context;
  for(const auto& box : from.boxes)
    context.insert(make_final_convex<Box>(convert(box)));

  for(const auto& circle : from.circles)
    context.insert(make_final_convex<Circle>(convert(circle)));

  return context;
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext convert(
    const geometry::ConvexShapeContext& _from)
{
  using rmf_traffic_msgs::msg::ConvexShape;
  using namespace rmf_traffic::geometry;

  const auto& from = geometry::ConvexShapeContext::Implementation::get(_from);

  rmf_traffic_msgs::msg::ConvexShapeContext context;

  for(const auto& box : from.shapes.at(ConvexShape::BOX))
    context.boxes.emplace_back(convert(static_cast<const Box&>(box->source())));

  for(const auto& circle : from.shapes.at(ConvexShape::CIRCLE))
    context.circles.emplace_back(convert(static_cast<const Circle&>(circle->source())));

  return context;
}

} // namespace rmf_traffic_ros2
