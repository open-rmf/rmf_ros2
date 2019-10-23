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

#ifndef RMF_TRAFFIC_ROS2__GEOMETRY__CONVEXSHAPE_HPP
#define RMF_TRAFFIC_ROS2__GEOMETRY__CONVEXSHAPE_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <rmf_traffic_msgs/msg/convex_shape.hpp>
#include <rmf_traffic_msgs/msg/convex_shape_context.hpp>

namespace rmf_traffic_ros2 {
namespace geometry {

//==============================================================================
class ConvexShapeContext
{
public:

  ConvexShapeContext();

  rmf_traffic_msgs::msg::ConvexShape insert(
      rmf_traffic::geometry::ConstFinalConvexShapePtr shape);

  rmf_traffic::geometry::ConstFinalConvexShapePtr at(
      const rmf_traffic_msgs::msg::ConvexShape& shape) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace geometry

//==============================================================================
geometry::ConvexShapeContext convert(
    const rmf_traffic_msgs::msg::ConvexShapeContext& from);

//==============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext convert(
    const geometry::ConvexShapeContext& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__GEOMETRY__CONVEXSHAPE_HPP
