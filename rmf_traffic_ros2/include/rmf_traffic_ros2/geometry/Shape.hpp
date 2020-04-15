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

#ifndef RMF_TRAFFIC_ROS2__GEOMETRY__SHAPE_HPP
#define RMF_TRAFFIC_ROS2__GEOMETRY__SHAPE_HPP

#include <rmf_traffic/geometry/Shape.hpp>

#include <rmf_traffic_msgs/msg/shape.hpp>
#include <rmf_traffic_msgs/msg/shape_context.hpp>

namespace rmf_traffic_ros2 {
namespace geometry {

//==============================================================================
class ShapeContext
{
public:

  ShapeContext();

  rmf_traffic_msgs::msg::Shape insert(
    rmf_traffic::geometry::ConstFinalShapePtr shape);

  rmf_traffic::geometry::ConstFinalShapePtr at(
    const rmf_traffic_msgs::msg::Shape& shape) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace geometry

//==============================================================================
geometry::ShapeContext convert(
  const rmf_traffic_msgs::msg::ShapeContext& from);

//==============================================================================
rmf_traffic_msgs::msg::ShapeContext convert(
  const geometry::ShapeContext& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__GEOMETRY__SHAPE_HPP
