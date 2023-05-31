/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <rmf_fleet_adapter/agv/Transformation.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Transformation::Implementation
{
public:

  double rotation;
  double scale;
  Eigen::Vector2d translation;

};

//==============================================================================
Transformation::Transformation(
  double rotation,
  double scale,
  Eigen::Vector2d translation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(rotation),
        std::move(scale),
        std::move(translation)
      }))
{
  // Do nothing
}

//==============================================================================
const double Transformation::rotation() const
{
  return _pimpl->rotation;
}

//==============================================================================
const double Transformation::scale() const
{
  return _pimpl->scale;
}

//==============================================================================
const Eigen::Vector2d& Transformation::translation() const
{
  return _pimpl->translation;
}

//==============================================================================
const Eigen::Vector3d transform(
  const Transformation& transformation,
  const Eigen::Vector3d& pose)
{
  const auto& rotated =
    Eigen::Rotation2D<double>(transformation.rotation()) *
    (transformation.scale() * pose.block<2, 1>(0, 0));
  const auto& translated = rotated + transformation.translation();

  return Eigen::Vector3d{
    translated[0], translated[1], pose[2] + transformation.rotation()};
}

} // namespace agv
} // namespace rmf_fleet_adapter
