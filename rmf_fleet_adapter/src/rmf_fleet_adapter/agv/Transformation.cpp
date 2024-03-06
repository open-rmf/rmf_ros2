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
#include <rmf_utils/math.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class Transformation::Implementation
{
public:

  double rotation;
  double scale;
  Eigen::Vector2d translation;

  Eigen::Affine2d transform;
  Eigen::Affine2d transform_inv;

  Implementation(
    double rotation_,
    double scale_,
    Eigen::Vector2d translation_)
  : rotation(rotation_),
    scale(scale_),
    translation(translation_)
  {
    update_transform();
  }

  void update_transform()
  {
    transform = Eigen::Affine2d::Identity();
    transform.translate(translation);
    transform.rotate(Eigen::Rotation2D<double>(rotation));
    transform.scale(scale);

    transform_inv = transform.inverse();
  }
};

//==============================================================================
Transformation::Transformation(
  double rotation,
  double scale,
  Eigen::Vector2d translation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      std::move(rotation),
      std::move(scale),
      std::move(translation)))
{
  // Do nothing
}

//==============================================================================
double Transformation::rotation() const
{
  return _pimpl->rotation;
}

//==============================================================================
double Transformation::scale() const
{
  return _pimpl->scale;
}

//==============================================================================
const Eigen::Vector2d& Transformation::translation() const
{
  return _pimpl->translation;
}

//==============================================================================
Eigen::Vector3d Transformation::apply(
  const Eigen::Vector3d& position) const
{
  Eigen::Vector2d p = _pimpl->transform * position.block<2, 1>(0, 0);
  double angle = rmf_utils::wrap_to_pi(position[2] + _pimpl->rotation);
  return Eigen::Vector3d(p[0], p[1], angle);
}

//==============================================================================
Eigen::Vector3d Transformation::apply_inverse(
  const Eigen::Vector3d& position) const
{
  Eigen::Vector2d p = _pimpl->transform_inv * position.block<2, 1>(0, 0);
  double angle = rmf_utils::wrap_to_pi(position[2] - _pimpl->rotation);
  return Eigen::Vector3d(p[0], p[1], angle);
}

} // namespace agv
} // namespace rmf_fleet_adapter
