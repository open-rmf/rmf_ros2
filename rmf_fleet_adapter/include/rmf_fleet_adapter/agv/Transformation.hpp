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

#ifndef RMF_FLEET_ADAPTER__AGV__TRANSFORMATION_HPP
#define RMF_FLEET_ADAPTER__AGV__TRANSFORMATION_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// A Transformation object that stores the transformation data needed to
/// perform transformation between robot and RMF cartesian frames.
class Transformation
{
public:

  /// Constructor
  ///
  /// \param[in] rotation
  ///   The rotation angle (radians) between the two cartesian frames.
  ///
  /// \param[in] scale
  ///   The scaling factor between the cartesian frames.
  ///
  /// \param[in] translation
  ///   The 2D translation from one coordinate to another.
  Transformation(
    double rotation,
    double scale,
    Eigen::Vector2d translation);

  /// Get the rotation of this Transformation
  double rotation() const;

  /// Get the scale of this Transformation
  double scale() const;

  /// Get the translation of this Transformation
  const Eigen::Vector2d& translation() const;

  /// Apply this transformation to an (x, y, yaw) position
  Eigen::Vector3d apply(const Eigen::Vector3d& position) const;

  /// Apply the inverse of this transformation to an (x, y, yaw) position
  Eigen::Vector3d apply_inverse(const Eigen::Vector3d& position) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__TRANSFORMATION_HPP
