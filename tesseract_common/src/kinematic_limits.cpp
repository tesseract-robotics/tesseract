/**
 * @file kinematic_limits.h
 * @brief Common Tesseract Kinematic Limits and Related Utility Functions
 *
 * @author Levi Armstrong
 * @date March 25, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/kinematic_limits.h>

namespace tesseract_common
{
bool KinematicLimits::operator==(const KinematicLimits& other) const
{
  bool ret_val = true;
  ret_val &= (joint_limits.isApprox(other.joint_limits, 1e-5));
  ret_val &= (velocity_limits.isApprox(other.velocity_limits, 1e-5));
  ret_val &= (acceleration_limits.isApprox(other.acceleration_limits, 1e-5));
  return ret_val;
}

bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
                             const Eigen::Ref<const Eigen::MatrixX2d>& position_limits,
                             double epsilon)
{
  if (((joint_positions.array() - epsilon) > position_limits.col(1).array()).any())
    return false;

  if (((joint_positions.array() + epsilon) < position_limits.col(0).array()).any())
    return false;

  return true;
}

void enforcePositionLimits(Eigen::Ref<Eigen::VectorXd> joint_positions,
                           const Eigen::Ref<const Eigen::MatrixX2d>& position_limits)
{
  joint_positions = (joint_positions.array() > position_limits.col(1).array() ||
                     joint_positions.array() < position_limits.col(0).array())
                        .select(position_limits.col(1).array(), joint_positions.array());
}
}  // namespace tesseract_common
