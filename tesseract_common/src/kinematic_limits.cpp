/**
 * @file kinematic_limits.h
 * @brief Common Tesseract Kinematic Limits and Related Utility Functions
 *
 * @author Levi Armstrong
 * @date March 25, 2021
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

namespace tesseract::common
{
void KinematicLimits::resize(Eigen::Index size)
{
  joint_limits.resize(size, 2);
  velocity_limits.resize(size, 2);
  acceleration_limits.resize(size, 2);
  jerk_limits.resize(size, 2);
}

bool KinematicLimits::operator==(const KinematicLimits& rhs) const
{
  bool ret_val = true;
  ret_val &= (joint_limits.isApprox(rhs.joint_limits, 1e-5));
  ret_val &= (velocity_limits.isApprox(rhs.velocity_limits, 1e-5));
  ret_val &= (acceleration_limits.isApprox(rhs.acceleration_limits, 1e-5));
  ret_val &= (jerk_limits.isApprox(rhs.jerk_limits, 1e-5));
  return ret_val;
}

bool KinematicLimits::operator!=(const KinematicLimits& rhs) const { return !operator==(rhs); }

template bool isWithinLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& values,
                                    const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& limits);

template bool isWithinLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& values,
                                     const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& limits);

template bool satisfiesLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& values,
                                     const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& limits,
                                     float max_diff,
                                     float max_rel_diff);

template bool satisfiesLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& values,
                                      const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& limits,
                                      double max_diff,
                                      double max_rel_diff);

template bool satisfiesLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& values,
                                     const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& limits,
                                     const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& max_diff,
                                     const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& max_rel_diff);

template bool satisfiesLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& values,
                                      const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& limits,
                                      const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& max_diff,
                                      const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& max_rel_diff);

template void enforceLimits<float>(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 1>> values,
                                   const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& limits);

template void enforceLimits<double>(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> values,
                                    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& limits);
}  // namespace tesseract::common
