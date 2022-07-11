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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <numeric>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/eigen_serialization.h>

namespace tesseract_common
{
void KinematicLimits::resize(Eigen::Index size)
{
  joint_limits.resize(size, 2);
  velocity_limits.resize(size);
  acceleration_limits.resize(size);
}

bool KinematicLimits::operator==(const KinematicLimits& rhs) const
{
  bool ret_val = true;
  ret_val &= (joint_limits.isApprox(rhs.joint_limits, 1e-5));
  ret_val &= (velocity_limits.isApprox(rhs.velocity_limits, 1e-5));
  ret_val &= (acceleration_limits.isApprox(rhs.acceleration_limits, 1e-5));
  return ret_val;
}

bool KinematicLimits::operator!=(const KinematicLimits& rhs) const { return !operator==(rhs); }

template <class Archive>
void KinematicLimits::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_NVP(joint_limits);
  ar& BOOST_SERIALIZATION_NVP(velocity_limits);
  ar& BOOST_SERIALIZATION_NVP(acceleration_limits);
}

template bool
isWithinPositionLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& joint_positions,
                              const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& position_limits);

template bool
isWithinPositionLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& joint_positions,
                               const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& position_limits);

template bool
satisfiesPositionLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& joint_positions,
                               const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& position_limits,
                               float max_diff,
                               float max_rel_diff);

template bool
satisfiesPositionLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& joint_positions,
                                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& position_limits,
                                double max_diff,
                                double max_rel_diff);

template bool
satisfiesPositionLimits<float>(const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& joint_positions,
                               const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& position_limits,
                               const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& max_diff,
                               const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 1>>& max_rel_diff);

template bool
satisfiesPositionLimits<double>(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& joint_positions,
                                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& position_limits,
                                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& max_diff,
                                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& max_rel_diff);

template void
enforcePositionLimits<float>(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 1>> joint_positions,
                             const Eigen::Ref<const Eigen::Matrix<float, Eigen::Dynamic, 2>>& position_limits);

template void
enforcePositionLimits<double>(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1>> joint_positions,
                              const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& position_limits);
}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::KinematicLimits)
