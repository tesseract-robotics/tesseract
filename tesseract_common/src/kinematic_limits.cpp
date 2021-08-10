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
#include <tesseract_common/serialization.h>

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

bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
                             const Eigen::Ref<const Eigen::MatrixX2d>& position_limits,
                             double max_diff,
                             double max_rel_diff)
{
  auto p = joint_positions.array();
  auto l0 = position_limits.col(0).array();
  auto l1 = position_limits.col(1).array();

  auto lower_diff_abs = (p - l0).abs();
  auto lower_diff = (lower_diff_abs <= max_diff);
  auto lower_relative_diff = (lower_diff_abs <= max_rel_diff * p.abs().max(l0.abs()));
  auto lower_check = p > l0 || lower_diff || lower_relative_diff;

  auto upper_diff_abs = (p - l1).abs();
  auto upper_diff = (upper_diff_abs <= max_diff);
  auto upper_relative_diff = (upper_diff_abs <= max_rel_diff * p.abs().max(l1.abs()));
  auto upper_check = p < l1 || upper_diff || upper_relative_diff;

  return (lower_check.all() && upper_check.all());
}

void enforcePositionLimits(Eigen::Ref<Eigen::VectorXd> joint_positions,
                           const Eigen::Ref<const Eigen::MatrixX2d>& position_limits)
{
  joint_positions = joint_positions.array().min(position_limits.col(1).array()).max(position_limits.col(0).array());
}
}  // namespace tesseract_common

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_common::KinematicLimits::serialize(boost::archive::xml_oarchive& ar,
                                                           const unsigned int version);
template void tesseract_common::KinematicLimits::serialize(boost::archive::xml_iarchive& ar,
                                                           const unsigned int version);
