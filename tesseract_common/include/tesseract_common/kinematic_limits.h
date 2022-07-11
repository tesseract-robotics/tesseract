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
#ifndef TESSERACT_COMMON_KINEMATIC_LIMITS_H
#define TESSERACT_COMMON_KINEMATIC_LIMITS_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/** @brief Store kinematic limits */
struct KinematicLimits
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  /** @brief The position limits */
  Eigen::MatrixX2d joint_limits;

  /** @brief The velocity limits */
  Eigen::VectorXd velocity_limits;

  /** @brief The acceleration limits */
  Eigen::VectorXd acceleration_limits;

  void resize(Eigen::Index size);

  bool operator==(const KinematicLimits& rhs) const;
  bool operator!=(const KinematicLimits& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief Check if within position limits
 * @param joint_positions The joint position to check
 * @param position_limits The joint limits to perform check
 * @return
 */
template <typename FloatType>
bool isWithinPositionLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_positions,
                            const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>>& position_limits)
{
  auto p = joint_positions.array();
  auto l0 = position_limits.col(0).array();
  auto l1 = position_limits.col(1).array();
  return (!(p > l1).any() && !(p < l0).any());
}

/**
 * @brief Check if joint position is within bounds or relatively equal to a limit
 * @param joint_positions The joint position to check
 * @param joint_limits The joint limits to perform check
 * @param max_diff The max diff when comparing position to limit value max(abs(position - limit)) <= max_diff, if true
 * they are considered equal
 * @param max_rel_diff The max relative diff between position and limit abs(position - limit) <= largest * max_rel_diff,
 * if true considered equal. The largest is the largest of the absolute values of position and limit.
 * @return True if the all position are within the limits or relatively equal to the limit, otherwise false.
 */
template <typename FloatType>
bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_positions,
                             const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>>& position_limits,
                             const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& max_diff,
                             const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& max_rel_diff)
{
  auto p = joint_positions.array();
  auto l0 = position_limits.col(0).array();
  auto l1 = position_limits.col(1).array();
  auto md = max_diff.array();
  auto mrd = max_rel_diff.array();

  auto lower_diff_abs = (p - l0).abs();
  auto lower_diff = (lower_diff_abs <= md);
  auto lower_relative_diff = (lower_diff_abs <= mrd * p.abs().max(l0.abs()));
  auto lower_check = p > l0 || lower_diff || lower_relative_diff;

  auto upper_diff_abs = (p - l1).abs();
  auto upper_diff = (upper_diff_abs <= md);
  auto upper_relative_diff = (upper_diff_abs <= mrd * p.abs().max(l1.abs()));
  auto upper_check = p < l1 || upper_diff || upper_relative_diff;

  return (lower_check.all() && upper_check.all());
}

/**
 * @brief Check if joint position is within bounds or relatively equal to a limit
 * @param joint_positions The joint position to check
 * @param joint_limits The joint limits to perform check
 * @param max_diff The max diff when comparing position to limit value max(abs(position - limit)) <= max_diff, if true
 * they are considered equal
 * @param max_rel_diff The max relative diff between position and limit abs(position - limit) <= largest * max_rel_diff,
 * if true considered equal. The largest is the largest of the absolute values of position and limit.
 * @return True if the all position are within the limits or relatively equal to the limit, otherwise false.
 */
template <typename FloatType>
bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_positions,
                             const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>>& position_limits,
                             FloatType max_diff = static_cast<FloatType>(1e-6),
                             FloatType max_rel_diff = std::numeric_limits<FloatType>::epsilon())
{
  const auto eigen_max_diff = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Constant(joint_positions.size(), max_diff);
  const auto eigen_max_rel_diff =
      Eigen::Matrix<FloatType, Eigen::Dynamic, 1>::Constant(joint_positions.size(), max_rel_diff);
  // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.UndefReturn)
  return satisfiesPositionLimits<FloatType>(joint_positions, position_limits, eigen_max_diff, eigen_max_rel_diff);
}

/**
 * @brief Enforce position to be within the provided limits
 * @param joint_positions The joint position to enforce bounds on
 * @param joint_limits The limits to perform check
 */
template <typename FloatType>
void enforcePositionLimits(Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> joint_positions,
                           const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>>& position_limits)
{
  joint_positions = joint_positions.array().min(position_limits.col(1).array()).max(position_limits.col(0).array());
}
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_KINEMATIC_LIMITS_H
