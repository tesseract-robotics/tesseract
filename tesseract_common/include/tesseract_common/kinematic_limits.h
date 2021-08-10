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
 * @brief Check if joint position is within bounds or relativilty equal to a limit
 * @param joint_positions The joint position to check
 * @param joint_limits The joint limits to perform check
 * @param max_diff The max diff when comparing position to limit value max(abs(position - limit)) <= max_diff, if true
 * they are considered equal
 * @param max_rel_diff The max relative diff between position and limit abs(position - limit) <= largest * max_rel_diff,
 * if true considered equal. The largest is the largest of the absolute values of position and limit.
 * @return True if the all position are within the limits or relativily equal to the limit, otherwise false.
 */
bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
                             const Eigen::Ref<const Eigen::MatrixX2d>& position_limits,
                             double max_diff = 1e-6,
                             double max_rel_diff = std::numeric_limits<double>::epsilon());

/**
 * @brief Enforce position to be within the provided limits
 * @param joint_positions The joint position to enforce bounds on
 * @param joint_limits The limits to perform check
 */
void enforcePositionLimits(Eigen::Ref<Eigen::VectorXd> joint_positions,
                           const Eigen::Ref<const Eigen::MatrixX2d>& position_limits);
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_KINEMATIC_LIMITS_H
