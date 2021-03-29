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
#include <numeric>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>

namespace tesseract_common
{
/** @brief Store kinematic limits */
struct KinematicLimits
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief The position limits */
  Eigen::MatrixX2d joint_limits;

  /** @brief The velocity limits */
  Eigen::VectorXd velocity_limits;

  /** @brief The acceleration limits */
  Eigen::VectorXd acceleration_limits;

  bool operator==(const KinematicLimits& other) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(joint_limits);
    ar& BOOST_SERIALIZATION_NVP(velocity_limits);
    ar& BOOST_SERIALIZATION_NVP(acceleration_limits);
  }
};

/**
 * @brief Check if joint position is within bounds provided the given epsilon
 * @param joint_positions The joint position to check
 * @param joint_limits The joint limits to perform check
 * @param epsilon The epsilon to leverage for check. Default is float epsilon
 * @return True if bounds are satisfied, otherwise false
 */
bool satisfiesPositionLimits(const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
                             const Eigen::Ref<const Eigen::MatrixX2d>& position_limits,
                             double epsilon = std::numeric_limits<float>::epsilon());

/**
 * @brief Enforce position to be within the provided limits
 * @param joint_positions The joint position to enforce bounds on
 * @param joint_limits The limits to perform check
 */
void enforcePositionLimits(Eigen::Ref<Eigen::VectorXd> joint_positions,
                           const Eigen::Ref<const Eigen::MatrixX2d>& position_limits);
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_KINEMATIC_LIMITS_H
