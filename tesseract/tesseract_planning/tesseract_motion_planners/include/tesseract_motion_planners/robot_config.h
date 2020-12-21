/**
 * @file robot_config.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_planning
{
/**
 * @brief The RobotConfig enum
 *
 * The first letter refers to 'Flip vs No-Flip', think of the human arm as the robot this would relate to wrist flipped
 * or not flipped.
 *   - Which is indicated by flipping the sign of J5 requiring J4 and J6 to move by +/-180 degrees
 * The second letter refers to 'Up vs Down' think of the human arm as the robot this would relate to elbow up or down
 *   - Which is indicated, J3 is greater than or less than 90 degrees and tool0 positiion x is positive or negative
 * The third letter refers to 'Front vs Back' think of the human arm as the robot this would relate to arm in front or
 * back
 *
 */
enum class RobotConfig
{
  NUT = 0,
  FUT = 1,
  NDT = 2,
  FDT = 3,
  NDB = 4,
  FDB = 5,
  NUB = 6,
  FUB = 7
};

/**
 * @brief Get the configuration of a six axis industrial robot
 * @param robot_kin The kinematics object of the robot.
 * @param joint_values The joint values of the robot.
 * @param sign_correction Correct the sign for Joint 3 and Joint 5 based on the robot manufacturer.
 * @return Robot Config
 */
template <typename FloatType>
inline RobotConfig getRobotConfig(const tesseract_kinematics::ForwardKinematics::ConstPtr& robot_kin,
                                  const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                                  const Eigen::Ref<const Eigen::Vector2i>& sign_correction = Eigen::Vector2i::Ones())
{
  // Get the pose at tool0
  Eigen::Isometry3d pose;
  robot_kin->calcFwdKin(pose, joint_values);

  // Get the base rotated by joint 1
  Eigen::Isometry3d prime_pose(Eigen::AngleAxisd(static_cast<double>(joint_values(0)), Eigen::Vector3d::UnitZ()));

  // Transform tool0 pose into new frame
  Eigen::Isometry3d pose_prime = prime_pose.inverse() * pose;

  // If pose_prime.x is greater than and equal to zero then it is in the forward configuration, otherwise
  // in the backward configuration.

  if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() >= 0 &&
      (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::FUT;
  }

  if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() >= 0 &&
      (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::NUT;
  }

  if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() >= 0 &&
      (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::FDT;
  }

  if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() >= 0 &&
      (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::NDT;
  }

  if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() < 0 &&
      (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::FUB;
  }

  if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() < 0 &&
      (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::NUB;
  }

  if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() < 0 &&
      (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::FDB;
  }

  return RobotConfig::NDB;
}
}  // namespace tesseract_planning

#ifdef SWIG
%template(getRobotConfig) tesseract_planning::getRobotConfig<double>;
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_ROBOT_CONFIG_H
