/**
 * @file planner_utils.h
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
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief Check if a joint is within limits
 * @param joint_values The joint values of the robot
 * @param limits The robot joint limits
 * @return
 */
template <typename FloatType>
bool isWithinJointLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                         const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((joint_values(i) < limits(i, 0)) || (joint_values(i) > limits(i, 1)))
      return false;

  return true;
}

/**
 * @brief Check if a joint is within limits
 * This will allows you to provide joint 4 and joint 6 coupling limits. This will still work if the robot is on a
 * position.
 * @param joint_values The joint values of the robot
 * @param limits The robot joint limits
 * @param coupling_limits Limits for robot joint 4 and 6 coupling.
 * @return True if within limits otherwise false
 */
template <typename FloatType>
bool isWithinJointLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                         const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits,
                         const Eigen::Vector2d& coupling_limits)
{
  if (isWithinJointLimits<FloatType>(joint_values, limits))
  {
    if (((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) < coupling_limits(0)) ||
        ((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) > coupling_limits(1)))
      return false;
  }
  else
  {
    return false;
  }

  return true;
}

/**
 * @brief Check if the robot is in a valid state
 * @param The robot kinematic representation
 * @param joint_values The joint values of the robot
 * @param limits The robot joint limits
 * @return True if within limits otherwise false
 */
template <typename FloatType>
bool isValidState(const tesseract_kinematics::ForwardKinematics::ConstPtr& robot_kin,
                  const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                  const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  if (!isWithinJointLimits(joint_values, limits))
    return false;

  Eigen::Vector2i sign_correction = Eigen::Vector2i::Ones();
  sign_correction(0) = -1;
  RobotConfig robot_config =
      getRobotConfig<FloatType>(joint_values.tail(robot_kin->numJoints()), robot_kin, sign_correction);

  return !(robot_config != RobotConfig::FUT && robot_config != RobotConfig::NUT);
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H
