#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H
#define TESSERACT_MOTION_PLANNERSE_PLANNER_UTILS_H

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
 * This will allows you to provide joint 4 and joint 6 coupling limits. This will still work if the robot is on a position.
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
}

#endif // TESSERACT_MOTION_PLANNERS_PLANNER_UTILS_H
