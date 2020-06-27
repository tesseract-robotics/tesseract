#ifndef TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_ROBOT_CONFIG_H

namespace tesseract_planning
{
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

#endif  // TESSERACT_COMMAND_LANGUAGE_ROBOT_CONFIG_H
