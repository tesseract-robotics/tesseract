#ifndef TESSERACT_KINEMATICS_KINEMATICS_GROUP_H
#define TESSERACT_KINEMATICS_KINEMATICS_GROUP_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
class KinematicGroup
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KinematicGroup>;
  using ConstPtr = std::shared_ptr<const KinematicGroup>;
  using UPtr = std::unique_ptr<KinematicGroup>;
  using ConstUPtr = std::unique_ptr<const KinematicGroup>;

  KinematicGroup() = default;
  virtual ~KinematicGroup() = default;
  KinematicGroup(const KinematicGroup&) = default;
  KinematicGroup& operator=(const KinematicGroup&) = default;
  KinematicGroup(KinematicGroup&&) = default;
  KinematicGroup& operator=(KinematicGroup&&) = default;

  /**
   * @brief Calculates tool pose of robot chain
   * @details Throws an exception on failures (including uninitialized)
   * @param pose Transform of end-of-tip relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   */
  virtual tesseract_common::TransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates joint solutions given a pose.
   * @details If redundant solutions are needed see utility funciton getRedundantSolutions.
   * @param solutions A vector of solutions, so check the size of the vector to determine the number of solutions
   * @param pose Transform of end-of-tip relative to working_frame
   * @param working_frame The link name the pose is relative to. It must be listed in getTipLinkNames().
   * @param tip_link_name The tip link to use for solving inverse kinematics. It must be listed in getTipLinkNames().
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  virtual IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                                 const std::string& working_frame,
                                 const std::string& tip_link_name,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed) const = 0;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   */
  virtual Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                       const std::string& link_name) const = 0;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Get list of all link names (with and without geometry) for kinematic object
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const = 0;

  /**
   * @brief Get list of active link names (with and without geometry) for kinematic object
   *
   * Note: This only includes links that are children of the active joints
   *
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const = 0;

  /**
   * @brief Getter for kinematic limits (joint, velocity, acceleration, etc.)
   * @return Kinematic Limits
   */
  virtual tesseract_common::KinematicLimits getLimits() const = 0;

  /**
   * @brief Setter for kinematic limits (joint, velocity, acceleration, etc.)
   * @param Kinematic Limits
   */
  virtual void setLimits(tesseract_common::KinematicLimits limits) = 0;

  /**
   * @brief Get vector indicating which joints are capable of producing redundant solutions
   * @return A vector of joint indicies
   */
  virtual std::vector<Eigen::Index> getRedundancyCapableJointIndices() const = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual Eigen::Index numJoints() const = 0;

  /** @brief getter for the robot base link name */
  virtual std::string getBaseLinkName() const = 0;

  /** @brief Get the working frames */
  virtual std::vector<std::string> getWorkingFrames() const = 0;

  /** @brief Get the tip link name */
  virtual std::vector<std::string> getTipLinkNames() const = 0;

  /** @brief Name of the maniputlator */
  virtual std::string getName() const = 0;

  /** @brief Clone of the motion group */
  virtual std::unique_ptr<KinematicGroup> clone() const = 0;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_KINEMATICS_GROUP_H
