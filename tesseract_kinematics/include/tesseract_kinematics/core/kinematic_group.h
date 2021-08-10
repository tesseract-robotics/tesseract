#ifndef TESSERACT_KINEMATICS_KINEMATICS_GROUP_H
#define TESSERACT_KINEMATICS_KINEMATICS_GROUP_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_scene_graph/adjacency_map.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

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

  virtual ~KinematicGroup() = default;
  KinematicGroup(const KinematicGroup& other);
  KinematicGroup& operator=(const KinematicGroup& other);
  KinematicGroup(KinematicGroup&&) = default;
  KinematicGroup& operator=(KinematicGroup&&) = default;

  KinematicGroup(tesseract_kinematics::ForwardKinematics::UPtr fwd_kin,
                 tesseract_kinematics::InverseKinematics::UPtr inv_kin,
                 const tesseract_scene_graph::SceneGraph& scene_graph,
                 const tesseract_common::TransformMap& state);

  /**
   * @brief Calculates tool pose of robot chain
   * @details Throws an exception on failures (including uninitialized)
   * @param pose Transform of end-of-tip relative to root
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   */
  tesseract_common::TransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

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
  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const std::string& working_frame,
                         const std::string& tip_link_name,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @details Throws an exception on failures (including uninitialized)
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name = "") const;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names
   */
  const std::vector<std::string>& getJointNames() const;

  /**
   * @brief Get list of all link names (with and without geometry) for kinematic object
   * @return A vector of link names
   */
  const std::vector<std::string>& getLinkNames() const;

  /**
   * @brief Get list of active link names (with and without geometry) for kinematic object
   *
   * Note: This only includes links that are children of the active joints
   *
   * @return A vector of active link names
   */
  const std::vector<std::string>& getActiveLinkNames() const;

  /**
   * @brief Getter for kinematic limits (joint, velocity, acceleration, etc.)
   * @return Kinematic Limits
   */
  const tesseract_common::KinematicLimits& getLimits() const;

  /**
   * @brief Setter for kinematic limits (joint, velocity, acceleration, etc.)
   * @param Kinematic Limits
   */
  void setLimits(tesseract_common::KinematicLimits limits);

  /**
   * @brief Get vector indicating which joints are capable of producing redundant solutions
   * @return A vector of joint indicies
   */
  const std::vector<Eigen::Index>& getRedundancyCapableJointIndices() const;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  Eigen::Index numJoints() const;

  /** @brief getter for the robot base link name */
  const std::string& getBaseLinkName() const;

  /** @brief Get the tip link name */
  const std::vector<std::string>& getTipLinkNames() const;

  /** @brief Name of the maniputlator */
  const std::string& getName() const;

  /** @brief Clone of the motion group */
  std::unique_ptr<KinematicGroup> clone() const;

private:
  std::string name_;
  ForwardKinematics::UPtr fwd_kin_;
  InverseKinematics::UPtr inv_kin_;
  Eigen::Isometry3d inv_to_fwd_base_{ Eigen::Isometry3d::Identity() };
  std::vector<Eigen::Index> inv_joint_map_; /**< @brief Synchronized joint solution remapping */
  tesseract_scene_graph::AdjacencyMap::UPtr adjacency_map_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_link_names_;
  std::vector<std::string> link_names_;
  std::string base_link_name_;
  std::vector<std::string> tip_link_names_;
  tesseract_common::KinematicLimits limits_;
  std::vector<Eigen::Index> redundancy_indices_;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_KINEMATICS_GROUP_H
