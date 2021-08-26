/**
 * @file kinematic_group.h
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
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
#ifndef TESSERACT_KINEMATICS_KINEMATIC_GROUP_H
#define TESSERACT_KINEMATICS_KINEMATIC_GROUP_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_scene_graph/adjacency_map.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_kinematics
{
/**
 * @brief The Kinematic Group Inverse Kinematics Input Data
 * @details For simple case your inverse kinetics object only requires a single input to solve for
 * but imagine the case where you have two robots and a positioner. Now each robot requires an
 * input to solve IK for. This structure is to support the ability to provide multiple inputs for
 * kinematic arragements involving multiple robots.
 */
struct KinGroupIKInput
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  KinGroupIKInput(const Eigen::Isometry3d& p, std::string wf, std::string tl)
    : pose(p), working_frame(std::move(wf)), tip_link_name(std::move(tl))
  {
  }

  /** @brief The desired inverse kinematic pose */
  Eigen::Isometry3d pose;

  /**
   * @brief The link name the pose is relative to
   * @details The provided working frame must be listed in InverseKinematics::getWorkingFrames()
   */
  std::string working_frame;

  /**
   * @brief The tip link of the kinematic object to solve IK
   * @details The provided tip link name must be listed in InverseKinematics::getTipLinkNames()
   */
  std::string tip_link_name;  // This defines the internal kinematic group the information belongs to
};

using KinGroupIKInputs = tesseract_common::AlignedVector<KinGroupIKInput>;

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

  ~KinematicGroup() = default;
  KinematicGroup(const KinematicGroup& other);
  KinematicGroup& operator=(const KinematicGroup& other);
  KinematicGroup(KinematicGroup&&) = default;
  KinematicGroup& operator=(KinematicGroup&&) = default;

  KinematicGroup(InverseKinematics::UPtr inv_kin,
                 const tesseract_scene_graph::SceneGraph& scene_graph,
                 const tesseract_scene_graph::SceneState& scene_state);

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
   * @param tip_link_poses The input information to solve inverse kinematics for. There must be an input for each link
   * provided in getTipLinkNames
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  IKSolutions calcInvKin(const KinGroupIKInputs& tip_link_poses, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param jacobian Output jacobian
   * @param joint_angles Input vector of joint angles
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const std::string& base_link_name) const;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names
   */
  std::vector<std::string> getJointNames() const;

  /**
   * @brief Get list of all link names (with and without geometry) for kinematic object
   * @return A vector of link names
   */
  std::vector<std::string> getLinkNames() const;

  /**
   * @brief Get list of active link names (with and without geometry) for kinematic object
   *
   * Note: This only includes links that are children of the active joints
   *
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames() const;

  /**
   * @brief Getter for kinematic limits (joint, velocity, acceleration, etc.)
   * @return Kinematic Limits
   */
  tesseract_common::KinematicLimits getLimits() const;

  /**
   * @brief Setter for kinematic limits (joint, velocity, acceleration, etc.)
   * @param Kinematic Limits
   */
  void setLimits(tesseract_common::KinematicLimits limits);

  /**
   * @brief Get vector indicating which joints are capable of producing redundant solutions
   * @return A vector of joint indicies
   */
  std::vector<Eigen::Index> getRedundancyCapableJointIndices() const;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  Eigen::Index numJoints() const;

  /** @brief getter for the robot base link name */
  std::string getBaseLinkName() const;

  /** @brief Get the working frames */
  std::vector<std::string> getWorkingFrames() const;

  /** @brief Get the tip link name */
  std::vector<std::string> getTipLinkNames() const;

  /** @brief Name of the maniputlator */
  std::string getName() const;

  /** @brief Clone of the motion group */
  std::unique_ptr<KinematicGroup> clone() const;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const;

private:
  std::string name_;
  tesseract_scene_graph::SceneState state_;
  tesseract_scene_graph::KDLStateSolver::UPtr state_solver_;
  InverseKinematics::UPtr inv_kin_;
  Eigen::Isometry3d inv_to_fwd_base_{ Eigen::Isometry3d::Identity() };
  std::vector<std::string> joint_names_;
  std::vector<std::string> working_frames_;
  std::vector<std::string> tip_link_names_;
  std::vector<std::string> static_link_names_;
  tesseract_common::KinematicLimits limits_;
  std::vector<Eigen::Index> redundancy_indices_;
  std::unordered_map<std::string, std::string> inv_working_frames_map_;
  std::unordered_map<std::string, std::string> inv_tip_links_map_;
  std::vector<Eigen::Index> jacobian_map_;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_KINEMATIC_GROUP_H
