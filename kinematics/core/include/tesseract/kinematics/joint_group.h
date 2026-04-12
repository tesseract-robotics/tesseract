/**
 * @file joint_group.h
 * @brief A joint group with forward kinematics, Jacobian, limits methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
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
#ifndef TESSERACT_KINEMATICS_JOINT_GROUP_H
#define TESSERACT_KINEMATICS_JOINT_GROUP_H

#include <tesseract/scene_graph/fwd.h>
#include <tesseract/state_solver/fwd.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/types.h>
#include <tesseract/scene_graph/scene_state.h>
#include <unordered_set>

namespace tesseract::kinematics
{
/**
 * @brief A Joint Group is defined by a list of joint_names.
 * @details Provides the ability to calculate forward kinematics and jacobian.
 * @note This creates an optimized object replace all joints not listed in the provided list with a fixed joint
 * calculated using the provided state. Also the calcFwdKin only return the link transforms in the optimized object
 * which is all active links and root of the scene graph usually.
 */
class JointGroup
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<JointGroup>;
  using ConstPtr = std::shared_ptr<const JointGroup>;
  using UPtr = std::unique_ptr<JointGroup>;
  using ConstUPtr = std::unique_ptr<const JointGroup>;

  virtual ~JointGroup();
  JointGroup(const JointGroup& other);
  JointGroup& operator=(const JointGroup& other);
  JointGroup(JointGroup&&) = default;
  JointGroup& operator=(JointGroup&&) = default;

  /**
   * @brief Create a kinematics group without inverse kinematics for the provided joint names
   * @param name The name of the kinematic group
   * @param joint_names The joints names to create kinematic group from
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  JointGroup(std::string name,
             std::vector<std::string> joint_names,
             const tesseract::scene_graph::SceneGraph& scene_graph,
             const tesseract::scene_graph::SceneState& scene_state);

  /**
   * @brief Calculates tool pose of robot chain (integer-keyed, primary)
   * @details Throws an exception on failures (including uninitialized)
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return A map of LinkId to transforms
   */
  tesseract::common::LinkIdTransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

  /**
   * @brief Calculates tool pose of robot chain (integer-keyed, populate by reference)
   * @details Throws an exception on failures (including uninitialized)
   * @param transforms The integer-keyed map to populate with transforms
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   */
  void calcFwdKin(tesseract::common::LinkIdTransformMap& transforms,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;


  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param link_id The LinkId of the frame that the jacobian is calculated for
   * @return The jacobian at the provided link relative to the joint group base link
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const tesseract::common::LinkId& link_id) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param link_name The frame that the jacobian is calculated for
   * @return The jacobian at the provided link_name relative to the joint group base link
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param link_id The LinkId of the frame that the jacobian is calculated for
   * @param link_point A point on the link that the jacobian is calculated for
   * @return The jacobian at the provided link relative to the joint group base link
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const tesseract::common::LinkId& link_id,
                               const Eigen::Vector3d& link_point) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param link_name The frame that the jacobian is calculated for
   * @param link_point A point on the link that the jacobian is calculated for
   * @return The jacobian at the provided link_name relative to the joint group base link
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const Eigen::Vector3d& link_point) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param base_link_name The frame that the jacobian is calculated in
   * @param link_name The frame that the jacobian is calculated for
   * @return The jacobian at the provided link_name relative to the provided base_link_name
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& base_link_name,
                               const std::string& link_name) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param base_link_name The frame that the jacobian is calculated in
   * @param link_name The frame that the jacobian is calculated for
   * @param link_point A point on the link that the jacobian is calculated for
   * @return The jacobian at the provided link_name relative to the provided base_link_name
   */
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& base_link_name,
                               const std::string& link_name,
                               const Eigen::Vector3d& link_point) const;

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
   * @brief Get list of static link names (with and without geometry) for kinematic object
   *
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames() const;

  /** @brief Get the active link IDs as a vector */
  std::vector<tesseract::common::LinkId> getActiveLinkIds() const;

  /** @brief Get the static link IDs */
  const std::vector<tesseract::common::LinkId>& getStaticLinkIds() const;

  /** @brief Check if link ID exists (O(1)) */
  bool hasLinkId(const tesseract::common::LinkId& link_id) const;

  /** @brief Get the base link ID */
  tesseract::common::LinkId getBaseLinkId() const;

  /**
   * @brief Check if link is an active link (by ID, O(1))
   * @param link_id The link ID to check
   * @return True if active, otherwise false
   */
  bool isActiveLinkId(const tesseract::common::LinkId& link_id) const;

  /**
   * @brief Check if link is an active link
   * @param link_name The link name to check
   * @return True if active, otherwise false
   */
  bool isActiveLinkName(const std::string& link_name) const;

  /**
   * @brief Check if link name exists
   * @param link_name The link name to check for
   * @return True if it exists, otherwise false
   */
  bool hasLinkName(const std::string& link_name) const;

  /** @brief Get the link IDs */
  const std::vector<tesseract::common::LinkId>& getLinkIds() const;

  /** @brief Get the joint IDs */
  const std::vector<tesseract::common::JointId>& getJointIds() const;

  /**
   * @brief Get the kinematic limits (joint, velocity, acceleration, etc.)
   * @return Kinematic Limits
   */
  tesseract::common::KinematicLimits getLimits() const;

  /**
   * @brief Setter for kinematic limits (joint, velocity, acceleration, etc.)
   * @param Kinematic Limits
   */
  void setLimits(const tesseract::common::KinematicLimits& limits);

  /**
   * @brief Get vector indicating which joints are capable of producing redundant solutions
   * @return A vector of joint indices
   */
  std::vector<Eigen::Index> getRedundancyCapableJointIndices() const;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  Eigen::Index numJoints() const;

  /** @brief Get the robot base link name */
  std::string getBaseLinkName() const;

  /** @brief Name of the manipulator */
  std::string getName() const;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const;

protected:
  std::string name_;
  tesseract::scene_graph::SceneState state_;
  std::unique_ptr<tesseract::scene_graph::StateSolver> state_solver_;
  std::vector<tesseract::common::LinkId> link_ids_;
  std::vector<tesseract::common::JointId> joint_ids_;
  std::unordered_set<tesseract::common::LinkId, tesseract::common::LinkId::Hash> link_id_set_;
  std::unordered_set<tesseract::common::LinkId, tesseract::common::LinkId::Hash> active_link_ids_;
  std::vector<tesseract::common::LinkId> static_link_ids_;
  tesseract::common::LinkIdTransformMap static_link_transforms_;
  tesseract::common::KinematicLimits limits_;
  std::vector<Eigen::Index> redundancy_indices_;
  std::vector<Eigen::Index> jacobian_map_;
};

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_JOINT_GROUP_H
