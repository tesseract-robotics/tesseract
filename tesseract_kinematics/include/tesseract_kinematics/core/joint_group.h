/**
 * @file joint_group.h
 * @brief A joint group with forward kinematics, Jacobian, limits methods.
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
#ifndef TESSERACT_KINEMATICS_JOINT_GROUP_H
#define TESSERACT_KINEMATICS_JOINT_GROUP_H

#include <tesseract_common/types.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>

namespace tesseract_kinematics
{
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

  ~JointGroup() = default;
  JointGroup(const JointGroup& other);
  JointGroup& operator=(const JointGroup& other);
  JointGroup(JointGroup&&) = default;
  JointGroup& operator=(JointGroup&&) = default;

  /**
   * @brief Create a kinematics group without inverse kinematics for the provided joint names
   * @details If calcInvKin is called an exception will be thrown
   * @param name The name of the kinematic group
   * @param joint_names The joints names to create kinematic group from
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  JointGroup(std::string name,
             std::vector<std::string> joint_names,
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
  void setLimits(const tesseract_common::KinematicLimits& limits);

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

  /** @brief Name of the maniputlator */
  std::string getName() const;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const;

protected:
  std::string name_;
  tesseract_scene_graph::SceneState state_;
  tesseract_scene_graph::StateSolver::UPtr state_solver_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> static_link_names_;
  tesseract_common::KinematicLimits limits_;
  std::vector<Eigen::Index> redundancy_indices_;
  std::vector<Eigen::Index> jacobian_map_;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_JOINT_GROUP_H
