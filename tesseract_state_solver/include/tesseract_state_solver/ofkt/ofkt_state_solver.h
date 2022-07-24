/**
 * @file ofkt_state_solver.h
 * @brief A implementation of the Optimized Forward Kinematic Tree as a state solver.
 *
 * This is based on the paper "A Forward Kinematics Data Structure for Efficient Evolutionary Inverse Kinematics".
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 * @author Levi Armstrong
 * @date August 24, 2020
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
#ifndef TESSERACT_STATE_SOLVER_OFKT_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_OFKT_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_node.h>

namespace tesseract_scene_graph
{
/**
 * @brief An implementation of the Optimized Forward Kinematic Tree as a stat solver
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 */
class OFKTStateSolver : public MutableStateSolver
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<OFKTStateSolver>;
  using ConstPtr = std::shared_ptr<const OFKTStateSolver>;
  using UPtr = std::unique_ptr<OFKTStateSolver>;
  using ConstUPtr = std::unique_ptr<const OFKTStateSolver>;

  OFKTStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix = "");
  OFKTStateSolver(const std::string& root_name);
  ~OFKTStateSolver() override = default;
  OFKTStateSolver(const OFKTStateSolver& other);
  OFKTStateSolver& operator=(const OFKTStateSolver& other);
  OFKTStateSolver(OFKTStateSolver&&) = delete;
  OFKTStateSolver& operator=(OFKTStateSolver&&) = delete;

  void setRevision(int revision) override final;

  int getRevision() const override final;

  void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) override final;
  void setState(const std::unordered_map<std::string, double>& joint_values) override final;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override final;

  SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;
  SceneState getState(const std::unordered_map<std::string, double>& joint_values) const override final;
  SceneState getState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  SceneState getState() const override final;

  SceneState getRandomState() const override final;

  Eigen::MatrixXd getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override final;

  Eigen::MatrixXd getJacobian(const std::unordered_map<std::string, double>& joints_values,
                              const std::string& link_name) const override final;
  Eigen::MatrixXd getJacobian(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const std::string& link_name) const override final;

  std::vector<std::string> getJointNames() const override final;

  std::vector<std::string> getActiveJointNames() const override final;

  std::string getBaseLinkName() const override final;

  std::vector<std::string> getLinkNames() const override final;

  std::vector<std::string> getActiveLinkNames() const override final;

  std::vector<std::string> getStaticLinkNames() const override final;

  bool isActiveLinkName(const std::string& link_name) const override final;

  bool hasLinkName(const std::string& link_name) const override final;

  tesseract_common::VectorIsometry3d getLinkTransforms() const override final;

  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const override final;

  Eigen::Isometry3d getRelativeLinkTransform(const std::string& from_link_name,
                                             const std::string& to_link_name) const override final;

  tesseract_common::KinematicLimits getLimits() const override final;

  bool addLink(const Link& link, const Joint& joint) override final;

  bool moveLink(const Joint& joint) override final;

  bool removeLink(const std::string& name) override final;

  bool replaceJoint(const Joint& joint) override final;

  bool removeJoint(const std::string& name) override final;

  bool moveJoint(const std::string& name, const std::string& parent_link) override final;

  bool changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin) override final;

  bool changeJointPositionLimits(const std::string& name, double lower, double upper) override final;

  bool changeJointVelocityLimits(const std::string& name, double limit) override final;

  bool changeJointAccelerationLimits(const std::string& name, double limit) override final;

  bool insertSceneGraph(const SceneGraph& scene_graph,
                        const Joint& joint,
                        const std::string& prefix = "") override final;

  StateSolver::UPtr clone() const override final;

private:
  SceneState current_state_;                              /**< Current state of the scene */
  std::vector<std::string> joint_names_;                  /**< The link names */
  std::vector<std::string> active_joint_names_;           /**< The active joint names */
  std::vector<std::string> link_names_;                   /**< The link names */
  std::unordered_map<std::string, OFKTNode::UPtr> nodes_; /**< The joint name map to node */
  std::unordered_map<std::string, OFKTNode*> link_map_;   /**< The link name map to node */
  tesseract_common::KinematicLimits limits_;              /**< The kinematic limits */
  OFKTNode::UPtr root_;                                   /**< The root node of the tree */
  int revision_{ 0 };                                     /**< The revision number */

  /** @brief The state solver can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  bool initHelper(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix);

  void clear();

  /** @brief load the active link names */
  void loadActiveLinkNamesRecursive(std::vector<std::string>& active_link_names,
                                    const OFKTNode* node,
                                    bool active) const;

  /** @brief load the static link names */
  void loadStaticLinkNamesRecursive(std::vector<std::string>& static_link_names, const OFKTNode* node) const;

  /**
   * @brief This update the local and world transforms
   * @param node The node to start from
   * @param update_required Indicates if work transform update is required
   */
  void update(OFKTNode* node, bool update_required);

  /**
   * @brief This is a const version of the function above
   * @param node The node to start from
   * @param parent_world_tf The nodes parent's world transformaiton
   * @param update_required Indicates if work transform update is required
   */
  void update(SceneState& state, const OFKTNode* node, Eigen::Isometry3d parent_world_tf, bool update_required) const;

  /**
   * @brief Given a set of joint values calculate the jacobian for the provided link_name
   * @param joints The joint values to calculate the jacobian for
   * @param link_name The link name to calculate the jacobian for
   * @return The calculated geometric jacobian
   */
  Eigen::MatrixXd calcJacobianHelper(const std::unordered_map<std::string, double>& joints,
                                     const std::string& link_name) const;

  /**
   * @brief A helper function used for cloning the OFKTStateSolver
   * @param cloned The cloned object
   * @param node The node cloning
   */
  void cloneHelper(OFKTStateSolver& cloned, const OFKTNode* node) const;

  /**
   * @brief Add a node to the tree
   *
   * The reason that joint_name, parent_link_name, child_link_name are required when joint is provided is to handle
   * add a scene graph with a prefix.
   *
   * @param joint The joint being added to the tree
   * @param joint_name The joints name
   * @param parent_link_name The joints parent link name
   * @param child_link_name The joints child link name
   * @param kinematic_joints The vector to store new kinematic joints added to the solver
   */
  void addNode(const Joint& joint,
               const std::string& joint_name,
               const std::string& parent_link_name,
               const std::string& child_link_name,
               std::vector<JointLimits::ConstPtr>& new_joint_limits);

  /**
   * @brief Remove a node and all of its children
   * @param node The node to remove
   * @param removed_links The removed link names container
   * @param removed_joints The removed joint names container
   * @param removed_active_joints The removed active joint names container
   * @param removed_active_joints_indices The removed active joint names indices container
   */
  void removeNode(OFKTNode* node,
                  std::vector<std::string>& removed_links,
                  std::vector<std::string>& removed_joints,
                  std::vector<std::string>& removed_active_joints,
                  std::vector<long>& removed_active_joints_indices);

  /**
   * @brief This a helper function for moving a link
   * @param new_kinematic_joints The vector to store new kinematic joints added to the solver
   * @param joint The joint performing the move
   */
  void moveLinkHelper(std::vector<JointLimits::ConstPtr>& new_joint_limits, const Joint& joint);

  /**
   * @brief This is a helper function for replacing a joint
   * @param new_kinematic_joints The vector to store new kinematic joints added to the solver
   * @param joint The joint performing the replacement
   */
  void replaceJointHelper(std::vector<JointLimits::ConstPtr>& new_joint_limits, const Joint& joint);

  /**
   * @brief This will clean up member variables joint_names_ and limits_
   * @param removed_links The removed link names container
   * @param removed_joints The removed joint names container
   * @param removed_active_joints The removed active joint names container
   * @param removed_active_joints_indices The removed active joint names indices container
   */
  void removeJointHelper(const std::vector<std::string>& removed_links,
                         const std::vector<std::string>& removed_joints,
                         const std::vector<std::string>& removed_active_joints,
                         const std::vector<long>& removed_active_joints_indices);

  /**
   * @brief appends the new joint limits
   * @param new_joint_limits
   */
  void addNewJointLimits(const std::vector<JointLimits::ConstPtr>& new_joint_limits);
  friend struct ofkt_builder;
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_STATE_SOLVER_OFKT_STATE_SOLVER_H
