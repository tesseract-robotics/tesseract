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
#ifndef TESSERACT_ENVIRONMENT_OFKT_STATE_SOLVER_H
#define TESSERACT_ENVIRONMENT_OFKT_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/state_solver.h>
#include <tesseract_environment/ofkt/ofkt_node.h>

namespace tesseract_environment
{
/**
 * @brief An implementation of the Optimized Forward Kinematic Tree as a stat solver
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 */
class OFKTStateSolver : public StateSolver
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<OFKTStateSolver>;
  using ConstPtr = std::shared_ptr<const OFKTStateSolver>;

  OFKTStateSolver() = default;
  ~OFKTStateSolver() override = default;
  OFKTStateSolver(const OFKTStateSolver&) = delete;
  OFKTStateSolver& operator=(const OFKTStateSolver&) = delete;
  OFKTStateSolver(OFKTStateSolver&&) = delete;
  OFKTStateSolver& operator=(OFKTStateSolver&&) = delete;

  StateSolver::Ptr clone() const override;

  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph, int revision = 1) override;

  /**
   * @brief Set the current state of the solver
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                         const std::vector<double>& joint_values) const override;
  EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  EnvState::ConstPtr getCurrentState() const override;

  EnvState::Ptr getRandomState() const override;

  const std::vector<std::string>& getJointNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

private:
  EnvState::Ptr current_state_{ std::make_shared<EnvState>() }; /**< Current state of the environment */
  std::vector<std::string> joint_names_;                        /**< The active joint names */
  std::unordered_map<std::string, OFKTNode::UPtr> nodes_;       /**< The joint name map to node */
  std::unordered_map<std::string, OFKTNode*> link_map_;         /**< The link name map to node */
  tesseract_common::KinematicLimits limits_;                    /**< The kinematic limits */
  OFKTNode::UPtr root_;                                         /**< The root node of the tree */
  int revision_{ 0 };                                           /**< The environment revision number */

  void clear();

  void onEnvironmentChanged(const Commands& commands) override;

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
  void update(EnvState& state, const OFKTNode* node, Eigen::Isometry3d parent_world_tf, bool update_required) const;

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
  void addNode(const tesseract_scene_graph::Joint::ConstPtr& joint,
               const std::string& joint_name,
               const std::string& parent_link_name,
               const std::string& child_link_name,
               std::vector<tesseract_scene_graph::Joint::ConstPtr>& kinematic_joints);

  /**
   * @brief Remove a node and all of its children
   * @param node The node to remove
   * @param removed_joints The removed joint names container
   * @param removed_joints_indices The removed joint names indices container
   */
  void removeNode(OFKTNode* node, std::vector<std::string>& removed_joints, std::vector<long>& removed_joints_indices);

  /**
   * @brief This a helper function for moving a link
   * @param new_kinematic_joints The vector to store new kinematic joints added to the solver
   * @param joint The joint performing the move
   */
  void moveLinkHelper(std::vector<tesseract_scene_graph::Joint::ConstPtr>& new_kinematic_joints,
                      const tesseract_scene_graph::Joint::ConstPtr& joint);

  /**
   * @brief This is a helper function for replacing a joint
   * @param new_kinematic_joints The vector to store new kinematic joints added to the solver
   * @param joint The joint performing the replacement
   */
  void replaceJointHelper(std::vector<tesseract_scene_graph::Joint::ConstPtr>& new_kinematic_joints,
                          const tesseract_scene_graph::Joint::ConstPtr& joint);

  /**
   * @brief This will clean up member variables joint_names_ and limits_
   * @param removed_joints The removed joint names container
   * @param removed_joints_indices The removed joint names indices container
   */
  void removeJointHelper(const std::vector<std::string>& removed_joints,
                         const std::vector<long>& removed_joints_indices);

  friend struct ofkt_builder;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_OFKT_STATE_SOLVER_H
