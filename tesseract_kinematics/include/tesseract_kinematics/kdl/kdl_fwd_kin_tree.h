/**
 * @file kdl_fwd_kinematic_tree.h
 * @brief Tesseract KDL forward kinematics tree implementation.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KDL_FWD_KINEMATIC_TREE_H
#define TESSERACT_KINEMATICS_KDL_FWD_KINEMATIC_TREE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::KDLFwdKinTree)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief ROS kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 * @todo The tip link should be provided in the init and update srdf to require a tip link
 */
class KDLFwdKinTree : public ForwardKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KDLFwdKinTree>;
  using ConstPtr = std::shared_ptr<const KDLFwdKinTree>;

  KDLFwdKinTree() = default;
  ~KDLFwdKinTree() override = default;
  KDLFwdKinTree(const KDLFwdKinTree&) = delete;
  KDLFwdKinTree& operator=(const KDLFwdKinTree&) = delete;
  KDLFwdKinTree(KDLFwdKinTree&&) = delete;
  KDLFwdKinTree& operator=(KDLFwdKinTree&&) = delete;

  ForwardKinematics::Ptr clone() const override;

  bool update() override;

  Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  tesseract_common::VectorIsometry3d
  calcFwdKinAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name) const override;

  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const std::vector<std::string>& getActiveLinkNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

  void setLimits(tesseract_common::KinematicLimits limits) override;

  unsigned int numJoints() const override;

  const std::string& getBaseLinkName() const override;

  const std::string& getTipLinkName() const override;  // TODO: Should make this be provided

  const std::string& getName() const override;

  const std::string& getSolverName() const override;

  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Initializes Forward Kinematics as tree
   * Creates a forward kinematics tree object
   * @param scene_graph The tesseract scene graph
   * @param joint_names The list of active joints to be considered
   * @param name The name of the kinematic chain
   * @param start_state The initial start state for the tree. This should inlclude all joints in the scene graph
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            const std::vector<std::string>& joint_names,
            std::string name,
            const std::unordered_map<std::string, double>& start_state = std::unordered_map<std::string, double>());

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false };                               /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_; /**< Tesseract Scene Graph */
  KDL::Tree kdl_tree_;                                      /**< KDL tree object */
  std::string name_;                                        /**< Name of the kinematic chain */
  std::string solver_name_{ "KDLFwdKinTree" };              /**< Name of this solver */
  std::vector<std::string> joint_list_;                     /**< List of joint names */
  KDL::JntArray start_state_; /**< Intial state of the tree. Should include all joints in the model. */
  std::unordered_map<std::string, double> input_start_state_; /**< Input start state before it has been translated into
                                                                 KDL types */
  std::vector<int> joint_qnr_; /**< The kdl segment number corrisponding to joint in joint_lists_ */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< The tree joint name to qnr */
  std::vector<std::string> link_list_;                         /**< List of link names */
  std::vector<std::string> active_link_list_; /**< List of link names that move with changes in joint values */
  tesseract_common::KinematicLimits limits_;  /**< Joint limits, velocity limits and acceleration limits */
  std::unique_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const KDLFwdKinTree& kin);

  /** @brief Set the start state for all joints in the tree. */
  void setStartState(std::unordered_map<std::string, double> start_state);

  /** @brief Get an updated kdl joint array */
  KDL::JntArray getKDLJntArray(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

  /** @brief calcFwdKin helper function */
  Eigen::Isometry3d calcFwdKinHelper(const KDL::JntArray& kdl_joints, const std::string& link_name) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian, const KDL::JntArray& kdl_joints, const std::string& link_name) const;

};  // class KDLKinematicTree

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_KINEMATIC_TREE_H
