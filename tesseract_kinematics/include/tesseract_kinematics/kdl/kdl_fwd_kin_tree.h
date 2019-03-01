/**
 * @file kdl_kinematic_tree.h
 * @brief Tesseract KDL kinematics tree implementation.
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
#ifndef TESSERACT_KINEMATICS_KDL_KINEMATIC_TREE_H
#define TESSERACT_KINEMATICS_KDL_KINEMATIC_TREE_H

#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include "tesseract_kinematics/core/forward_kinematics.h"

namespace tesseract_kinematics
{
/**
 * @brief ROS kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class KDLFwdKinTree : public ForwardKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLFwdKinTree() : initialized_(false) {}
  bool calcFwdKin(Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  bool calcFwdKin(Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                  const std::string& link_name) const override;

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                    const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                    const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const Eigen::MatrixX2d& getLimits() const override;

  tesseract_scene_graph::SceneGraphConstPtr getSceneGraph() const { return scene_graph_; }
  unsigned int numJoints() const override { return static_cast<unsigned>(joint_list_.size()); }
  const std::string& getBaseLinkName() const override { return scene_graph_->getRoot(); }
  const std::string& getName() const override { return name_; }

  /**
   * @brief Initializes ROSKin
   * Creates KDL::Tree from Tesseract Scene Graph, populates joint_list_, joint_limits_, and link_list_
   * @param scene_graph The tesseract scene graph
   * @param joint_names The list of active joints to be considered
   * @param start_state The initial start state for the tree. This should inlclude all joints in the model
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraphConstPtr scene_graph,
            const std::vector<std::string>& joint_names,
            const std::unordered_map<std::string, double>& start_state,
            const std::string name);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const
  {
    if (!initialized_)
    {
      CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
    }

    return initialized_;
  }

  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  KDLFwdKinTree& operator=(const KDLFwdKinTree& rhs);

private:
  bool initialized_;                                      /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraphConstPtr scene_graph_; /**< Tesseract Scene Graph */
  KDL::Tree kdl_tree_;                                    /**< KDL tree object */
  std::string name_;                                      /**< Name of the kinematic chain */
  std::vector<std::string> joint_list_;                   /**< List of joint names */
  KDL::JntArray start_state_;                             /**< Intial state of the tree. Should include all joints in the model. */
  std::vector<int> joint_qnr_;                            /**< The kdl segment number corrisponding to joint in joint_lists_ */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_;                 /**< The tree joint name to qnr */
  std::vector<std::string> link_list_;                                         /**< List of link names */
  Eigen::MatrixX2d joint_limits_;                                              /**< Joint limits */
  std::unique_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;                  /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;                        /**< KDL Jacobian Solver */

  /** @brief Set the start state for all joints in the tree. */
  void setStartState(std::unordered_map<std::string, double> start_state);

  /** @brief Get an updated kdl joint array */
  KDL::JntArray getKDLJntArray(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(Eigen::Isometry3d& pose,
                        const KDL::JntArray& kdl_joints,
                        const std::string& link_name) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian,
                          const KDL::JntArray& kdl_joints,
                          const std::string& link_name) const;

};  // class KDLKinematicTree

typedef std::shared_ptr<KDLFwdKinTree> KDLFwdKinTreePtr;
typedef std::shared_ptr<const KDLFwdKinTree> KDLFwdKinTreeConstPtr;
}
#endif  // TESSERACT_KINEMATICS_KDL_KINEMATIC_TREE_H
