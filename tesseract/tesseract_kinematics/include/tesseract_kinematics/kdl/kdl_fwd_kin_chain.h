/**
 * @file kdl_fwd_kin_chain.h
 * @brief Tesseract KDL forward kinematics chain implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_KINEMATICS_KDL_FWD_KINEMATIC_CHAIN_H
#define TESSERACT_KINEMATICS_KDL_FWD_KINEMATIC_CHAIN_H

#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
/**
 * @brief KDL kinematic chain implementation.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class KDLFwdKinChain : public ForwardKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLFwdKinChain() : initialized_(false), solver_name_("KDLFwdKinChain") {}
  KDLFwdKinChain(const KDLFwdKinChain& kin);

  bool calcFwdKin(Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override;

  bool calcFwdKin(VectorIsometry3d& poses,
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

  const std::vector<std::string>& getActiveLinkNames() const override;

  const Eigen::MatrixX2d& getLimits() const override;

  tesseract_scene_graph::SceneGraphConstPtr getSceneGraph() const { return scene_graph_; }
  unsigned int numJoints() const override { return kdl_data_.robot_chain.getNrOfJoints(); }
  const std::string& getBaseLinkName() const override { return kdl_data_.base_name; }
  const std::string& getName() const override { return name_; }
  const std::string& getSolverName() const override { return solver_name_; }
  ForwardKinematicsPtr clone() const override { std::make_shared<KDLFwdKinChain>(*this); }

  /**
   * @brief Initializes Forward Kinematics as chain
   * Creates a forward kinematic chain object
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraphConstPtr scene_graph,
            const std::string& base_link,
            const std::string& tip_link,
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

  /** @brief Get the tip link name */
  const std::string& getTipLinkName() const { return kdl_data_.tip_name; }

  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  KDLFwdKinChain& operator=(const KDLFwdKinChain& rhs);

private:
  bool initialized_;                                           /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraphConstPtr scene_graph_;      /**< Tesseract Scene Graph */
  KDLChainData kdl_data_;                                      /**< KDL data parsed from Scene Graph */
  std::string name_;                                           /**< Name of the kinematic chain */
  std::string solver_name_;                                    /**< Name of this solver */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(Eigen::Isometry3d& pose,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                        int segment_num = -1) const;

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(VectorIsometry3d& poses,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                        int segment_num = -1) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          int segment_num = -1) const;

};  // class KDLKinematicChain

typedef std::shared_ptr<KDLFwdKinChain> KDLFwdKinChainPtr;
typedef std::shared_ptr<const KDLFwdKinChain> KDLFwdKinChainConstPtr;
}
#endif  // TESSERACT_KDL_KINEMATIC_CHAIN_H
