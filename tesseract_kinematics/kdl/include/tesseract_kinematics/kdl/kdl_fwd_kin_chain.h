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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <unordered_map>
#include <console_bridge/console.h>
#include <mutex>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
static const std::string KDL_FWD_KIN_CHAIN_SOLVER_NAME = "KDLFwdKinChain";

/**
 * @brief KDL kinematic chain implementation.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class KDLFwdKinChain : public ForwardKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KDLFwdKinChain>;
  using ConstPtr = std::shared_ptr<const KDLFwdKinChain>;
  using UPtr = std::unique_ptr<KDLFwdKinChain>;
  using ConstUPtr = std::unique_ptr<const KDLFwdKinChain>;

  ~KDLFwdKinChain() override = default;
  KDLFwdKinChain(const KDLFwdKinChain& other);
  KDLFwdKinChain& operator=(const KDLFwdKinChain& other);
  KDLFwdKinChain(KDLFwdKinChain&&) = delete;
  KDLFwdKinChain& operator=(KDLFwdKinChain&&) = delete;

  /**
   * @brief Initializes Forward Kinematics as chain
   * Creates a forward kinematic chain object
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param solver_name The solver name of the kinematic chain
   */
  KDLFwdKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                 const std::string& base_link,
                 const std::string& tip_link,
                 std::string solver_name = KDL_FWD_KIN_CHAIN_SOLVER_NAME);

  /**
   * @brief Construct Forward Kinematics as chain
   * Creates a forward kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param solver_name The solver name of the kinematic chain
   */
  KDLFwdKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                 const std::vector<std::pair<std::string, std::string> >& chains,
                 std::string solver_name = KDL_FWD_KIN_CHAIN_SOLVER_NAME);

  tesseract_common::TransformMap calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const override final;

  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& joint_link_name) const override final;

  std::string getBaseLinkName() const override final;
  std::vector<std::string> getJointNames() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  Eigen::Index numJoints() const override final;
  std::string getSolverName() const override final;
  ForwardKinematics::UPtr clone() const override final;

private:
  KDLChainData kdl_data_;                                      /**< KDL data parsed from Scene Graph */
  std::string name_;                                           /**< Name of the kinematic chain */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */
  std::string solver_name_{ KDL_FWD_KIN_CHAIN_SOLVER_NAME };   /**< @brief Name of this solver */
  mutable std::mutex mutex_; /**< @brief KDL is not thread safe due to mutable variables in Joint Class */

  /** @brief calcFwdKin helper function */
  tesseract_common::TransformMap calcFwdKinHelperAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian& jacobian,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          int segment_num = -1) const;

};  // class KDLKinematicChain

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KDL_KINEMATIC_CHAIN_H
