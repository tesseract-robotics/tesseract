/**
 * @file kdl_inv_kin_chain_lma.h
 * @brief Tesseract KDL Inverse kinematics chain Levenberg-Marquardt implementation.
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
#ifndef TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
#define TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <unordered_map>
#include <console_bridge/console.h>
#include <mutex>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
static const std::string KDL_INV_KIN_CHAIN_LMA_SOLVER_NAME = "KDLInvKinChainLMA";

/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class KDLInvKinChainLMA : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KDLInvKinChainLMA>;
  using ConstPtr = std::shared_ptr<const KDLInvKinChainLMA>;
  using UPtr = std::unique_ptr<KDLInvKinChainLMA>;
  using ConstUPtr = std::unique_ptr<const KDLInvKinChainLMA>;

  ~KDLInvKinChainLMA() override = default;
  KDLInvKinChainLMA(const KDLInvKinChainLMA& other);
  KDLInvKinChainLMA& operator=(const KDLInvKinChainLMA& other);
  KDLInvKinChainLMA(KDLInvKinChainLMA&&) = delete;
  KDLInvKinChainLMA& operator=(KDLInvKinChainLMA&&) = delete;

  /**
   * @brief Construct Inverse Kinematics as chain
   * Creates a inverse kinematic chain object
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param solver_name The name of the kinematic chain
   */
  KDLInvKinChainLMA(const tesseract_scene_graph::SceneGraph& scene_graph,
                    const std::string& base_link,
                    const std::string& tip_link,
                    std::string solver_name = KDL_INV_KIN_CHAIN_LMA_SOLVER_NAME);

  /**
   * @brief Construct Inverse Kinematics as chain
   * Creates a inverse kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param solver_name The solver name of the kinematic chain
   */
  KDLInvKinChainLMA(const tesseract_scene_graph::SceneGraph& scene_graph,
                    const std::vector<std::pair<std::string, std::string> >& chains,
                    std::string solver_name = KDL_INV_KIN_CHAIN_LMA_SOLVER_NAME);

  IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  std::vector<std::string> getJointNames() const override final;
  Eigen::Index numJoints() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

private:
  KDLChainData kdl_data_;                                        /**< @brief KDL data parsed from Scene Graph */
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;         /**< @brief KDL Inverse kinematic solver */
  std::string solver_name_{ KDL_INV_KIN_CHAIN_LMA_SOLVER_NAME }; /**< @brief Name of this solver */
  mutable std::mutex mutex_; /**< @brief KDL is not thread safe due to mutable variables in Joint Class */

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose,
                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                               int segment_num = -1) const;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
