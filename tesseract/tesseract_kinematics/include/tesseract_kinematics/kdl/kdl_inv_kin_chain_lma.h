/**
 * @file kdl_fwd_kin_chain_lma.h
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
#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class KDLInvKinChainLMA : public InverseKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLInvKinChainLMA() : initialized_(false) {}

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed,
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

  /**
   * @brief Initializes KDL Forward Kinematics
   * Creates KDL::Chain from tesseract scene graph
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
  KDLInvKinChainLMA& operator=(const KDLInvKinChainLMA& rhs);

private:
  bool initialized_;                                           /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraphConstPtr scene_graph_;      /**< Tesseract Scene Graph */
  KDLChainData kdl_data_;                                      /**< KDL data parsed from Scene Graph */
  std::string name_;                                           /**< Name of the kinematic chain */
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;       /**< KDL Inverse kinematic solver */

  /** @brief calcFwdKin helper function */
  bool calcInvKinHelper(Eigen::VectorXd& solutions,
                        const Eigen::Isometry3d& pose,
                        const Eigen::Ref<const Eigen::VectorXd>& seed,
                        int segment_num = -1) const;

};

typedef std::shared_ptr<KDLInvKinChainLMA> KDLInvKinChainLMAPtr;
typedef std::shared_ptr<const KDLInvKinChainLMA> KDLInvKinChainLMAConstPtr;
}
#endif // TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
