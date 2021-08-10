/**
 * @file kdl_fwd_kin_chain_nr.cpp
 * @brief Tesseract KDL inverse kinematics chain Newton-Raphson implementation.
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/parser/kdl_parser.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::UPtr KDLInvKinChainNR::clone() const { return std::make_unique<KDLInvKinChainNR>(*this); }

KDLInvKinChainNR::KDLInvKinChainNR(const KDLInvKinChainNR& other) { *this = other; }

KDLInvKinChainNR& KDLInvKinChainNR::operator=(const KDLInvKinChainNR& other)
{
  initialized_ = other.initialized_;
  name_ = other.name_;
  solver_name_ = other.solver_name_;
  kdl_data_ = other.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);

  return *this;
}

// bool KDLInvKinChainNR::update()
//{
//  if (!init(scene_graph_, kdl_data_.base_link_name, kdl_data_.tip_link_name, name_))
//    return false;

//  if (sync_fwd_kin_ != nullptr)
//    synchronize(sync_fwd_kin_);

//  return true;
//}

IKSolutions KDLInvKinChainNR::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                                               int /*segment_num*/) const
{
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  Eigen::VectorXd solution(seed.size());

  // run IK solver
  // TODO: Need to update to handle seg number. Neet to create an IK solver for each seg.
  KDL::Frame kdl_pose;
  EigenToKDL(pose, kdl_pose);
  int status = ik_solver_->CartToJnt(kdl_seed, kdl_pose, kdl_solution);
  if (status < 0)
  {
    // LCOV_EXCL_START
    if (status == KDL::ChainIkSolverPos_NR::E_DEGRADED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, solution converged to <eps in maxiter, but solution is "
                              "degraded in quality (e.g. pseudo-inverse in iksolver is singular)");
    }
    else if (status == KDL::ChainIkSolverPos_NR::E_IKSOLVER_FAILED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, velocity solver failed");
    }
    else if (status == KDL::ChainIkSolverPos_NR::E_NO_CONVERGE)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, no solution found");
    }
#ifndef KDL_LESS_1_4_0
    else if (status == KDL::ChainIkSolverPos_NR::E_MAX_ITERATIONS_EXCEEDED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, max iteration exceeded");
    }
#endif
    else
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK");
    }
    // LCOV_EXCL_STOP
    return IKSolutions();
  }

  KDLToEigen(kdl_solution, solution);

  return { solution };
}

IKSolutions KDLInvKinChainNR::calcInvKin(const Eigen::Isometry3d& pose,
                                         const std::string& /*link_name*/,
                                         const std::string& /*tip_link_name*/,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(pose, seed);
}

std::vector<std::string> KDLInvKinChainNR::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.joint_names;
}

Eigen::Index KDLInvKinChainNR::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

std::string KDLInvKinChainNR::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::vector<std::string> KDLInvKinChainNR::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string KDLInvKinChainNR::getName() const { return name_; }

std::string KDLInvKinChainNR::getSolverName() const { return solver_name_; }

bool KDLInvKinChainNR::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                            const std::vector<std::pair<std::string, std::string>>& chains,
                            std::string name)
{
  initialized_ = false;
  kdl_data_ = KDLChainData();

  name_ = name;

  if (!scene_graph.getLink(scene_graph.getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (!parseSceneGraph(kdl_data_, scene_graph, chains))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL data from Scene Graph");
    return false;
  }

  // Create KDL FK and IK Solver
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainNR::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                            const std::string& base_link,
                            const std::string& tip_link,
                            const std::string& name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.push_back(std::make_pair(base_link, tip_link));
  return init(scene_graph, chains, name);
}

bool KDLInvKinChainNR::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
