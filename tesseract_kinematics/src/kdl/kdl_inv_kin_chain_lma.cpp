/**
 * @file kdl_fwd_kin_chain_lma.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/kdl_parser.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::UPtr KDLInvKinChainLMA::clone() const { return std::make_unique<KDLInvKinChainLMA>(*this); }

KDLInvKinChainLMA::KDLInvKinChainLMA(const KDLInvKinChainLMA& other) { *this = other; }

KDLInvKinChainLMA& KDLInvKinChainLMA::operator=(const KDLInvKinChainLMA& other)
{
  initialized_ = other.initialized_;
  name_ = other.name_;
  solver_name_ = other.solver_name_;
  kdl_data_ = other.kdl_data_;
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);

  return *this;
}

// bool KDLInvKinChainLMA::update()
//{
//  if (!init(scene_graph_, kdl_data_.base_link_name, kdl_data_.tip_link_name, name_))
//    return false;

//  if (sync_fwd_kin_ != nullptr)
//    synchronize(sync_fwd_kin_);

//  return true;
//}

IKSolutions KDLInvKinChainLMA::calcInvKinHelper(const Eigen::Isometry3d& pose,
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
#ifndef KDL_LESS_1_4_0
    if (status == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, gradient joints are tool small");
    }
    else if (status == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, increment joints are tool small");
    }
    else if (status == KDL::ChainIkSolverPos_LMA::E_MAX_ITERATIONS_EXCEEDED)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, max iteration exceeded");
    }
#else
    CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK");
#endif
    // LCOV_EXCL_STOP
    return IKSolutions();
  }

  KDLToEigen(kdl_solution, solution);

  return { solution };
}

IKSolutions KDLInvKinChainLMA::calcInvKin(const Eigen::Isometry3d& pose,
                                          const std::string& /*link_name*/,
                                          const std::string& /*tip_link_name*/,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(pose, seed);
}

std::vector<std::string> KDLInvKinChainLMA::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.joint_names;
}

Eigen::Index KDLInvKinChainLMA::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

std::string KDLInvKinChainLMA::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::vector<std::string> KDLInvKinChainLMA::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string KDLInvKinChainLMA::getName() const { return name_; }

std::string KDLInvKinChainLMA::getSolverName() const { return solver_name_; }

bool KDLInvKinChainLMA::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                             const std::vector<std::pair<std::string, std::string>>& chains,
                             std::string name)
{
  initialized_ = false;
  kdl_data_ = KDLChainData();

  name_ = std::move(name);

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

  // Create KDL IK Solver
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainLMA::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                             const std::string& base_link,
                             const std::string& tip_link,
                             std::string name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.push_back(std::make_pair(base_link, tip_link));
  return init(scene_graph, chains, name);
}

bool KDLInvKinChainLMA::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}
}  // namespace tesseract_kinematics
