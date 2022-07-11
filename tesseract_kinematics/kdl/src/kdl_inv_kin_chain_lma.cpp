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

KDLInvKinChainLMA::KDLInvKinChainLMA(const tesseract_scene_graph::SceneGraph& scene_graph,
                                     const std::vector<std::pair<std::string, std::string>>& chains,
                                     std::string solver_name)
  : solver_name_(std::move(solver_name))
{
  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (!parseSceneGraph(kdl_data_, scene_graph, chains))
    throw std::runtime_error("Failed to parse KDL data from Scene Graph");

  // Create KDL IK Solver
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);
}

KDLInvKinChainLMA::KDLInvKinChainLMA(const tesseract_scene_graph::SceneGraph& scene_graph,
                                     const std::string& base_link,
                                     const std::string& tip_link,
                                     std::string solver_name)
  : KDLInvKinChainLMA(scene_graph, { std::make_pair(base_link, tip_link) }, std::move(solver_name))
{
}

InverseKinematics::UPtr KDLInvKinChainLMA::clone() const { return std::make_unique<KDLInvKinChainLMA>(*this); }

KDLInvKinChainLMA::KDLInvKinChainLMA(const KDLInvKinChainLMA& other) { *this = other; }

KDLInvKinChainLMA& KDLInvKinChainLMA::operator=(const KDLInvKinChainLMA& other)
{
  kdl_data_ = other.kdl_data_;
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);
  solver_name_ = other.solver_name_;

  return *this;
}

IKSolutions KDLInvKinChainLMA::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                int /*segment_num*/) const
{
  assert(std::abs(1.0 - pose.matrix().determinant()) < 1e-6);  // NOLINT
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  Eigen::VectorXd solution(seed.size());

  // run IK solver
  KDL::Frame kdl_pose;
  EigenToKDL(pose, kdl_pose);
  int status{ -1 };
  {
    std::lock_guard<std::mutex> guard(mutex_);
    status = ik_solver_->CartToJnt(kdl_seed, kdl_pose, kdl_solution);
  }
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
    return {};
  }

  KDLToEigen(kdl_solution, solution);

  return { solution };
}

IKSolutions KDLInvKinChainLMA::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(kdl_data_.tip_link_name) != tip_link_poses.end());
  return calcInvKinHelper(tip_link_poses.at(kdl_data_.tip_link_name), seed);
}

std::vector<std::string> KDLInvKinChainLMA::getJointNames() const { return kdl_data_.joint_names; }

Eigen::Index KDLInvKinChainLMA::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

std::string KDLInvKinChainLMA::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::string KDLInvKinChainLMA::getWorkingFrame() const { return kdl_data_.base_link_name; }

std::vector<std::string> KDLInvKinChainLMA::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string KDLInvKinChainLMA::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
