/**
 * @file kdl_fwd_kin_chain_nr_jl.cpp
 * @brief Tesseract KDL inverse kinematics chain Newton-Raphson implementation.
 *
 * @author Levi Armstrong, Roelof Oomen
 * @date July 26, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
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
#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kdl_parser.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_jl.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

KDLInvKinChainNR_JL::KDLInvKinChainNR_JL(const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const std::vector<std::pair<std::string, std::string>>& chains,
                                         std::string solver_name)
  : solver_name_(std::move(solver_name))
{
  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (!parseSceneGraph(kdl_data_, scene_graph, chains))
    throw std::runtime_error("Failed to parse KDL data from Scene Graph");

  // Create KDL FK and IK Solver
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      kdl_data_.robot_chain, kdl_data_.q_min, kdl_data_.q_max, *fk_solver_, *ik_vel_solver_);
}

KDLInvKinChainNR_JL::KDLInvKinChainNR_JL(const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const std::string& base_link,
                                         const std::string& tip_link,
                                         std::string solver_name)
  : KDLInvKinChainNR_JL(scene_graph, { std::make_pair(base_link, tip_link) }, std::move(solver_name))
{
}

InverseKinematics::UPtr KDLInvKinChainNR_JL::clone() const { return std::make_unique<KDLInvKinChainNR_JL>(*this); }

KDLInvKinChainNR_JL::KDLInvKinChainNR_JL(const KDLInvKinChainNR_JL& other) { *this = other; }

KDLInvKinChainNR_JL& KDLInvKinChainNR_JL::operator=(const KDLInvKinChainNR_JL& other)
{
  kdl_data_ = other.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      kdl_data_.robot_chain, kdl_data_.q_min, kdl_data_.q_max, *fk_solver_, *ik_vel_solver_);
  solver_name_ = other.solver_name_;

  return *this;
}

IKSolutions KDLInvKinChainNR_JL::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                  const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                  int /*segment_num*/) const
{
  assert(std::abs(1.0 - pose.matrix().determinant()) < 1e-6);  // NOLINT
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  Eigen::VectorXd solution(seed.size());

  // run IK solver
  // TODO: Need to update to handle seg number. Need to create an IK solver for each seg.
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
    if (status == KDL::ChainIkSolverPos_NR_JL::E_DEGRADED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, solution converged to <eps in maxiter, but solution is "
                              "degraded in quality (e.g. pseudo-inverse in iksolver is singular)");
    }
    else if (status == KDL::ChainIkSolverPos_NR_JL::E_IKSOLVERVEL_FAILED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, velocity IK solver failed");
    }
    else if (status == KDL::ChainIkSolverPos_NR_JL::E_FKSOLVERPOS_FAILED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, position FK solver failed");
    }
    else if (status == KDL::ChainIkSolverPos_NR_JL::E_NO_CONVERGE)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, no solution found");
    }
#ifndef KDL_LESS_1_4_0
    else if (status == KDL::ChainIkSolverPos_NR_JL::E_MAX_ITERATIONS_EXCEEDED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, max iteration exceeded");
    }
#endif
    else
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK");
    }
    // LCOV_EXCL_STOP
    return {};
  }

  KDLToEigen(kdl_solution, solution);

  return { solution };
}

IKSolutions KDLInvKinChainNR_JL::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                            const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(kdl_data_.tip_link_name) != tip_link_poses.end());
  return calcInvKinHelper(tip_link_poses.at(kdl_data_.tip_link_name), seed);
}

std::vector<std::string> KDLInvKinChainNR_JL::getJointNames() const { return kdl_data_.joint_names; }

Eigen::Index KDLInvKinChainNR_JL::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

std::string KDLInvKinChainNR_JL::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::string KDLInvKinChainNR_JL::getWorkingFrame() const { return kdl_data_.base_link_name; }

std::vector<std::string> KDLInvKinChainNR_JL::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string KDLInvKinChainNR_JL::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
