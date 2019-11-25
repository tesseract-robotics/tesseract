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
#include <tesseract_scene_graph/parser/kdl_parser.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr KDLInvKinChainLMA::clone() const
{
  auto cloned_invkin = std::make_shared<KDLInvKinChainLMA>();
  cloned_invkin->init(*this);
  return std::move(cloned_invkin);
}

bool KDLInvKinChainLMA::calcInvKinHelper(Eigen::VectorXd& solutions,
                                         const Eigen::Isometry3d& pose,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                                         int /*segment_num*/) const
{
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  solutions.resize(seed.size());

  // run IK solver
  // TODO: Need to update to handle seg number. Neet to create an IK solver for each seg.
  KDL::Frame kdl_pose;
  EigenToKDL(pose, kdl_pose);
  int status = ik_solver_->CartToJnt(kdl_seed, kdl_pose, kdl_solution);
  if (status < 0)
  {
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
    return false;
  }

  KDLToEigen(kdl_solution, solutions);

  return true;
}

bool KDLInvKinChainLMA::calcInvKin(Eigen::VectorXd& solutions,
                                   const Eigen::Isometry3d& pose,
                                   const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(solutions, pose, seed);
}

bool KDLInvKinChainLMA::calcInvKin(Eigen::VectorXd& /*solutions*/,
                                   const Eigen::Isometry3d& /*pose*/,
                                   const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                   const std::string& /*link_name*/) const
{
  assert(checkInitialized());
  assert(false);

  CONSOLE_BRIDGE_logError("This method call is not supported by KDLInvKinChainLMA yet.");

  return false;
}

bool KDLInvKinChainLMA::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != kdl_data_.robot_chain.getNrOfJoints())
  {
    CONSOLE_BRIDGE_logError("Number of joint angles (%d) don't match robot_model (%d)",
                            static_cast<int>(vec.size()),
                            kdl_data_.robot_chain.getNrOfJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < kdl_data_.joint_limits(i, 0)) || (vec(i) > kdl_data_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              kdl_data_.joint_list[static_cast<size_t>(i)].c_str(),
                              kdl_data_.joint_limits(i, 0),
                              vec(i),
                              kdl_data_.joint_limits(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& KDLInvKinChainLMA::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.joint_list;
}

const std::vector<std::string>& KDLInvKinChainLMA::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.link_list;
}

const std::vector<std::string>& KDLInvKinChainLMA::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.active_link_list;
}

const Eigen::MatrixX2d& KDLInvKinChainLMA::getLimits() const { return kdl_data_.joint_limits; }

bool KDLInvKinChainLMA::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                             const std::string& base_link,
                             const std::string& tip_link,
                             std::string name)
{
  initialized_ = false;

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  scene_graph_ = std::move(scene_graph);
  name_ = std::move(name);

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (!parseSceneGraph(kdl_data_, *scene_graph_, base_link, tip_link))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL data from Scene Graph");
    return false;
  }

  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainLMA::init(const KDLInvKinChainLMA& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);
  scene_graph_ = kin.scene_graph_;

  return initialized_;
}
}  // namespace tesseract_kinematics
