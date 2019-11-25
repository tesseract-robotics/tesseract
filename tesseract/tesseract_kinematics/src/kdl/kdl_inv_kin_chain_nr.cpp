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

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr KDLInvKinChainNR::clone() const
{
  auto cloned_invkin = std::make_shared<KDLInvKinChainNR>();
  cloned_invkin->init(*this);
  return std::move(cloned_invkin);
}

bool KDLInvKinChainNR::calcInvKinHelper(Eigen::VectorXd& solutions,
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
    return false;
  }

  KDLToEigen(kdl_solution, solutions);

  return true;
}

bool KDLInvKinChainNR::calcInvKin(Eigen::VectorXd& solutions,
                                  const Eigen::Isometry3d& pose,
                                  const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(solutions, pose, seed);
}

bool KDLInvKinChainNR::calcInvKin(Eigen::VectorXd& /*solutions*/,
                                  const Eigen::Isometry3d& /*pose*/,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                  const std::string& /*link_name*/) const
{
  assert(checkInitialized());
  assert(false);

  CONSOLE_BRIDGE_logError("This method call is not supported by KDLInvKinChainNR yet.");

  return false;
}

bool KDLInvKinChainNR::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

const std::vector<std::string>& KDLInvKinChainNR::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.joint_list;
}

const std::vector<std::string>& KDLInvKinChainNR::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.link_list;
}

const std::vector<std::string>& KDLInvKinChainNR::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.active_link_list;
}

const Eigen::MatrixX2d& KDLInvKinChainNR::getLimits() const { return kdl_data_.joint_limits; }

bool KDLInvKinChainNR::init(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
                            const std::string& base_link,
                            const std::string& tip_link,
                            const std::string& name)
{
  initialized_ = false;

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  scene_graph_ = scene_graph;
  name_ = name;

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

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainNR::init(const KDLInvKinChainNR& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);
  scene_graph_ = kin.scene_graph_;

  return initialized_;
}

}  // namespace tesseract_kinematics
