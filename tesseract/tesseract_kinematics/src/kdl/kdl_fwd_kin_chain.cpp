/**
 * @file kdl_kinematic_chain.cpp
 * @brief Tesseract KDL kinematics chain implementation.
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardKinematics::Ptr KDLFwdKinChain::clone() const
{
  auto cloned_fwdkin = std::make_shared<KDLFwdKinChain>();
  cloned_fwdkin->init(*this);
  return std::move(cloned_fwdkin);
}

bool KDLFwdKinChain::calcFwdKinHelper(Eigen::Isometry3d& pose,
                                      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                      int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);

  return true;
}

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

bool KDLFwdKinChain::calcFwdKinHelper(tesseract_common::VectorIsometry3d& poses,
                                      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                      int segment_num) const
{
#ifndef KDL_LESS_1_4_0
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  std::vector<KDL::Frame> kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, poses);

  return true;
#else
  return false;
#endif
  UNUSED(poses);
  UNUSED(joint_angles);
  UNUSED(segment_num);
}

TESSERACT_COMMON_IGNORE_WARNINGS_POP

bool KDLFwdKinChain::calcFwdKin(tesseract_common::VectorIsometry3d& poses,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  return calcFwdKinHelper(poses, joint_angles);
}

bool KDLFwdKinChain::calcFwdKin(Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  return calcFwdKinHelper(pose, joint_angles);
}

bool KDLFwdKinChain::calcFwdKin(Eigen::Isometry3d& pose,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(kdl_data_.segment_index.find(link_name) != kdl_data_.segment_index.end());

  int segment_nr = kdl_data_.segment_index.at(link_name);
  return calcFwdKinHelper(pose, joint_angles, segment_nr);
}

bool KDLFwdKinChain::calcJacobianHelper(KDL::Jacobian& jacobian,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                        int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  jacobian.resize(static_cast<unsigned>(joint_angles.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate jacobian");
    return false;
  }

  return true;
}

bool KDLFwdKinChain::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, joint_angles))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }

  return false;
}

bool KDLFwdKinChain::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                  const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(kdl_data_.segment_index.find(link_name) != kdl_data_.segment_index.end());

  int segment_nr = kdl_data_.segment_index.at(link_name);
  KDL::Jacobian kdl_jacobian;

  if (calcJacobianHelper(kdl_jacobian, joint_angles, segment_nr))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }

  return false;
}

bool KDLFwdKinChain::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

const std::vector<std::string>& KDLFwdKinChain::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.joint_list;
}

const std::vector<std::string>& KDLFwdKinChain::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.link_list;
}

const std::vector<std::string>& KDLFwdKinChain::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.active_link_list;
}

const Eigen::MatrixX2d& KDLFwdKinChain::getLimits() const { return kdl_data_.joint_limits; }

bool KDLFwdKinChain::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
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

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);

  initialized_ = true;
  return initialized_;
}

bool KDLFwdKinChain::init(const KDLFwdKinChain& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);
  scene_graph_ = kin.scene_graph_;

  return initialized_;
}

}  // namespace tesseract_kinematics
