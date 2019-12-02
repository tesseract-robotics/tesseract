/**
 * @file kdl_kinematic_tree.cpp
 * @brief Tesseract KDL kinematic tree implementation.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
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

#include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
#include "tesseract_kinematics/kdl/kdl_utils.h"

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardKinematics::Ptr KDLFwdKinTree::clone() const
{
  auto cloned_fwdkin = std::make_shared<KDLFwdKinTree>();
  cloned_fwdkin->init(*this);
  return std::move(cloned_fwdkin);
}

KDL::JntArray KDLFwdKinTree::getKDLJntArray(const std::vector<std::string>& joint_names,
                                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(joint_names.size() == static_cast<unsigned>(joint_angles.size()));

  KDL::JntArray kdl_joints(start_state_);
  for (unsigned i = 0; i < joint_names.size(); ++i)
    kdl_joints.data(joint_qnr_[i]) = joint_angles[i];

  return kdl_joints;
}

void KDLFwdKinTree::setStartState(std::unordered_map<std::string, double> start_state)
{
  KDL::JntArray kdl_joints;
  kdl_joints.resize(static_cast<unsigned>(start_state.size()));
  for (const auto& jnt : start_state)
    kdl_joints.data(joint_to_qnr_.at(jnt.first)) = jnt.second;

  start_state_ = kdl_joints;
}

bool KDLFwdKinTree::calcFwdKinHelper(Eigen::Isometry3d& pose,
                                     const KDL::JntArray& kdl_joints,
                                     const std::string& link_name) const
{
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, link_name) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);

  return true;
}

bool KDLFwdKinTree::calcFwdKin(Eigen::Isometry3d& /*pose*/, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);
  UNUSED(joint_angles);

  CONSOLE_BRIDGE_logError("This method call is not supported by KDLFwdKinTree, must pass link name.");

  return false;
}

bool KDLFwdKinTree::calcFwdKin(tesseract_common::VectorIsometry3d& /*poses*/,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);
  UNUSED(joint_angles);

  CONSOLE_BRIDGE_logError("This method call is not supported by KDLFwdKinTree, must pass link name.");

  return false;
}

bool KDLFwdKinTree::calcFwdKin(Eigen::Isometry3d& pose,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_list_, joint_angles);
  return calcFwdKinHelper(pose, kdl_joint_vals, link_name);
}

bool KDLFwdKinTree::calcJacobianHelper(KDL::Jacobian& jacobian,
                                       const KDL::JntArray& kdl_joints,
                                       const std::string& link_name) const
{
  jacobian.resize(static_cast<unsigned>(kdl_joints.data.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, link_name) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate jacobian");
    return false;
  }

  return true;
}

bool KDLFwdKinTree::calcJacobian(Eigen::Ref<Eigen::MatrixXd> /*jacobian*/,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);
  UNUSED(joint_angles);

  CONSOLE_BRIDGE_logError("This method call is not supported by KDLFwdKinTree, must pass link name.");

  return false;
}

bool KDLFwdKinTree::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                 const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_list_, joint_angles);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
  {
    KDLToEigen(kdl_jacobian, joint_qnr_, jacobian);
    return true;
  }

  return false;
}

bool KDLFwdKinTree::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (static_cast<unsigned>(vec.size()) != joint_list_.size())
  {
    CONSOLE_BRIDGE_logError("Number of joint angles (%d) don't match robot_model (%d)",
                            static_cast<unsigned>(vec.size()),
                            joint_list_.size());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < joint_limits_(i, 0)) || (vec(i) > joint_limits_(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_list_[static_cast<size_t>(i)].c_str(),
                              joint_limits_(i, 0),
                              vec(i),
                              joint_limits_(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& KDLFwdKinTree::getJointNames() const
{
  assert(checkInitialized());
  return joint_list_;
}

const std::vector<std::string>& KDLFwdKinTree::getLinkNames() const
{
  assert(checkInitialized());
  return link_list_;
}

const std::vector<std::string>& KDLFwdKinTree::getActiveLinkNames() const
{
  assert(checkInitialized());
  return active_link_list_;
}

const Eigen::MatrixX2d& KDLFwdKinTree::getLimits() const { return joint_limits_; }

bool KDLFwdKinTree::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                         const std::vector<std::string>& joint_names,
                         std::string name,
                         const std::unordered_map<std::string, double>& start_state)
{
  initialized_ = false;

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Tesseract Scene Graph");
    return false;
  }

  scene_graph_ = std::move(scene_graph);
  name_ = std::move(name);

  std::unordered_map<std::string, double> start_state_zeros;

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (!tesseract_scene_graph::parseSceneGraph(*scene_graph_, kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  if (joint_names.empty())
  {
    CONSOLE_BRIDGE_logError("Joint names must not be empty!");
    return false;
  }

  joint_list_.resize(joint_names.size());
  joint_limits_.resize(static_cast<long int>(joint_names.size()), 2);
  joint_qnr_.resize(joint_names.size());

  unsigned j = 0;
  const std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph_->getLinks();
  link_list_.reserve(links.size());
  for (const auto& link : links)
    link_list_.push_back(link->getName());

  active_link_list_.clear();
  for (const auto& tree_element : kdl_tree_.getSegments())
  {
    const KDL::Segment& seg = tree_element.second.segment;
    const KDL::Joint& jnt = seg.getJoint();

    auto joint_it = std::find(joint_names.begin(), joint_names.end(), jnt.getName());

    if (jnt.getType() != KDL::Joint::None)
    {
      joint_to_qnr_[jnt.getName()] = tree_element.second.q_nr;
      start_state_zeros[jnt.getName()] = 0;
      std::vector<std::string> children = scene_graph_->getJointChildrenNames(jnt.getName());
      active_link_list_.insert(active_link_list_.end(), children.begin(), children.end());
    }

    if (joint_it == joint_names.end())
      continue;

    assert(jnt.getType() != KDL::Joint::None);

    joint_list_[j] = jnt.getName();
    joint_qnr_[j] = static_cast<int>(tree_element.second.q_nr);

    const tesseract_scene_graph::Joint::ConstPtr& joint = scene_graph_->getJoint(jnt.getName());
    joint_limits_(j, 0) = joint->limits->lower;
    joint_limits_(j, 1) = joint->limits->upper;

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optization library but may be nice to have
    if (joint->type == tesseract_scene_graph::JointType::CONTINUOUS &&
        std::abs(joint_limits_(j, 0) - joint_limits_(j, 1)) <= std::numeric_limits<float>::epsilon())
    {
      joint_limits_(j, 0) = -4 * M_PI;
      joint_limits_(j, 1) = +4 * M_PI;
    }
    ++j;
  }
  // Need to remove duplicates link names
  std::sort(active_link_list_.begin(), active_link_list_.end());
  active_link_list_.erase(std::unique(active_link_list_.begin(), active_link_list_.end()), active_link_list_.end());

  assert(joint_names.size() == joint_list_.size());

  fk_solver_ = std::make_unique<KDL::TreeFkSolverPos_recursive>(kdl_tree_);
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(kdl_tree_);

  if (start_state.empty())
    setStartState(start_state_zeros);
  else
    setStartState(start_state);

  initialized_ = true;
  return initialized_;
}

bool KDLFwdKinTree::init(const KDLFwdKinTree& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_tree_ = kin.kdl_tree_;
  joint_limits_ = kin.joint_limits_;
  joint_list_ = kin.joint_list_;
  link_list_ = kin.link_list_;
  active_link_list_ = kin.active_link_list_;
  fk_solver_ = std::make_unique<KDL::TreeFkSolverPos_recursive>(kdl_tree_);
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(kdl_tree_);
  scene_graph_ = kin.scene_graph_;
  start_state_ = kin.start_state_;
  joint_qnr_ = kin.joint_qnr_;
  joint_to_qnr_ = kin.joint_to_qnr_;

  return true;
}

}  // namespace tesseract_kinematics
