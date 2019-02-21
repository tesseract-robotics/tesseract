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
#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <kdl_parser/kdl_parser.hpp>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
#include "tesseract_kinematics/kdl/kdl_utils.h"

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

bool KDLFwdKinTree::calcFwdKin(Eigen::Isometry3d& /*pose*/,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);

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
  else
  {
    return false;
  }
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
      CONSOLE_BRIDGE_logWarn("Joint %s is out-of-range (%g < %g < %g)",
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

const Eigen::MatrixX2d& KDLFwdKinTree::getLimits() const { return joint_limits_; }
void KDLFwdKinTree::addChildrenRecursive(const urdf::LinkConstSharedPtr urdf_link)
{
  // recursively build child links
  link_list_.push_back(urdf_link->name);
  for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    addChildrenRecursive(urdf_link->child_links[i]);
}

bool KDLFwdKinTree::init(std::shared_ptr<const urdf::ModelInterface> model,
                         const std::vector<std::string>& joint_names,
                         const std::unordered_map<std::string, double>& start_state,
                         const std::string name)
{
  initialized_ = false;

  if (model == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to URDF Model");
    return false;
  }

  model_ = model;
  name_ = name;

  if (!model_->getRoot())
  {
    CONSOLE_BRIDGE_logError("Invalid URDF in ROSKin::init call");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*model_, kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize KDL from URDF model");
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
  link_list_.push_back(kdl_tree_.getRootSegment()->second.segment.getName());
  for (const auto& tree_element : kdl_tree_.getSegments())
  {
    const KDL::Segment& seg = tree_element.second.segment;
    const KDL::Joint& jnt = seg.getJoint();

    std::vector<std::string>::const_iterator joint_it =
        std::find(joint_names.begin(), joint_names.end(), jnt.getName());

    if (jnt.getType() != KDL::Joint::None)
      joint_to_qnr_[jnt.getName()] = tree_element.second.q_nr;

    if (joint_it == joint_names.end())
      continue;

    assert(jnt.getType() != KDL::Joint::None);

    // Add affected link names to list
    std::vector<std::string>::const_iterator link_it = std::find(link_list_.begin(), link_list_.end(), seg.getName());
    if (link_it == link_list_.end())
      addChildrenRecursive(model_->getLink(seg.getName()));

    joint_list_[j] = jnt.getName();
    joint_qnr_[j] = static_cast<int>(tree_element.second.q_nr);

    urdf::JointConstSharedPtr joint = model_->getJoint(jnt.getName());
    joint_limits_(j, 0) = joint->limits->lower;
    joint_limits_(j, 1) = joint->limits->upper;

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optization library but may be nice to have
    if (joint->type == urdf::Joint::CONTINUOUS &&
        std::abs(joint_limits_(j, 0) - joint_limits_(j, 1)) <= std::numeric_limits<float>::epsilon())
    {
      joint_limits_(j, 0) = -4 * M_PI;
      joint_limits_(j, 1) = +4 * M_PI;
    }
    ++j;
  }

  assert(joint_names.size() == joint_list_.size());

  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(kdl_tree_));
  jac_solver_.reset(new KDL::TreeJntToJacSolver(kdl_tree_));

  setStartState(start_state);

  initialized_ = true;
  return initialized_;
}

KDLFwdKinTree& KDLFwdKinTree::operator=(const KDLFwdKinTree& rhs)
{
  initialized_ = rhs.initialized_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(kdl_tree_));
  jac_solver_.reset(new KDL::TreeJntToJacSolver(kdl_tree_));
  model_ = rhs.model_;
  start_state_ = rhs.start_state_;
  joint_qnr_ = rhs.joint_qnr_;
  joint_to_qnr_ = rhs.joint_to_qnr_;

  return *this;
}
}
