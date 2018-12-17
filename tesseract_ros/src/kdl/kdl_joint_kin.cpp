/**
 * @file kdl_joint_kin.cpp
 * @brief Tesseract ROS KDL Joint kinematics implementation.
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
#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/segment.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_ros/kdl/kdl_joint_kin.h"
#include "tesseract_ros/kdl/kdl_utils.h"

namespace tesseract
{
namespace tesseract_ros
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

KDL::JntArray KDLJointKin::getKDLJntArray(const EnvState& state,
                                          const std::vector<std::string>& joint_names,
                                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(joint_names.size() == static_cast<unsigned>(joint_angles.size()));

  KDL::JntArray kdl_joints;
  kdl_joints.resize(static_cast<unsigned>(state.joints.size()));
  for (const auto& jnt : state.joints)
    kdl_joints.data(joint_to_qnr_.at(jnt.first)) = jnt.second;

  for (unsigned i = 0; i < joint_names.size(); ++i)
    kdl_joints.data(joint_qnr_[i]) = joint_angles[i];

  return kdl_joints;
}

bool KDLJointKin::calcFwdKinHelper(Eigen::Isometry3d& pose,
                                   const Eigen::Isometry3d& change_base,
                                   const KDL::JntArray& kdl_joints,
                                   const std::string& link_name) const
{
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, link_name) < 0)
  {
    ROS_ERROR("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);
  pose = change_base * pose;

  return true;
}

bool KDLJointKin::calcFwdKin(Eigen::Isometry3d& /*pose*/,
                             const Eigen::Isometry3d& /*change_base*/,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);

  ROS_ERROR("This method call is not supported by KDLJointKin, must pass link name.");

  return false;
}

bool KDLJointKin::calcFwdKin(Eigen::Isometry3d& pose,
                             const Eigen::Isometry3d& change_base,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                             const std::string& link_name,
                             const EnvState& state) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(state, joint_list_, joint_angles);
  return calcFwdKinHelper(pose, change_base, kdl_joint_vals, link_name);
}

bool KDLJointKin::calcJacobianHelper(KDL::Jacobian& jacobian,
                                     const Eigen::Isometry3d& change_base,
                                     const KDL::JntArray& kdl_joints,
                                     const std::string& link_name) const
{
  jacobian.resize(static_cast<unsigned>(kdl_joints.data.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, link_name) < 0)
  {
    ROS_ERROR("Failed to calculate jacobian");
    return false;
  }

  if (!change_base.matrix().isIdentity())
  {
    KDL::Frame frame;
    EigenToKDL(change_base, frame);
    jacobian.changeBase(frame.M);
  }

  return true;
}

bool KDLJointKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> /*jacobian*/,
                               const Eigen::Isometry3d& /*change_base*/,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(false);

  ROS_ERROR("This method call is not supported by KDLJointKin, must pass link name.");

  return false;
}

bool KDLJointKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                               const Eigen::Isometry3d& change_base,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const EnvState& state) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  bool attached_link =
      std::find(attached_link_list_.begin(), attached_link_list_.end(), link_name) != attached_link_list_.end();
  std::string tree_link_name = (attached_link) ? attached_link_too_parent_link_.at(link_name) : link_name;

  KDL::JntArray kdl_joint_vals = getKDLJntArray(state, joint_list_, joint_angles);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, kdl_joint_vals, tree_link_name))
  {
    if (attached_link)
    {
      Eigen::Isometry3d ref_frame, ref_frame2;
      calcFwdKinHelper(ref_frame, change_base, kdl_joint_vals, tree_link_name);

      ref_frame2 = ref_frame * (state.transforms.at(tree_link_name).inverse() * state.transforms.at(link_name));
      Eigen::VectorXd ref_point = ref_frame2.translation() - ref_frame.translation();
      KDL::Vector pt(ref_point[0], ref_point[1], ref_point[2]);
      kdl_jacobian.changeRefPoint(pt);
    }

    KDLToEigen(kdl_jacobian, joint_qnr_, jacobian);
    return true;
  }
  else
  {
    return false;
  }
}

bool KDLJointKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                               const Eigen::Isometry3d& change_base,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const EnvState& state,
                               const Eigen::Ref<const Eigen::Vector3d>& link_point) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  bool attached_link =
      std::find(attached_link_list_.begin(), attached_link_list_.end(), link_name) != attached_link_list_.end();
  std::string tree_link_name = (attached_link) ? attached_link_too_parent_link_.at(link_name) : link_name;

  KDL::JntArray kdl_joint_vals = getKDLJntArray(state, joint_list_, joint_angles);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, kdl_joint_vals, tree_link_name))
  {
    // When changing ref point you must provide a vector from the current ref
    // point to the new ref point. This is why the forward kin calculation is
    // required, but need to figure out if there is a more direct way to get
    // this information from KDL.
    Eigen::Isometry3d ref_frame;
    calcFwdKinHelper(ref_frame, change_base, kdl_joint_vals, tree_link_name);

    Eigen::VectorXd ref_point = link_point - ref_frame.translation();
    KDL::Vector pt(ref_point(0), ref_point(1), ref_point(2));
    kdl_jacobian.changeRefPoint(pt);

    KDLToEigen(kdl_jacobian, joint_qnr_, jacobian);
    return true;
  }
  else
  {
    return false;
  }
}

bool KDLJointKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (static_cast<unsigned>(vec.size()) != joint_list_.size())
  {
    ROS_ERROR("Number of joint angles (%d) don't match robot_model (%d)",
              static_cast<unsigned>(vec.size()),
              joint_list_.size());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < joint_limits_(i, 0)) || (vec(i) > joint_limits_(i, 1)))
    {
      ROS_WARN("Joint %s is out-of-range (%g < %g < %g)",
               joint_list_[static_cast<size_t>(i)].c_str(),
               joint_limits_(i, 0),
               vec(i),
               joint_limits_(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& KDLJointKin::getJointNames() const
{
  assert(checkInitialized());
  return joint_list_;
}

const std::vector<std::string>& KDLJointKin::getLinkNames() const
{
  assert(checkInitialized());
  return link_list_;
}

const Eigen::MatrixX2d& KDLJointKin::getLimits() const { return joint_limits_; }
void KDLJointKin::addChildrenRecursive(const urdf::LinkConstSharedPtr urdf_link)
{
  // recursively build child links
  link_list_.push_back(urdf_link->name);
  for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    addChildrenRecursive(urdf_link->child_links[i]);
}

bool KDLJointKin::init(urdf::ModelInterfaceConstSharedPtr model,
                       const std::vector<std::string>& joint_names,
                       const std::string name)
{
  initialized_ = false;

  if (model == nullptr)
  {
    ROS_ERROR_STREAM("Null pointer to URDF Model");
    return false;
  }

  model_ = model;
  name_ = name;

  if (!model_->getRoot())
  {
    ROS_ERROR("Invalid URDF in ROSKin::init call");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*model_, kdl_tree_))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (joint_names.empty())
  {
    ROS_ERROR("Joint names must not be empty!");
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

  initialized_ = true;
  return initialized_;
}

void KDLJointKin::addAttachedLink(const std::string& link_name, const std::string& parent_link_name)
{
  auto check = std::find(link_list_.begin(), link_list_.end(), link_name);

  if (check != link_list_.end())
  {
    ROS_WARN("Tried add attached link to manipulator that already exists!");
    return;
  }

  auto it = std::find(link_list_.begin(), link_list_.end(), parent_link_name);
  if (it != link_list_.end())
  {
    attached_link_list_.push_back(link_name);
    link_list_.push_back(link_name);
    attached_link_too_parent_link_[link_name] = parent_link_name;
  }
  else
  {
    ROS_WARN("Tried add attached link to manipulator, but parent link is not "
             "associated with this manipulator!");
  }
}

void KDLJointKin::removeAttachedLink(const std::string& link_name)
{
  auto check = std::find(attached_link_list_.begin(), attached_link_list_.end(), link_name);
  if (check != attached_link_list_.end())
  {
    attached_link_list_.erase(std::remove(attached_link_list_.begin(), attached_link_list_.end(), link_name),
                              attached_link_list_.end());
    link_list_.erase(std::remove(link_list_.begin(), link_list_.end(), link_name), link_list_.end());
    attached_link_too_parent_link_.erase(link_name);
  }
  else
  {
    ROS_WARN("Tried to remove attached link from manipulator which is not "
             "attached!");
  }
}

void KDLJointKin::clearAttachedLinks()
{
  for (const auto& al : attached_link_list_)
    link_list_.erase(std::remove(link_list_.begin(), link_list_.end(), al), link_list_.end());

  attached_link_list_.clear();
  attached_link_too_parent_link_.clear();
}

KDLJointKin& KDLJointKin::operator=(const KDLJointKin& rhs)
{
  initialized_ = rhs.initialized_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(kdl_tree_));
  jac_solver_.reset(new KDL::TreeJntToJacSolver(kdl_tree_));
  model_ = rhs.model_;
  attached_link_list_ = rhs.attached_link_list_;
  attached_link_too_parent_link_ = rhs.attached_link_too_parent_link_;
  joint_qnr_ = rhs.joint_qnr_;
  joint_to_qnr_ = rhs.joint_to_qnr_;

  return *this;
}
}
}
