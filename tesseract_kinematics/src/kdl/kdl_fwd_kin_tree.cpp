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

#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_common/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardKinematics::Ptr KDLFwdKinTree::clone() const
{
  auto cloned_fwdkin = std::make_shared<KDLFwdKinTree>();
  cloned_fwdkin->init(*this);
  return cloned_fwdkin;
}

bool KDLFwdKinTree::update() { return init(scene_graph_, joint_list_, name_, input_start_state_); }

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
  input_start_state_ = start_state;
  KDL::JntArray kdl_joints;
  kdl_joints.resize(static_cast<unsigned>(start_state.size()));
  for (const auto& jnt : start_state)
    kdl_joints.data(joint_to_qnr_.at(jnt.first)) = jnt.second;

  start_state_ = kdl_joints;
}

Eigen::Isometry3d KDLFwdKinTree::calcFwdKinHelper(const KDL::JntArray& kdl_joints, const std::string& link_name) const
{
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, link_name) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    throw std::runtime_error("KDLFwdKinTree: Failed to calculate forward kinematics.");
  }

  Eigen::Isometry3d pose;
  KDLToEigen(kdl_pose, pose);

  return pose;
}

Eigen::Isometry3d KDLFwdKinTree::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/) const
{
  throw std::runtime_error("This method call is not supported by KDLFwdKinTree, must pass link name.");
}

tesseract_common::VectorIsometry3d
KDLFwdKinTree::calcFwdKinAll(const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/) const
{
  throw std::runtime_error("This method call is not supported by KDLFwdKinTree, must pass link name.");
}

Eigen::Isometry3d KDLFwdKinTree::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                            const std::string& link_name) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());
  assert(std::find(link_list_.begin(), link_list_.end(), link_name) != link_list_.end());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_list_, joint_angles);
  return calcFwdKinHelper(kdl_joint_vals, link_name);
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

Eigen::MatrixXd KDLFwdKinTree::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/) const
{
  throw std::runtime_error("KDLFwdKinTree: This method call is not supported by KDLFwdKinTree, must pass link name.");
}

Eigen::MatrixXd KDLFwdKinTree::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                            const std::string& link_name) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_list_, joint_angles);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
  {
    Eigen::MatrixXd jacobian(6, numJoints());
    KDLToEigen(kdl_jacobian, joint_qnr_, jacobian);
    return jacobian;
  }

  throw std::runtime_error("KDLFwdKinTree: Failed to calculate jacobian.");
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
    if ((vec[i] < limits_.joint_limits(i, 0)) || (vec(i) > limits_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_list_[static_cast<size_t>(i)].c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
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

const tesseract_common::KinematicLimits& KDLFwdKinTree::getLimits() const { return limits_; }

void KDLFwdKinTree::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  limits_ = std::move(limits);
}

unsigned int KDLFwdKinTree::numJoints() const { return static_cast<unsigned>(joint_list_.size()); }

const std::string& KDLFwdKinTree::getBaseLinkName() const { return scene_graph_->getRoot(); }

const std::string& KDLFwdKinTree::getTipLinkName() const { return link_list_.back(); }

const std::string& KDLFwdKinTree::getName() const { return name_; }

const std::string& KDLFwdKinTree::getSolverName() const { return solver_name_; }

tesseract_scene_graph::SceneGraph::ConstPtr KDLFwdKinTree::getSceneGraph() const { return scene_graph_; }

bool KDLFwdKinTree::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                         const std::vector<std::string>& joint_names,
                         std::string name,
                         const std::unordered_map<std::string, double>& start_state)
{
  initialized_ = false;
  kdl_tree_ = KDL::Tree();

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

  for (const auto& joint_name : joint_names)
  {
    if (scene_graph_->getJoint(joint_name) == nullptr)
    {
      CONSOLE_BRIDGE_logError("The parameter joint_names contains a joint name '%s' which does not exist in the scene "
                              "graph!",
                              joint_name.c_str());
      return false;
    }
  }

  joint_list_.resize(joint_names.size());
  limits_.joint_limits.resize(static_cast<long int>(joint_names.size()), 2);
  limits_.velocity_limits.resize(static_cast<long int>(joint_names.size()));
  limits_.acceleration_limits.resize(static_cast<long int>(joint_names.size()));
  joint_qnr_.resize(joint_names.size());

  unsigned j = 0;
  const std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph_->getLinks();
  link_list_.clear();
  link_list_.reserve(links.size());
  for (const auto& link : links)
    link_list_.push_back(link->getName());

  joint_to_qnr_.clear();
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
    limits_.joint_limits(j, 0) = joint->limits->lower;
    limits_.joint_limits(j, 1) = joint->limits->upper;
    limits_.velocity_limits(j) = joint->limits->velocity;
    limits_.acceleration_limits(j) = joint->limits->acceleration;

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optization library but may be nice to have
    if (joint->type == tesseract_scene_graph::JointType::CONTINUOUS &&
        tesseract_common::almostEqualRelativeAndAbs(limits_.joint_limits(j, 0), limits_.joint_limits(j, 1), 1e-5))
    {
      limits_.joint_limits(j, 0) = -4 * M_PI;
      limits_.joint_limits(j, 1) = +4 * M_PI;
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
  limits_ = kin.limits_;
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

bool KDLFwdKinTree::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
