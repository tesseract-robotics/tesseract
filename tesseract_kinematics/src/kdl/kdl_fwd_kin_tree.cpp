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
#include <tesseract_scene_graph/kdl_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_common/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

KDLFwdKinTree::KDLFwdKinTree(std::string name,
                             const tesseract_scene_graph::SceneGraph& scene_graph,
                             const tesseract_scene_graph::SceneState& scene_state,
                             const std::vector<std::string>& joint_names,
                             std::string solver_name)
  : name_(std::move(name)), solver_name_(std::move(solver_name))
{
  std::unordered_map<std::string, double> start_state_zeros;

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  tesseract_scene_graph::KDLTreeData data = tesseract_scene_graph::parseSceneGraph(scene_graph);
  kdl_tree_ = data.tree;

  if (joint_names.empty())
    throw std::runtime_error("Joint names must not be empty!");

  for (const auto& joint_name : joint_names)
  {
    if (scene_graph.getJoint(joint_name) == nullptr)
      throw std::runtime_error("The parameter joint_names contains a joint name '" + joint_name +
                               "' which does not exist in the scene graph!");
  }

  joint_names_.resize(joint_names.size());
  joint_qnr_.resize(joint_names.size());

  unsigned j = 0;
  joint_to_qnr_.clear();
  for (const auto& tree_element : kdl_tree_.getSegments())
  {
    const KDL::Segment& seg = tree_element.second.segment;
    const KDL::Joint& jnt = seg.getJoint();

    auto joint_it = std::find(joint_names.begin(), joint_names.end(), jnt.getName());

    if (jnt.getType() != KDL::Joint::None)
    {
      joint_to_qnr_[jnt.getName()] = tree_element.second.q_nr;
      start_state_zeros[jnt.getName()] = 0;
    }

    if (joint_it == joint_names.end())
      continue;

    assert(jnt.getType() != KDL::Joint::None);

    joint_names_[j] = jnt.getName();
    joint_qnr_[j] = static_cast<int>(tree_element.second.q_nr);
    ++j;
  }
  assert(joint_names.size() == joint_names_.size());

  fk_solver_ = std::make_unique<KDL::TreeFkSolverPos_recursive>(kdl_tree_);
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(kdl_tree_);

  if (scene_state.joints.empty())
    setStartState(start_state_zeros);
  else
    setStartState(scene_state.joints);
}
ForwardKinematics::UPtr KDLFwdKinTree::clone() const { return std::make_unique<KDLFwdKinTree>(*this); }

KDLFwdKinTree::KDLFwdKinTree(const KDLFwdKinTree& other) { *this = other; }

KDLFwdKinTree& KDLFwdKinTree::operator=(const KDLFwdKinTree& other)
{
  name_ = other.name_;
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  kdl_tree_ = other.kdl_tree_;
  joint_names_ = other.joint_names_;
  fk_solver_ = std::make_unique<KDL::TreeFkSolverPos_recursive>(kdl_tree_);
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(kdl_tree_);
  start_state_ = other.start_state_;
  joint_qnr_ = other.joint_qnr_;
  joint_to_qnr_ = other.joint_to_qnr_;
  solver_name_ = other.solver_name_;
  return *this;
}

// bool KDLFwdKinTree::update() { return init(scene_graph_, joint_list_, name_, input_start_state_); }

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

void kdlRecursiveFk(tesseract_common::TransformMap& poses,
                    KDL::Frame parent_frame,
                    const KDL::JntArray& kdl_joints,
                    const KDL::SegmentMap::const_iterator& it)
{
  const KDL::TreeElementType& current_element = it->second;
  const KDL::Segment& current_segment = GetTreeElementSegment(current_element);
  KDL::Frame current_world = parent_frame * current_segment.pose(kdl_joints(GetTreeElementQNr(current_element)));

  Eigen::Isometry3d pose;
  KDLToEigen(current_world, pose);
  poses[current_segment.getName()] = pose;

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(current_element);
  for (const auto& child : children)
    kdlRecursiveFk(poses, current_world, kdl_joints, child);
}

tesseract_common::TransformMap KDLFwdKinTree::calcFwdKinHelper(const KDL::JntArray& kdl_joints) const
{
  if (kdl_joints.rows() != kdl_tree_.getNrOfJoints())
    throw std::runtime_error("kdl_joints size is not correct!");

  KDL::Frame kdl_pose = KDL::Frame::Identity();
  tesseract_common::TransformMap all_poses;

  KDL::SegmentMap::const_iterator rootIterator = kdl_tree_.getRootSegment();
  kdlRecursiveFk(all_poses, kdl_pose, kdl_joints, rootIterator);

  tesseract_common::TransformMap poses;
  poses[tip_link_name_] = all_poses[tip_link_name_];
  return poses;
}

tesseract_common::TransformMap KDLFwdKinTree::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(joint_angles.size() == numJoints());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_names_, joint_angles);

  return calcFwdKinHelper(kdl_joint_vals);
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

Eigen::MatrixXd KDLFwdKinTree::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                            const std::string& link_name) const
{
  assert(joint_angles.size() == numJoints());

  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_names_, joint_angles);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
  {
    Eigen::MatrixXd jacobian(6, numJoints());
    KDLToEigen(kdl_jacobian, joint_qnr_, jacobian);
    return jacobian;
  }

  throw std::runtime_error("KDLFwdKinTree: Failed to calculate jacobian.");
}

std::vector<std::string> KDLFwdKinTree::getJointNames() const { return joint_names_; }

Eigen::Index KDLFwdKinTree::numJoints() const { return static_cast<Eigen::Index>(joint_names_.size()); }

std::string KDLFwdKinTree::getBaseLinkName() const { return base_link_name_; }

std::vector<std::string> KDLFwdKinTree::getTipLinkNames() const { return { tip_link_name_ }; }

std::string KDLFwdKinTree::getName() const { return name_; }

std::string KDLFwdKinTree::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
