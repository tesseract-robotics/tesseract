/**
 * @file kinematic_group.cpp
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/joint_group.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/kdl_parser.h>

#include <tesseract/state_solver/kdl/kdl_state_solver.h>

namespace tesseract::kinematics
{
using tesseract::common::JointId;
using tesseract::common::LinkId;
JointGroup::JointGroup(std::string name,
                       std::vector<std::string> joint_names,
                       const tesseract::scene_graph::SceneGraph& scene_graph,
                       const tesseract::scene_graph::SceneState& scene_state)
  : name_(std::move(name)), state_(scene_state)
{
  for (const auto& joint_name : joint_names)
  {
    if (scene_graph.getJoint(joint_name) == nullptr)
      throw std::runtime_error("Joint name '" + joint_name + "' does not exist in the provided scene graph!");
  }

  // Build string-keyed maps from integer-keyed SceneState for parseSceneGraph
  std::unordered_map<std::string, double> joint_values_str;
  for (const auto& joint : scene_graph.getJoints())
  {
    auto it = scene_state.joints.find(joint->getId());
    if (it != scene_state.joints.end())
      joint_values_str[joint->getName()] = it->second;
  }
  tesseract::common::TransformMap floating_joints_str;
  for (const auto& [id, tf] : scene_state.floating_joints)
  {
    for (const auto& joint : scene_graph.getJoints())
    {
      if (joint->getId() == id)
      {
        floating_joints_str[joint->getName()] = tf;
        break;
      }
    }
  }
  tesseract::scene_graph::KDLTreeData data =
      tesseract::scene_graph::parseSceneGraph(scene_graph, joint_names, joint_values_str, floating_joints_str);
  state_solver_ = std::make_unique<tesseract::scene_graph::KDLStateSolver>(scene_graph, data);

  // Build joint IDs from local joint_names parameter
  joint_ids_.reserve(joint_names.size());
  for (const auto& jn : joint_names)
    joint_ids_.push_back(JointId::fromName(jn));

  // Build jacobian_map_
  jacobian_map_.reserve(joint_names.size());
  std::vector<JointId> solver_jids = state_solver_->getActiveJointIds();
  for (const auto& jid : joint_ids_)
    jacobian_map_.push_back(std::distance(solver_jids.begin(), std::find(solver_jids.begin(), solver_jids.end(), jid)));

  // Build active link IDs set
  std::vector<LinkId> active_link_ids = state_solver_->getActiveLinkIds();
  active_link_ids_.insert(active_link_ids.begin(), active_link_ids.end());

  // Build link_ids_, link_id_set_, static_link_ids_, static_link_transforms_
  for (const auto& link : scene_graph.getLinks())
  {
    const auto& link_id = link->getId();
    link_ids_.push_back(link_id);
    link_id_set_.insert(link_id);
    if (active_link_ids_.count(link_id) == 0)
    {
      static_link_ids_.push_back(link_id);
      static_link_transforms_[link_id] = scene_state.link_transforms.at(link_id);
    }
  }

  // Build limits from joint_names parameter
  limits_.resize(static_cast<Eigen::Index>(joint_names.size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joint_names.size()); ++i)
  {
    auto joint = scene_graph.getJoint(joint_names[static_cast<std::size_t>(i)]);

    limits_.joint_limits(i, 0) = joint->limits->lower;
    limits_.joint_limits(i, 1) = joint->limits->upper;
    limits_.velocity_limits(i, 0) = -joint->limits->velocity;
    limits_.velocity_limits(i, 1) = joint->limits->velocity;
    limits_.acceleration_limits(i, 0) = -joint->limits->acceleration;
    limits_.acceleration_limits(i, 1) = joint->limits->acceleration;
    limits_.jerk_limits(i, 0) = -joint->limits->jerk;
    limits_.jerk_limits(i, 1) = joint->limits->jerk;

    switch (joint->type)
    {
      case tesseract::scene_graph::JointType::REVOLUTE:
      case tesseract::scene_graph::JointType::CONTINUOUS:
        redundancy_indices_.push_back(i);
        break;
      default:
        break;
    }
  }

  if (static_link_ids_.size() + active_link_ids_.size() != scene_graph.getLinks().size())
    throw std::runtime_error("JointGroup: Static link names are not correct!");
}

JointGroup::~JointGroup() = default;

JointGroup::JointGroup(const JointGroup& other)
  : name_(other.name_)
  , state_(other.state_)
  , state_solver_(other.state_solver_->clone())
  , link_ids_(other.link_ids_)
  , joint_ids_(other.joint_ids_)
  , link_id_set_(other.link_id_set_)
  , active_link_ids_(other.active_link_ids_)
  , static_link_ids_(other.static_link_ids_)
  , static_link_transforms_(other.static_link_transforms_)
  , limits_(other.limits_)
  , redundancy_indices_(other.redundancy_indices_)
  , jacobian_map_(other.jacobian_map_)
{
}

JointGroup& JointGroup::operator=(const JointGroup& other)
{
  if (this == &other)
    return *this;

  name_ = other.name_;
  state_ = other.state_;
  state_solver_ = other.state_solver_->clone();
  link_ids_ = other.link_ids_;
  joint_ids_ = other.joint_ids_;
  link_id_set_ = other.link_id_set_;
  active_link_ids_ = other.active_link_ids_;
  static_link_ids_ = other.static_link_ids_;
  static_link_transforms_ = other.static_link_transforms_;
  limits_ = other.limits_;
  redundancy_indices_ = other.redundancy_indices_;
  jacobian_map_ = other.jacobian_map_;
  return *this;
}

tesseract::common::LinkIdTransformMap
JointGroup::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  tesseract::common::LinkIdTransformMap transforms;
  state_solver_->getLinkTransforms(transforms, joint_ids_, joint_angles);
  transforms.insert(static_link_transforms_.begin(), static_link_transforms_.end());
  return transforms;
}

void JointGroup::calcFwdKin(tesseract::common::LinkIdTransformMap& transforms,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  state_solver_->getLinkTransforms(transforms, joint_ids_, joint_angles);
  transforms.insert(static_link_transforms_.begin(), static_link_transforms_.end());
}

Eigen::MatrixXd JointGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                         const tesseract::common::LinkId& link_id) const
{
  Eigen::MatrixXd solver_jac = state_solver_->getJacobian(joint_ids_, joint_angles, link_id);

  Eigen::MatrixXd kin_jac(6, numJoints());
  for (Eigen::Index i = 0; i < numJoints(); ++i)
    kin_jac.col(i) = solver_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

  return kin_jac;
}

Eigen::MatrixXd JointGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                         const tesseract::common::LinkId& link_id,
                                         const Eigen::Vector3d& link_point) const
{
  Eigen::MatrixXd kin_jac = calcJacobian(joint_angles, link_id);

  tesseract::common::LinkIdTransformMap transforms;
  state_solver_->getLinkTransforms(transforms, joint_ids_, joint_angles);
  assert(transforms.find(link_id) != transforms.end());

  tesseract::common::jacobianChangeRefPoint(kin_jac, transforms[link_id].linear() * link_point);
  return kin_jac;
}

Eigen::MatrixXd JointGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                         const tesseract::common::LinkId& base_link_id,
                                         const tesseract::common::LinkId& link_id) const
{
  if (base_link_id == getBaseLinkId())
    return calcJacobian(joint_angles, link_id);

  Eigen::MatrixXd solver_jac = state_solver_->getJacobian(joint_ids_, joint_angles, link_id);

  Eigen::MatrixXd kin_jac(6, numJoints());
  for (Eigen::Index i = 0; i < numJoints(); ++i)
    kin_jac.col(i) = solver_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

  if (isActiveLinkId(base_link_id))
  {
    tesseract::common::LinkIdTransformMap transforms;
    state_solver_->getLinkTransforms(transforms, joint_ids_, joint_angles);
    assert(transforms.find(base_link_id) != transforms.end());
    const Eigen::Isometry3d& base_link_tf = transforms[base_link_id];

    Eigen::MatrixXd base_link_jac = state_solver_->getJacobian(joint_ids_, joint_angles, base_link_id);
    Eigen::MatrixXd base_kin_jac(6, numJoints());
    for (Eigen::Index i = 0; i < numJoints(); ++i)
      base_kin_jac.col(i) = base_link_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

    tesseract::common::jacobianChangeBase(kin_jac, base_link_tf.inverse());
    tesseract::common::jacobianChangeBase(base_kin_jac, base_link_tf.inverse());
    kin_jac = kin_jac - base_kin_jac;
  }
  else
  {
    tesseract::common::jacobianChangeBase(kin_jac, state_.link_transforms.at(base_link_id).inverse());
  }

  return kin_jac;
}

Eigen::MatrixXd JointGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                         const tesseract::common::LinkId& base_link_id,
                                         const tesseract::common::LinkId& link_id,
                                         const Eigen::Vector3d& link_point) const
{
  if (base_link_id == getBaseLinkId())
    return calcJacobian(joint_angles, link_id, link_point);

  Eigen::MatrixXd solver_jac = state_solver_->getJacobian(joint_ids_, joint_angles, link_id);

  Eigen::MatrixXd kin_jac(6, numJoints());
  for (Eigen::Index i = 0; i < numJoints(); ++i)
    kin_jac.col(i) = solver_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

  tesseract::common::LinkIdTransformMap transforms;
  state_solver_->getLinkTransforms(transforms, joint_ids_, joint_angles);
  assert(transforms.find(link_id) != transforms.end());
  assert(transforms.find(base_link_id) != transforms.end());
  const Eigen::Isometry3d& link_tf = transforms[link_id];
  const Eigen::Isometry3d& base_link_tf = transforms[base_link_id];

  if (isActiveLinkId(base_link_id))
  {
    Eigen::MatrixXd base_link_jac = state_solver_->getJacobian(joint_ids_, joint_angles, base_link_id);
    Eigen::MatrixXd base_kin_jac(6, numJoints());
    for (Eigen::Index i = 0; i < numJoints(); ++i)
      base_kin_jac.col(i) = base_link_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

    tesseract::common::jacobianChangeBase(kin_jac, base_link_tf.inverse());
    tesseract::common::jacobianChangeRefPoint(kin_jac, (base_link_tf.inverse() * link_tf).linear() * link_point);

    tesseract::common::jacobianChangeBase(base_kin_jac, base_link_tf.inverse());

    kin_jac = kin_jac - base_kin_jac;
  }
  else
  {
    tesseract::common::jacobianChangeBase(kin_jac, base_link_tf.inverse());
    tesseract::common::jacobianChangeRefPoint(kin_jac, (base_link_tf.inverse() * link_tf).linear() * link_point);
  }

  return kin_jac;
}

bool JointGroup::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != numJoints())
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < limits_.joint_limits(i, 0)) || (vec(i) > limits_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_ids_[static_cast<size_t>(i)].name().c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

const std::vector<tesseract::common::JointId>& JointGroup::getJointIds() const { return joint_ids_; }

std::vector<std::string> JointGroup::getJointNames() const
{
  std::vector<std::string> names;
  names.reserve(joint_ids_.size());
  for (const auto& id : joint_ids_)
    names.push_back(id.name());
  return names;
}

const std::vector<tesseract::common::LinkId>& JointGroup::getLinkIds() const { return link_ids_; }

std::vector<std::string> JointGroup::getLinkNames() const
{
  std::vector<std::string> names;
  names.reserve(link_ids_.size());
  for (const auto& id : link_ids_)
    names.push_back(id.name());
  return names;
}

std::vector<tesseract::common::LinkId> JointGroup::getActiveLinkIds() const
{
  return { active_link_ids_.begin(), active_link_ids_.end() };
}

std::vector<std::string> JointGroup::getActiveLinkNames() const
{
  std::vector<std::string> names;
  names.reserve(active_link_ids_.size());
  for (const auto& id : active_link_ids_)
    names.push_back(id.name());
  return names;
}

const std::vector<tesseract::common::LinkId>& JointGroup::getStaticLinkIds() const { return static_link_ids_; }

std::vector<std::string> JointGroup::getStaticLinkNames() const
{
  std::vector<std::string> names;
  names.reserve(static_link_ids_.size());
  for (const auto& id : static_link_ids_)
    names.push_back(id.name());
  return names;
}

tesseract::common::LinkId JointGroup::getBaseLinkId() const { return state_solver_->getBaseLinkId(); }

bool JointGroup::isActiveLinkId(const tesseract::common::LinkId& link_id) const
{
  return active_link_ids_.count(link_id) != 0;
}

bool JointGroup::hasLinkId(const tesseract::common::LinkId& link_id) const { return link_id_set_.count(link_id) != 0; }

tesseract::common::KinematicLimits JointGroup::getLimits() const { return limits_; }

void JointGroup::setLimits(const tesseract::common::KinematicLimits& limits)
{
  Eigen::Index nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.rows() != nj ||
      limits.acceleration_limits.rows() != nj || limits.jerk_limits.rows() != nj)
    throw std::runtime_error("Kinematics Group limits assigned are invalid!");

  limits_ = limits;
}

std::vector<Eigen::Index> JointGroup::getRedundancyCapableJointIndices() const { return redundancy_indices_; }

Eigen::Index JointGroup::numJoints() const { return static_cast<Eigen::Index>(joint_ids_.size()); }

std::string JointGroup::getName() const { return name_; }

}  // namespace tesseract::kinematics
