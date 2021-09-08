/**
 * @file kinematic_group.cpp
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_common/utils.h>

#include <tesseract_scene_graph/kdl_parser.h>

namespace tesseract_kinematics
{
JointGroup::JointGroup(std::string name,
                       std::vector<std::string> joint_names,
                       const tesseract_scene_graph::SceneGraph& scene_graph,
                       const tesseract_scene_graph::SceneState& scene_state)
  : name_(std::move(name)), state_(scene_state), joint_names_(std::move(joint_names))
{
  for (const auto& joint_name : joint_names_)
  {
    if (scene_graph.getJoint(joint_name) == nullptr)
      throw std::runtime_error("Inverse kinematic joint name '" + joint_name +
                               "' does not exist in the provided scene graph!");
  }

  tesseract_scene_graph::KDLTreeData data =
      tesseract_scene_graph::parseSceneGraph(scene_graph, joint_names_, scene_state.joints);
  state_solver_ = std::make_unique<tesseract_scene_graph::KDLStateSolver>(scene_graph, data);
  jacobian_map_.reserve(joint_names_.size());
  std::vector<std::string> solver_jn = state_solver_->getActiveJointNames();
  for (const auto& joint_name : joint_names_)
    jacobian_map_.push_back(
        std::distance(solver_jn.begin(), std::find(solver_jn.begin(), solver_jn.end(), joint_name)));

  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
  for (const auto& link : scene_graph.getLinks())
  {
    auto it = std::find(active_link_names.begin(), active_link_names.end(), link->getName());
    if (it == active_link_names.end())
      static_link_names_.push_back(link->getName());
  }

  limits_.resize(static_cast<Eigen::Index>(joint_names_.size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joint_names_.size()); ++i)
  {
    auto joint = scene_graph.getJoint(joint_names_[static_cast<std::size_t>(i)]);

    // Set limits
    limits_.joint_limits(i, 0) = joint->limits->lower;
    limits_.joint_limits(i, 1) = joint->limits->upper;
    limits_.velocity_limits(i) = joint->limits->velocity;
    limits_.acceleration_limits(i) = joint->limits->acceleration;

    // Set redundancy indices
    switch (joint->type)
    {
      case tesseract_scene_graph::JointType::REVOLUTE:
      case tesseract_scene_graph::JointType::CONTINUOUS:
        redundancy_indices_.push_back(static_cast<Eigen::Index>(i));
        break;
      default:
        break;
    }
  }
}

JointGroup::JointGroup(const JointGroup& other) { *this = other; }

JointGroup& JointGroup::operator=(const JointGroup& other)
{
  name_ = other.name_;
  state_ = other.state_;
  joint_names_ = other.joint_names_;
  static_link_names_ = other.static_link_names_;
  limits_ = other.limits_;
  redundancy_indices_ = other.redundancy_indices_;
  jacobian_map_ = other.jacobian_map_;
  return *this;
}

tesseract_common::TransformMap JointGroup::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  return state_solver_->getState(joint_names_, joint_angles).link_transforms;
}

Eigen::MatrixXd JointGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                         const std::string& link_name,
                                         const std::string& base_link_name) const
{
  Eigen::MatrixXd solver_jac = state_solver_->getJacobian(joint_names_, joint_angles, link_name);

  Eigen::MatrixXd kin_jac(6, numJoints());
  for (Eigen::Index i = 0; i < numJoints(); ++i)
    kin_jac.col(i) = solver_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

  if (base_link_name != getBaseLinkName())
  {
    std::vector<std::string> active_links = getActiveLinkNames();
    if (std::find(active_links.begin(), active_links.end(), base_link_name) != active_links.end())
    {
      tesseract_scene_graph::SceneState state = state_solver_->getState(joint_angles);
      const Eigen::Isometry3d& base_link_tf = state.link_transforms.at(base_link_name);

      Eigen::MatrixXd base_link_jac = state_solver_->getJacobian(joint_names_, joint_angles, base_link_name);
      Eigen::MatrixXd base_kin_jac(6, numJoints());
      for (Eigen::Index i = 0; i < numJoints(); ++i)
        base_link_jac.col(i) = base_link_jac.col(jacobian_map_[static_cast<std::size_t>(i)]);

      tesseract_common::jacobianChangeBase(kin_jac, base_link_tf.inverse());
      tesseract_common::jacobianChangeBase(base_link_jac, base_link_tf.inverse());

      kin_jac = kin_jac + base_link_jac;
    }
    else
    {
      tesseract_common::jacobianChangeBase(kin_jac, state_.link_transforms.at(base_link_name).inverse());
    }
  }

  return kin_jac;
}

bool JointGroup::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != static_cast<Eigen::Index>(joint_names_.size()))
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
                              joint_names_[static_cast<size_t>(i)].c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

std::vector<std::string> JointGroup::getJointNames() const { return joint_names_; }

std::vector<std::string> JointGroup::getLinkNames() const { return state_solver_->getLinkNames(); }

std::vector<std::string> JointGroup::getActiveLinkNames() const { return state_solver_->getActiveLinkNames(); }

tesseract_common::KinematicLimits JointGroup::getLimits() const { return limits_; }

void JointGroup::setLimits(tesseract_common::KinematicLimits limits)
{
  Eigen::Index nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics Group limits assigned are invalid!");

  limits_ = limits;
}

std::vector<Eigen::Index> JointGroup::getRedundancyCapableJointIndices() const { return redundancy_indices_; }

Eigen::Index JointGroup::numJoints() const { return static_cast<Eigen::Index>(joint_names_.size()); }

std::string JointGroup::getBaseLinkName() const { return state_solver_->getBaseLinkName(); }

std::string JointGroup::getName() const { return name_; }

}  // namespace tesseract_kinematics
