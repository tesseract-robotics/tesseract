/**
 * @file kdl_state_solver.cpp
 * @brief Tesseract scene graph kdl solver implementation.
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/kdl_parser.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>

namespace tesseract_scene_graph
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

StateSolver::UPtr KDLStateSolver::clone() const { return std::make_unique<KDLStateSolver>(*this); }

KDLStateSolver::KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  if (scene_graph.isEmpty())
    throw std::runtime_error("Cannot create a state solver form empty scene!");

  createKDETree(scene_graph);
}

KDLStateSolver::KDLStateSolver(const KDLStateSolver& other) { *this = other; }
KDLStateSolver& KDLStateSolver::operator=(const KDLStateSolver& other)
{
  current_state_ = other.current_state_;
  kdl_tree_ = other.kdl_tree_;
  joint_to_qnr_ = other.joint_to_qnr_;
  kdl_jnt_array_ = other.kdl_jnt_array_;
  limits_ = other.limits_;
  joint_names_ = other.joint_names_;
  return *this;
}

void KDLStateSolver::setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  assert(static_cast<Eigen::Index>(joint_names_.size()) == joint_values.size());
  for (auto i = 0u; i < joint_names_.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names_[i], joint_values[i]))
      current_state_.joints[joint_names_[i]] = joint_values[i];
  }

  calculateTransforms(current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::unordered_map<std::string, double>& joints)
{
  for (auto& joint : joints)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
      current_state_.joints[joint.first] = joint.second;
  }

  calculateTransforms(current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  assert(static_cast<Eigen::Index>(joint_names.size()) == joint_values.size());
  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
      current_state_.joints[joint_names[i]] = joint_values[i];
  }

  calculateTransforms(current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
}

SceneState KDLStateSolver::getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  assert(static_cast<Eigen::Index>(joint_names_.size()) == joint_values.size());
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names_.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names_[i], joint_values[i]))
      state.joints[joint_names_[i]] = joint_values[i];
  }

  calculateTransforms(state, jnt_array, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

SceneState KDLStateSolver::getState(const std::unordered_map<std::string, double>& joints) const
{
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
      state.joints[joint.first] = joint.second;
  }

  calculateTransforms(state, jnt_array, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

SceneState KDLStateSolver::getState(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
      state.joints[joint_names[i]] = joint_values[i];
  }

  calculateTransforms(state, jnt_array, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

const SceneState& KDLStateSolver::getState() const { return current_state_; }

SceneState KDLStateSolver::getRandomState() const
{
  return getState(joint_names_, tesseract_common::generateRandomNumber(limits_.joint_limits));
}

const std::vector<std::string>& KDLStateSolver::getJointNames() const { return joint_names_; }

const tesseract_common::KinematicLimits& KDLStateSolver::getLimits() const { return limits_; }

bool KDLStateSolver::createKDETree(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  KDLTreeData data;
  try
  {
    data = tesseract_scene_graph::parseSceneGraph(scene_graph);
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  kdl_tree_ = data.tree;
  current_state_ = SceneState();
  kdl_jnt_array_.resize(kdl_tree_.getNrOfJoints());
  limits_.joint_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()), 2);
  limits_.velocity_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()));
  limits_.acceleration_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()));
  joint_names_.resize(kdl_tree_.getNrOfJoints());
  joint_to_qnr_.clear();
  size_t j = 0;
  for (const auto& seg : kdl_tree_.getSegments())
  {
    const KDL::Joint& jnt = seg.second.segment.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
    kdl_jnt_array_(seg.second.q_nr) = 0.0;
    current_state_.joints.insert(std::make_pair(jnt.getName(), 0.0));
    joint_names_[j] = jnt.getName();

    // Store joint limits.
    const auto& sj = scene_graph.getJoint(jnt.getName());
    limits_.joint_limits(static_cast<long>(j), 0) = sj->limits->lower;
    limits_.joint_limits(static_cast<long>(j), 1) = sj->limits->upper;
    limits_.velocity_limits(static_cast<long>(j)) = sj->limits->velocity;
    limits_.acceleration_limits(static_cast<long>(j)) = sj->limits->acceleration;

    j++;
  }

  calculateTransforms(current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
  return true;
}

bool KDLStateSolver::setJointValuesHelper(KDL::JntArray& q,
                                          const std::string& joint_name,
                                          const double& joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_name);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    return true;
  }

  CONSOLE_BRIDGE_logError("Tried to set joint name %s which does not exist!", joint_name.c_str());
  return false;
}

void KDLStateSolver::calculateTransformsHelper(SceneState& state,
                                               const KDL::JntArray& q_in,
                                               const KDL::SegmentMap::const_iterator& it,
                                               const Eigen::Isometry3d& parent_frame) const
{
  if (it != kdl_tree_.getSegments().end())
  {
    const KDL::TreeElementType& current_element = it->second;
    KDL::Frame current_frame;
    if (q_in.data.size() > 0)
      current_frame = GetTreeElementSegment(current_element).pose(q_in(GetTreeElementQNr(current_element)));
    else
      current_frame = GetTreeElementSegment(current_element).pose(0);

    Eigen::Isometry3d local_frame = convert(current_frame);
    Eigen::Isometry3d global_frame = parent_frame * local_frame;
    state.link_transforms[current_element.segment.getName()] = global_frame;
    if (current_element.segment.getName() != kdl_tree_.getRootSegment()->first)
      state.joint_transforms[current_element.segment.getJoint().getName()] = global_frame;

    for (auto& child : current_element.children)
    {
      calculateTransformsHelper(state, q_in, child, global_frame);
    }
  }
}

void KDLStateSolver::calculateTransforms(SceneState& state,
                                         const KDL::JntArray& q_in,
                                         const KDL::SegmentMap::const_iterator& it,
                                         const Eigen::Isometry3d& parent_frame) const
{
  calculateTransformsHelper(state, q_in, it, parent_frame);
}

}  // namespace tesseract_scene_graph
