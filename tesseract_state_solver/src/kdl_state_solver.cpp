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

  data_ = tesseract_scene_graph::parseSceneGraph(scene_graph);
  processKDLData(scene_graph);
}

KDLStateSolver::KDLStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph, KDLTreeData data)
  : data_(std::move(data))
{
  processKDLData(scene_graph);
}

KDLStateSolver::KDLStateSolver(const KDLStateSolver& other) { *this = other; }
KDLStateSolver& KDLStateSolver::operator=(const KDLStateSolver& other)
{
  current_state_ = other.current_state_;
  data_ = other.data_;
  joint_to_qnr_ = other.joint_to_qnr_;
  joint_qnr_ = other.joint_qnr_;
  kdl_jnt_array_ = other.kdl_jnt_array_;
  limits_ = other.limits_;
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(data_.tree);
  return *this;
}

void KDLStateSolver::setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  assert(static_cast<Eigen::Index>(data_.active_joint_names.size()) == joint_values.size());
  for (auto i = 0U; i < data_.active_joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, data_.active_joint_names[i], joint_values[i]))
      current_state_.joints[data_.active_joint_names[i]] = joint_values[i];
  }

  calculateTransforms(current_state_, kdl_jnt_array_, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::unordered_map<std::string, double>& joint_values)
{
  for (const auto& joint : joint_values)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
      current_state_.joints[joint.first] = joint.second;
  }

  calculateTransforms(current_state_, kdl_jnt_array_, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  assert(static_cast<Eigen::Index>(joint_names.size()) == joint_values.size());
  for (auto i = 0U; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
      current_state_.joints[joint_names[i]] = joint_values[i];
  }

  calculateTransforms(current_state_, kdl_jnt_array_, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());
}

SceneState KDLStateSolver::getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  assert(static_cast<Eigen::Index>(data_.active_joint_names.size()) == joint_values.size());
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0U; i < data_.active_joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, data_.active_joint_names[i], joint_values[i]))
      state.joints[data_.active_joint_names[i]] = joint_values[i];
  }

  calculateTransforms(state, jnt_array, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

SceneState KDLStateSolver::getState(const std::unordered_map<std::string, double>& joint_values) const
{
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (const auto& joint : joint_values)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
      state.joints[joint.first] = joint.second;
  }

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(state, jnt_array, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

SceneState KDLStateSolver::getState(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  SceneState state{ current_state_ };
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0U; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
      state.joints[joint_names[i]] = joint_values[i];
  }

  Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };
  calculateTransforms(state, jnt_array, data_.tree.getRootSegment(), parent_frame);  // NOLINT

  return state;
}

SceneState KDLStateSolver::getState() const { return current_state_; }

SceneState KDLStateSolver::getRandomState() const
{
  Eigen::VectorXd rs = tesseract_common::generateRandomNumber(limits_.joint_limits);
  return getState(data_.active_joint_names, rs);  // NOLINT
}

Eigen::MatrixXd KDLStateSolver::getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                            const std::string& link_name) const
{
  assert(joint_values.size() == data_.tree.getNrOfJoints());
  KDL::JntArray kdl_joint_vals = getKDLJntArray(data_.active_joint_names, joint_values);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
    return convert(kdl_jacobian, joint_qnr_);

  throw std::runtime_error("KDLStateSolver: Failed to calculate jacobian.");
}

Eigen::MatrixXd KDLStateSolver::getJacobian(const std::unordered_map<std::string, double>& joint_values,
                                            const std::string& link_name) const
{
  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_values);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
    return convert(kdl_jacobian, joint_qnr_);

  throw std::runtime_error("KDLStateSolver: Failed to calculate jacobian.");
}

Eigen::MatrixXd KDLStateSolver::getJacobian(const std::vector<std::string>& joint_names,
                                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                            const std::string& link_name) const
{
  KDL::JntArray kdl_joint_vals = getKDLJntArray(joint_names, joint_values);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, kdl_joint_vals, link_name))
    return convert(kdl_jacobian, joint_qnr_);

  throw std::runtime_error("KDLStateSolver: Failed to calculate jacobian.");
}

std::vector<std::string> KDLStateSolver::getJointNames() const { return data_.joint_names; }

std::vector<std::string> KDLStateSolver::getActiveJointNames() const { return data_.active_joint_names; }

std::string KDLStateSolver::getBaseLinkName() const { return data_.base_link_name; }

std::vector<std::string> KDLStateSolver::getLinkNames() const { return data_.link_names; }

std::vector<std::string> KDLStateSolver::getActiveLinkNames() const { return data_.active_link_names; }

std::vector<std::string> KDLStateSolver::getStaticLinkNames() const { return data_.static_link_names; }

bool KDLStateSolver::isActiveLinkName(const std::string& link_name) const
{
  return (std::find(data_.active_link_names.begin(), data_.active_link_names.end(), link_name) !=
          data_.active_link_names.end());
};

bool KDLStateSolver::hasLinkName(const std::string& link_name) const
{
  return (std::find(data_.link_names.begin(), data_.link_names.end(), link_name) != data_.link_names.end());
}

tesseract_common::VectorIsometry3d KDLStateSolver::getLinkTransforms() const
{
  tesseract_common::VectorIsometry3d link_tfs;
  link_tfs.reserve(current_state_.link_transforms.size());
  for (const auto& link_name : data_.link_names)
    link_tfs.push_back(current_state_.link_transforms.at(link_name));

  return link_tfs;
}

Eigen::Isometry3d KDLStateSolver::getLinkTransform(const std::string& link_name) const
{
  return current_state_.link_transforms.at(link_name);
}

Eigen::Isometry3d KDLStateSolver::getRelativeLinkTransform(const std::string& from_link_name,
                                                           const std::string& to_link_name) const
{
  return current_state_.link_transforms.at(from_link_name).inverse() * current_state_.link_transforms.at(to_link_name);
}

tesseract_common::KinematicLimits KDLStateSolver::getLimits() const { return limits_; }

bool KDLStateSolver::processKDLData(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  current_state_ = SceneState();
  kdl_jnt_array_.resize(data_.tree.getNrOfJoints());
  limits_.joint_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()), 2);
  limits_.velocity_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()));
  limits_.acceleration_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()));
  joint_qnr_.resize(data_.tree.getNrOfJoints());
  joint_to_qnr_.clear();
  size_t j = 0;
  for (const auto& seg : data_.tree.getSegments())
  {
    const KDL::Joint& jnt = seg.second.segment.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
    kdl_jnt_array_(seg.second.q_nr) = 0.0;
    current_state_.joints.insert(std::make_pair(jnt.getName(), 0.0));
    data_.active_joint_names[j] = jnt.getName();
    joint_qnr_[j] = static_cast<int>(seg.second.q_nr);

    // Store joint limits.
    const auto& sj = scene_graph.getJoint(jnt.getName());
    limits_.joint_limits(static_cast<long>(j), 0) = sj->limits->lower;
    limits_.joint_limits(static_cast<long>(j), 1) = sj->limits->upper;
    limits_.velocity_limits(static_cast<long>(j)) = sj->limits->velocity;
    limits_.acceleration_limits(static_cast<long>(j)) = sj->limits->acceleration;

    j++;
  }

  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(data_.tree);

  calculateTransforms(current_state_, kdl_jnt_array_, data_.tree.getRootSegment(), Eigen::Isometry3d::Identity());
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
  if (it != data_.tree.getSegments().end())
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
    if (current_element.segment.getName() != data_.tree.getRootSegment()->first)
      state.joint_transforms[current_element.segment.getJoint().getName()] = global_frame;

    for (const auto& child : current_element.children)
    {
      calculateTransformsHelper(state, q_in, child, global_frame);  // NOLINT
    }
  }
}

void KDLStateSolver::calculateTransforms(SceneState& state,
                                         const KDL::JntArray& q_in,
                                         const KDL::SegmentMap::const_iterator& it,
                                         const Eigen::Isometry3d& parent_frame) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  calculateTransformsHelper(state, q_in, it, parent_frame);  // NOLINT
}

bool KDLStateSolver::calcJacobianHelper(KDL::Jacobian& jacobian,
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

KDL::JntArray KDLStateSolver::getKDLJntArray(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  assert(data_.active_joint_names.size() == static_cast<unsigned>(joint_values.size()));

  KDL::JntArray kdl_joints(kdl_jnt_array_);
  for (unsigned i = 0; i < joint_names.size(); ++i)
    kdl_joints.data(joint_to_qnr_.at(joint_names[i])) = joint_values[i];

  return kdl_joints;
}

KDL::JntArray KDLStateSolver::getKDLJntArray(const std::unordered_map<std::string, double>& joint_values) const
{
  assert(data_.active_joint_names.size() == static_cast<unsigned>(joint_values.size()));

  KDL::JntArray kdl_joints(kdl_jnt_array_);
  for (const auto& joint : joint_values)
    kdl_joints.data(joint_to_qnr_.at(joint.first)) = joint.second;

  return kdl_joints;
}

}  // namespace tesseract_scene_graph
