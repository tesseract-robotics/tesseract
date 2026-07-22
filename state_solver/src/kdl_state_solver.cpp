/**
 * @file kdl_state_solver.cpp
 * @brief Tesseract scene graph kdl solver implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/common/types.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/kdl_parser.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>

namespace tesseract::scene_graph
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
using tesseract::common::JointId;
using tesseract::common::LinkId;

// This is compared directory to the OFKT state solver results so disable code coverage

// LCOV_EXCL_START

thread_local KDL::JntArray KDLStateSolver::kdl_joints_cache;    // NOLINT
thread_local KDL::Jacobian KDLStateSolver::kdl_jacobian_cache;  // NOLINT

StateSolver::UPtr KDLStateSolver::clone() const { return std::make_unique<KDLStateSolver>(*this); }

KDLStateSolver::KDLStateSolver(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  if (scene_graph.isEmpty())
    throw std::runtime_error("Cannot create a state solver form empty scene!");

  data_ = tesseract::scene_graph::parseSceneGraph(scene_graph);
  processKDLData(scene_graph);
}

KDLStateSolver::KDLStateSolver(const tesseract::scene_graph::SceneGraph& scene_graph, KDLTreeData data)
  : data_(std::move(data))
{
  processKDLData(scene_graph);
}

KDLStateSolver::KDLStateSolver(const KDLStateSolver& other) { *this = other; }
KDLStateSolver& KDLStateSolver::operator=(const KDLStateSolver& other)
{
  if (this == &other)
    return *this;

  current_state_ = other.current_state_;
  data_ = other.data_;
  joint_to_qnr_ = other.joint_to_qnr_;
  joint_qnr_ = other.joint_qnr_;
  kdl_jnt_array_ = other.kdl_jnt_array_;
  limits_ = other.limits_;
  active_link_ids_set_ = other.active_link_ids_set_;
  link_ids_set_ = other.link_ids_set_;
  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(data_.tree);

  // Rebuild pointer-keyed cache using our own tree (pointers from other's tree are invalid)
  segment_id_cache_.clear();
  for (const auto& seg : data_.tree.getSegments())
  {
    SegmentIdCache entry;
    entry.link_id = LinkId(seg.first);
    entry.joint_id = JointId(seg.second.segment.getJoint().getName());
    segment_id_cache_[&seg.second] = entry;
  }
  root_element_ = &data_.tree.getRootSegment()->second;
  return *this;
}

void KDLStateSolver::setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::JointIdTransformMap& /*floating_joint_values*/)
{
  assert(static_cast<Eigen::Index>(data_.active_joint_ids.size()) == joint_values.size());
  for (auto i = 0U; i < data_.active_joint_ids.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, data_.active_joint_ids[i], joint_values[i]))
      current_state_.joints[data_.active_joint_ids[i]] = joint_values[i];
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(current_state_.link_transforms,
                      current_state_.joint_transforms,
                      kdl_jnt_array_,
                      data_.tree.getRootSegment(),
                      parent_frame);
}

void KDLStateSolver::setState(const SceneState::JointValues& joint_values,
                              const tesseract::common::JointIdTransformMap& /*floating_joint_values*/)
{
  for (const auto& [id, val] : joint_values)
  {
    if (setJointValuesHelper(kdl_jnt_array_, id, val))
      current_state_.joints[id] = val;
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(current_state_.link_transforms,
                      current_state_.joint_transforms,
                      kdl_jnt_array_,
                      data_.tree.getRootSegment(),
                      parent_frame);
}

void KDLStateSolver::setState(const std::vector<JointId>& joint_ids,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::JointIdTransformMap& /*floating_joint_values*/)
{
  assert(static_cast<Eigen::Index>(joint_ids.size()) == joint_values.size());
  for (auto i = 0U; i < joint_ids.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_ids[i], joint_values[i]))
      current_state_.joints[joint_ids[i]] = joint_values[i];
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(current_state_.link_transforms,
                      current_state_.joint_transforms,
                      kdl_jnt_array_,
                      data_.tree.getRootSegment(),
                      parent_frame);
}

void KDLStateSolver::setState(const tesseract::common::JointIdTransformMap& /*floating_joint_values*/)
{
  throw std::runtime_error("KDLStateSolver, not supported!");
}

SceneState KDLStateSolver::getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                    const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  SceneState state{ current_state_ };

  kdl_joints_cache = kdl_jnt_array_;

  for (auto i = 0U; i < data_.active_joint_ids.size(); ++i)
  {
    if (setJointValuesHelper(kdl_joints_cache, data_.active_joint_ids[i], joint_values[i]))
      state.joints[data_.active_joint_ids[i]] = joint_values[i];
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(
      state.link_transforms, state.joint_transforms, kdl_joints_cache, data_.tree.getRootSegment(), parent_frame);
  return state;
}

SceneState KDLStateSolver::getState(const SceneState::JointValues& joint_values,
                                    const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  SceneState state{ current_state_ };

  kdl_joints_cache = kdl_jnt_array_;

  for (const auto& [id, val] : joint_values)
  {
    if (setJointValuesHelper(kdl_joints_cache, id, val))
      state.joints[id] = val;
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(
      state.link_transforms, state.joint_transforms, kdl_joints_cache, data_.tree.getRootSegment(), parent_frame);

  return state;
}

void KDLStateSolver::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                       const std::vector<tesseract::common::JointId>& joint_ids,
                                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                       const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  getLinkTransforms(link_transforms, joint_ids, joint_values);
}

void KDLStateSolver::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                       const std::vector<JointId>& joint_ids,
                                       const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  kdl_joints_cache = kdl_jnt_array_;

  for (auto i = 0U; i < joint_ids.size(); ++i)
    setJointValuesHelper(kdl_joints_cache, joint_ids[i], joint_values[i]);

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(link_transforms, kdl_joints_cache, data_.tree.getRootSegment(), parent_frame);
}

SceneState KDLStateSolver::getState(const std::vector<JointId>& joint_ids,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                    const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  SceneState state{ current_state_ };

  kdl_joints_cache = kdl_jnt_array_;

  for (auto i = 0U; i < joint_ids.size(); ++i)
  {
    if (setJointValuesHelper(kdl_joints_cache, joint_ids[i], joint_values[i]))
      state.joints[joint_ids[i]] = joint_values[i];
  }

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  // NOLINTNEXTLINE(clang-analyzer-core.UndefinedBinaryOperatorResult)
  calculateTransforms(
      state.link_transforms, state.joint_transforms, kdl_joints_cache, data_.tree.getRootSegment(), parent_frame);

  return state;
}

SceneState KDLStateSolver::getState(const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  throw std::runtime_error("KDLStateSolver, not supported!");
}

SceneState KDLStateSolver::getState() const { return current_state_; }

SceneState KDLStateSolver::getRandomState() const
{
  Eigen::VectorXd rs = tesseract::common::generateRandomNumber(limits_.joint_limits);
  return getState(data_.active_joint_ids, rs);  // NOLINT
}

Eigen::MatrixXd
KDLStateSolver::getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tesseract::common::LinkId& link_id,
                            const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  assert(joint_values.size() == data_.tree.getNrOfJoints());
  getKDLJntArray(kdl_joints_cache, data_.active_joint_ids, joint_values);
  if (calcJacobianHelper(kdl_jacobian_cache, kdl_joints_cache, link_id))
    return convert(kdl_jacobian_cache, joint_qnr_);

  throw std::runtime_error("KDLStateSolver: Failed to calculate jacobian.");
}

Eigen::MatrixXd
KDLStateSolver::getJacobian(const std::vector<JointId>& joint_ids,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tesseract::common::LinkId& link_id,
                            const tesseract::common::JointIdTransformMap& /*floating_joint_values*/) const
{
  getKDLJntArray(kdl_joints_cache, joint_ids, joint_values);
  if (calcJacobianHelper(kdl_jacobian_cache, kdl_joints_cache, link_id))
    return convert(kdl_jacobian_cache, joint_qnr_);

  throw std::runtime_error("KDLStateSolver: Failed to calculate jacobian.");
}

std::vector<JointId> KDLStateSolver::getJointIds() const { return data_.joint_ids; }

std::vector<JointId> KDLStateSolver::getFloatingJointIds() const { return data_.floating_joint_ids; }

std::vector<JointId> KDLStateSolver::getActiveJointIds() const { return data_.active_joint_ids; }

LinkId KDLStateSolver::getBaseLinkId() const { return data_.base_link_id; }

std::vector<LinkId> KDLStateSolver::getLinkIds() const { return data_.link_ids; }

std::vector<LinkId> KDLStateSolver::getActiveLinkIds() const { return data_.active_link_ids; }

std::vector<LinkId> KDLStateSolver::getStaticLinkIds() const { return data_.static_link_ids; }

bool KDLStateSolver::isActiveLinkId(const tesseract::common::LinkId& link_id) const
{
  return active_link_ids_set_.find(link_id) != active_link_ids_set_.end();
}

bool KDLStateSolver::hasLinkId(const tesseract::common::LinkId& link_id) const
{
  return link_ids_set_.find(link_id) != link_ids_set_.end();
}

tesseract::common::VectorIsometry3d KDLStateSolver::getLinkTransforms() const
{
  tesseract::common::VectorIsometry3d link_tfs;
  link_tfs.reserve(current_state_.link_transforms.size());
  for (const auto& link_id : data_.link_ids)
    link_tfs.push_back(current_state_.link_transforms.at(link_id));

  return link_tfs;
}

Eigen::Isometry3d KDLStateSolver::getLinkTransform(const tesseract::common::LinkId& link_id) const
{
  return current_state_.link_transforms.at(link_id);
}

Eigen::Isometry3d KDLStateSolver::getRelativeLinkTransform(const tesseract::common::LinkId& from_link_id,
                                                           const tesseract::common::LinkId& to_link_id) const
{
  return current_state_.link_transforms.at(from_link_id).inverse() * current_state_.link_transforms.at(to_link_id);
}

tesseract::common::KinematicLimits KDLStateSolver::getLimits() const { return limits_; }

bool KDLStateSolver::processKDLData(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  current_state_ = SceneState();
  current_state_.floating_joints = data_.floating_joint_values;
  kdl_jnt_array_.resize(data_.tree.getNrOfJoints());
  limits_.joint_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()), 2);
  limits_.velocity_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()), 2);
  limits_.acceleration_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()), 2);
  limits_.jerk_limits.resize(static_cast<long int>(data_.tree.getNrOfJoints()), 2);
  joint_qnr_.resize(data_.tree.getNrOfJoints());
  joint_to_qnr_.clear();
  size_t j = 0;
  for (const auto& seg : data_.tree.getSegments())
  {
    const KDL::Joint& jnt = seg.second.segment.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    // Compute the JointId once and reuse it for all four sites below; the JointId
    // ctor performs a name->id lookup so reusing avoids redundant map lookups.
    const JointId jid(jnt.getName());

    joint_to_qnr_.insert(std::make_pair(jid, seg.second.q_nr));
    kdl_jnt_array_(seg.second.q_nr) = 0.0;
    current_state_.joints.insert(std::make_pair(jid, 0.0));
    data_.active_joint_ids[j] = jid;
    joint_qnr_[j] = static_cast<int>(seg.second.q_nr);

    // Store joint limits.
    const auto& sj = scene_graph.getJoint(jid);
    limits_.joint_limits(static_cast<long>(j), 0) = sj->limits->lower;
    limits_.joint_limits(static_cast<long>(j), 1) = sj->limits->upper;
    limits_.velocity_limits(static_cast<long>(j), 0) = -sj->limits->velocity;
    limits_.velocity_limits(static_cast<long>(j), 1) = sj->limits->velocity;
    limits_.acceleration_limits(static_cast<long>(j), 0) = -sj->limits->acceleration;
    limits_.acceleration_limits(static_cast<long>(j), 1) = sj->limits->acceleration;
    limits_.jerk_limits(static_cast<long>(j), 0) = -sj->limits->jerk;
    limits_.jerk_limits(static_cast<long>(j), 1) = sj->limits->jerk;

    j++;
  }

  jac_solver_ = std::make_unique<KDL::TreeJntToJacSolver>(data_.tree);

  // Cache LinkId/JointId per segment to avoid per-FK constructor calls.
  // Keyed by pointer to KDL TreeElement (pointer-stable in std::map).
  segment_id_cache_.clear();
  for (const auto& seg : data_.tree.getSegments())
  {
    SegmentIdCache entry;
    entry.link_id = LinkId(seg.first);
    entry.joint_id = JointId(seg.second.segment.getJoint().getName());
    segment_id_cache_[&seg.second] = entry;
  }
  root_element_ = &data_.tree.getRootSegment()->second;

  calculateTransforms(current_state_.link_transforms,
                      current_state_.joint_transforms,
                      kdl_jnt_array_,
                      data_.tree.getRootSegment(),
                      Eigen::Isometry3d::Identity());

  // Build O(1) mirrors of the active/all link id vectors so isActiveLinkId/hasLinkId
  // can avoid the linear std::find that previously dominated lookups in hot paths.
  active_link_ids_set_.clear();
  active_link_ids_set_.insert(data_.active_link_ids.begin(), data_.active_link_ids.end());
  link_ids_set_.clear();
  link_ids_set_.insert(data_.link_ids.begin(), data_.link_ids.end());

  return true;
}

bool KDLStateSolver::setJointValuesHelper(KDL::JntArray& q,
                                          const tesseract::common::JointId& joint_id,
                                          const double& joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_id);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    return true;
  }

  CONSOLE_BRIDGE_logError("Tried to set joint name %s which does not exist!", joint_id.name().c_str());
  return false;
}

void KDLStateSolver::calculateTransformsHelper(tesseract::common::LinkIdTransformMap& link_transforms,
                                               tesseract::common::JointIdTransformMap& joint_transforms,
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
    Eigen::Isometry3d global_frame{ parent_frame * local_frame };
    const auto& cached = segment_id_cache_.at(&current_element);
    link_transforms[cached.link_id] = global_frame;
    if (&current_element != root_element_)
      joint_transforms[cached.joint_id] = global_frame;

    for (const auto& child : current_element.children)
    {
      calculateTransformsHelper(link_transforms, joint_transforms, q_in, child, global_frame);  // NOLINT
    }
  }
}

void KDLStateSolver::calculateTransformsHelper(tesseract::common::LinkIdTransformMap& link_transforms,
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
    Eigen::Isometry3d global_frame{ parent_frame * local_frame };
    const auto& cached = segment_id_cache_.at(&current_element);
    link_transforms[cached.link_id] = global_frame;
    for (const auto& child : current_element.children)
    {
      calculateTransformsHelper(link_transforms, q_in, child, global_frame);  // NOLINT
    }
  }
}

void KDLStateSolver::calculateTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                         tesseract::common::JointIdTransformMap& joint_transforms,
                                         const KDL::JntArray& q_in,
                                         const KDL::SegmentMap::const_iterator& it,
                                         const Eigen::Isometry3d& parent_frame) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  calculateTransformsHelper(link_transforms, joint_transforms, q_in, it, parent_frame);  // NOLINT
}

void KDLStateSolver::calculateTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                         const KDL::JntArray& q_in,
                                         const KDL::SegmentMap::const_iterator& it,
                                         const Eigen::Isometry3d& parent_frame) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  calculateTransformsHelper(link_transforms, q_in, it, parent_frame);  // NOLINT
}

bool KDLStateSolver::calcJacobianHelper(KDL::Jacobian& jacobian,
                                        const KDL::JntArray& kdl_joints,
                                        const tesseract::common::LinkId& link_id) const
{
  jacobian.resize(static_cast<unsigned>(kdl_joints.data.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, link_id.name()) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate jacobian");
    return false;
  }

  return true;
}

void KDLStateSolver::getKDLJntArray(KDL::JntArray& kdl_joints,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  kdl_joints = kdl_jnt_array_;
  for (unsigned i = 0; i < joint_ids.size(); ++i)
    kdl_joints.data(joint_to_qnr_.at(joint_ids[i])) = joint_values[i];
}

// LCOV_EXCL_STOP
}  // namespace tesseract::scene_graph
