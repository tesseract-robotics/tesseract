/**
 * @file ofkt_state_solver.cpp
 * @brief A implementation of the Optimized Forward Kinematic Tree as a state solver.
 *
 * This is based on the paper "A Forward Kinematics Data Structure for Efficient Evolutionary Inverse Kinematics".
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 * @author Levi Armstrong
 * @date August 24, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <boost/graph/depth_first_search.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_node.h>
#include <tesseract/state_solver/ofkt/ofkt_nodes.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/common/utils.h>

#include <mutex>

namespace tesseract::scene_graph
{
using tesseract::common::JointId;
using tesseract::common::LinkId;

/** @brief Every time a vertex is visited for the first time add a new node to the tree */
struct ofkt_builder : public boost::dfs_visitor<>
{
  ofkt_builder(OFKTStateSolver& tree, std::vector<JointLimits::ConstPtr>& new_joints_limits, std::string prefix = "")
    : tree_(tree), new_joints_limits_(new_joints_limits), prefix_(std::move(prefix))
  {
  }

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    // Get incoming edges
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));
    if (num_in_edges == 0)  // The root of the tree will have not incoming edges
      return;

    boost::graph_traits<tesseract::scene_graph::Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    tesseract::scene_graph::SceneGraph::Edge e = *ei;
    const tesseract::scene_graph::Joint::ConstPtr& joint = boost::get(boost::edge_joint, graph)[e];
    std::string joint_name = prefix_ + joint->getName();
    std::string parent_link_name = prefix_ + joint->parent_link_name;
    std::string child_link_name = prefix_ + joint->child_link_name;

    tree_.addNode(*joint, joint_name, parent_link_name, child_link_name, new_joints_limits_);
  }

protected:
  OFKTStateSolver& tree_;                                  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  std::vector<JointLimits::ConstPtr>& new_joints_limits_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  std::string prefix_;
};

void OFKTStateSolver::cloneHelper(OFKTStateSolver& cloned, const OFKTNode* node) const
{
  OFKTNode* parent_node = cloned.link_map_[node->getLinkId()];
  for (const OFKTNode* child : node->getChildren())
  {
    if (child->getType() == tesseract::scene_graph::JointType::FIXED)
    {
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, child->getLinkName(), child->getJointName(), child->getStaticTransformation());
      cloned.link_map_[child->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[child->getJointId()] = std::move(n);
    }
    else if (child->getType() == tesseract::scene_graph::JointType::FLOATING)
    {
      auto n = std::make_unique<OFKTFloatingNode>(
          parent_node, child->getLinkName(), child->getJointName(), child->getStaticTransformation());
      cloned.link_map_[child->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[child->getJointId()] = std::move(n);
    }
    else if (child->getType() == tesseract::scene_graph::JointType::REVOLUTE)
    {
      const auto* cn = static_cast<const OFKTRevoluteNode*>(child);

      auto n = std::make_unique<OFKTRevoluteNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointId()] = std::move(n);
    }
    else if (child->getType() == tesseract::scene_graph::JointType::CONTINUOUS)
    {
      const auto* cn = static_cast<const OFKTContinuousNode*>(child);

      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointId()] = std::move(n);
    }
    else if (child->getType() == tesseract::scene_graph::JointType::PRISMATIC)
    {
      const auto* cn = static_cast<const OFKTPrismaticNode*>(child);

      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointId()] = std::move(n);
    }
    else
    {
      throw std::runtime_error("Unsupported OFKTNode type!");  // LCOV_EXCL_LINE
    }

    cloneHelper(cloned, child);
  }
}

OFKTStateSolver::OFKTStateSolver(const tesseract::scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  initHelper(scene_graph, prefix);
}

OFKTStateSolver::OFKTStateSolver(const std::string& root_name)
{
  root_ = std::make_unique<OFKTRootNode>(root_name);
  link_map_[root_->getLinkId()] = root_.get();
  link_ids_ = { root_->getLinkId() };
  current_state_.link_transforms[root_->getLinkId()] = root_->getWorldTransformation();
}

OFKTStateSolver::OFKTStateSolver(const OFKTStateSolver& other) { *this = other; }

OFKTStateSolver& OFKTStateSolver::operator=(const OFKTStateSolver& other)
{
  if (this == &other)
    return *this;

  current_state_ = other.current_state_;
  joint_ids_ = other.joint_ids_;
  active_joint_ids_ = other.active_joint_ids_;
  floating_joint_ids_ = other.floating_joint_ids_;
  link_ids_ = other.link_ids_;
  root_ = std::make_unique<OFKTRootNode>(other.root_->getLinkName());
  link_map_[other.root_->getLinkId()] = root_.get();
  limits_ = other.limits_;
  revision_ = other.revision_;
  cloneHelper(*this, other.root_.get());
  return *this;
}

StateSolver::UPtr OFKTStateSolver::clone() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::make_unique<OFKTStateSolver>(*this);
}

void OFKTStateSolver::setRevision(int revision)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  revision_ = revision;
}

int OFKTStateSolver::getRevision() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return revision_;
}

void OFKTStateSolver::clear()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);

  current_state_ = SceneState();
  joint_ids_.clear();
  active_joint_ids_.clear();
  floating_joint_ids_.clear();
  link_ids_.clear();
  nodes_.clear();
  link_map_.clear();
  limits_ = tesseract::common::KinematicLimits();
  root_ = nullptr;
}

void OFKTStateSolver::setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  assert(active_joint_ids_.size() == static_cast<std::size_t>(joint_values.size()));
  for (std::size_t i = 0; i < active_joint_ids_.size(); ++i)
  {
    nodes_[active_joint_ids_[i]]->storeJointValue(joint_values(static_cast<long>(i)));
    current_state_.joints[active_joint_ids_[i]] = joint_values(static_cast<long>(i));
  }

  for (const auto& [joint_id, transform] : floating_joint_values)
  {
    current_state_.floating_joints.at(joint_id) = transform;
    for (const auto& fj_id : floating_joint_ids_)
    {
      if (fj_id == joint_id)
      {
        nodes_[fj_id]->setStaticTransformation(transform);
        break;
      }
    }
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::unordered_map<std::string, double>& joint_values,
                               const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  SceneState::JointValues id_values;
  id_values.reserve(joint_values.size());
  for (const auto& [name, val] : joint_values)
    id_values[JointId::fromName(name)] = val;
  setState(id_values, floating_joint_values);
}

void OFKTStateSolver::setState(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::vector<JointId> ids;
  ids.reserve(joint_names.size());
  for (const auto& name : joint_names)
    ids.push_back(JointId::fromName(name));
  setState(ids, joint_values, floating_joint_values);
}

void OFKTStateSolver::setState(const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  for (const auto& [joint_id, transform] : floating_joint_values)
  {
    current_state_.floating_joints.at(joint_id) = transform;
    for (const auto& fj_id : floating_joint_ids_)
    {
      if (fj_id == joint_id)
      {
        nodes_[fj_id]->setStaticTransformation(transform);
        break;
      }
    }
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const SceneState::JointValues& joint_values,
                               const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  for (const auto& [joint_id, value] : joint_values)
  {
    nodes_[joint_id]->storeJointValue(value);
    current_state_.joints[joint_id] = value;
  }

  for (const auto& [joint_id, transform] : floating_joint_values)
  {
    current_state_.floating_joints.at(joint_id) = transform;
    for (const auto& fj_id : floating_joint_ids_)
    {
      if (fj_id == joint_id)
      {
        nodes_[fj_id]->setStaticTransformation(transform);
        break;
      }
    }
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::vector<tesseract::common::JointId>& joint_ids,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  assert(joint_ids.size() == static_cast<std::size_t>(joint_values.size()));
  for (std::size_t i = 0; i < joint_ids.size(); ++i)
  {
    nodes_[joint_ids[i]]->storeJointValue(joint_values(static_cast<long>(i)));
    current_state_.joints[joint_ids[i]] = joint_values(static_cast<long>(i));
  }

  for (const auto& [joint_id, transform] : floating_joint_values)
  {
    current_state_.floating_joints.at(joint_id) = transform;
    for (const auto& fj_id : floating_joint_ids_)
    {
      if (fj_id == joint_id)
      {
        nodes_[fj_id]->setStaticTransformation(transform);
        break;
      }
    }
  }

  update(root_.get(), false);
}

SceneState OFKTStateSolver::getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  auto state = SceneState(current_state_);
  for (std::size_t i = 0; i < active_joint_ids_.size(); ++i)
    state.joints[active_joint_ids_[i]] = joint_values[static_cast<long>(i)];

  for (const auto& [joint_id, transform] : floating_joint_values)
    state.floating_joints.at(joint_id) = transform;

  update(state, root_.get(), parent_frame, false);
  return state;
}

SceneState OFKTStateSolver::getState(const std::unordered_map<std::string, double>& joint_values,
                                     const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  SceneState::JointValues id_values;
  id_values.reserve(joint_values.size());
  for (const auto& [name, val] : joint_values)
    id_values[JointId::fromName(name)] = val;
  return getState(id_values, floating_joint_values);
}

SceneState OFKTStateSolver::getState(const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  assert(static_cast<Eigen::Index>(joint_names.size()) == joint_values.size());
  std::vector<JointId> ids;
  ids.reserve(joint_names.size());
  for (const auto& name : joint_names)
    ids.push_back(JointId::fromName(name));
  return getState(ids, joint_values, floating_joint_values);
}

SceneState OFKTStateSolver::getState(const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  auto state = SceneState(current_state_);
  for (const auto& [joint_id, transform] : floating_joint_values)
    state.floating_joints.at(joint_id) = transform;

  update(state, root_.get(), parent_frame, false);
  return state;
}

SceneState OFKTStateSolver::getStateUnlocked(const SceneState::JointValues& joint_values,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  auto state = SceneState(current_state_);
  for (const auto& [joint_id, value] : joint_values)
    state.joints[joint_id] = value;

  for (const auto& [joint_id, transform] : floating_joint_values)
    state.floating_joints.at(joint_id) = transform;

  update(state, root_.get(), parent_frame, false);
  return state;
}

SceneState OFKTStateSolver::getState(const SceneState::JointValues& joint_values,
                                     const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return getStateUnlocked(joint_values, floating_joint_values);
}

SceneState OFKTStateSolver::getState(const std::vector<tesseract::common::JointId>& joint_ids,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  assert(joint_ids.size() == static_cast<std::size_t>(joint_values.size()));
  SceneState::JointValues id_values;
  id_values.reserve(joint_ids.size());
  for (std::size_t i = 0; i < joint_ids.size(); ++i)
    id_values[joint_ids[i]] = joint_values[static_cast<long>(i)];
  return getState(id_values, floating_joint_values);
}

SceneState OFKTStateSolver::getState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_;
}

void OFKTStateSolver::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                        const std::vector<std::string>& joint_names,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  SceneState::JointValues joints{ current_state_.joints };
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    joints[JointId::fromName(joint_names[i])] = joint_values[static_cast<long>(i)];

  link_transforms = current_state_.link_transforms;
  update(link_transforms, joints, current_state_.floating_joints, root_.get(), parent_frame, false);
}

void OFKTStateSolver::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                        const std::vector<JointId>& joint_ids,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  static const Eigen::Isometry3d parent_frame{ Eigen::Isometry3d::Identity() };

  SceneState::JointValues joints{ current_state_.joints };
  for (std::size_t i = 0; i < joint_ids.size(); ++i)
    joints[joint_ids[i]] = joint_values[static_cast<long>(i)];

  link_transforms = current_state_.link_transforms;
  update(link_transforms, joints, current_state_.floating_joints, root_.get(), parent_frame, false);
}

SceneState OFKTStateSolver::getRandomState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const Eigen::VectorXd random_values = tesseract::common::generateRandomNumber(limits_.joint_limits);
  SceneState::JointValues id_values;
  id_values.reserve(active_joint_ids_.size());
  for (std::size_t i = 0; i < active_joint_ids_.size(); ++i)
    id_values[active_joint_ids_[i]] = random_values[static_cast<long>(i)];
  return getStateUnlocked(id_values, {});
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const std::string& link_name,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  SceneState::JointValues joints = current_state_.joints;
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joints[active_joint_ids_[static_cast<std::size_t>(i)]] = joint_values[i];

  tesseract::common::JointIdTransformMap floating_joints{ current_state_.floating_joints };
  for (const auto& [joint_id, transform] : floating_joint_values)
    floating_joints.at(joint_id) = transform;

  return calcJacobianHelper(joints, LinkId::fromName(link_name), floating_joints);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const std::unordered_map<std::string, double>& joints_values,
                                             const std::string& link_name,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  SceneState::JointValues joints = current_state_.joints;
  for (const auto& joint : joints_values)
    joints[JointId::fromName(joint.first)] = joint.second;

  tesseract::common::JointIdTransformMap floating_joints{ current_state_.floating_joints };
  for (const auto& [joint_id, transform] : floating_joint_values)
    floating_joints.at(joint_id) = transform;

  return calcJacobianHelper(joints, LinkId::fromName(link_name), floating_joints);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const std::string& link_name,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  SceneState::JointValues joints = current_state_.joints;
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joints[JointId::fromName(joint_names[static_cast<std::size_t>(i)])] = joint_values[i];

  tesseract::common::JointIdTransformMap floating_joints{ current_state_.floating_joints };
  for (const auto& [joint_id, transform] : floating_joint_values)
    floating_joints.at(joint_id) = transform;

  return calcJacobianHelper(joints, LinkId::fromName(link_name), floating_joints);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const tesseract::common::LinkId& link_id,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  SceneState::JointValues joints = current_state_.joints;
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joints[active_joint_ids_[static_cast<std::size_t>(i)]] = joint_values[i];

  tesseract::common::JointIdTransformMap floating_joints{ current_state_.floating_joints };
  for (const auto& [joint_id, transform] : floating_joint_values)
    floating_joints.at(joint_id) = transform;

  return calcJacobianHelper(joints, link_id, floating_joints);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const std::vector<JointId>& joint_ids,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const tesseract::common::LinkId& link_id,
                                             const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  SceneState::JointValues joints = current_state_.joints;
  for (std::size_t i = 0; i < joint_ids.size(); ++i)
    joints[joint_ids[i]] = joint_values[static_cast<Eigen::Index>(i)];

  tesseract::common::JointIdTransformMap floating_joints{ current_state_.floating_joints };
  for (const auto& [joint_id, transform] : floating_joint_values)
    floating_joints.at(joint_id) = transform;

  return calcJacobianHelper(joints, link_id, floating_joints);
}

Eigen::MatrixXd OFKTStateSolver::calcJacobianHelper(const SceneState::JointValues& joints,
                                                    const tesseract::common::LinkId& link_id,
                                                    const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  OFKTNode* node = link_map_.at(link_id);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(active_joint_ids_.size()));

  Eigen::Isometry3d total_tf{ Eigen::Isometry3d::Identity() };
  while (node != root_.get())
  {
    if (node->getType() == JointType::FIXED)
    {
      total_tf = node->getLocalTransformation() * total_tf;
    }
    else if (node->getType() == JointType::FLOATING)
    {
      total_tf = floating_joint_values.at(node->getJointId()) * total_tf;
    }
    else
    {
      Eigen::Isometry3d local_tf = node->computeLocalTransformation(joints.at(node->getJointId()));
      total_tf = local_tf * total_tf;

      Eigen::Index idx =
          std::distance(active_joint_ids_.begin(),
                        std::find(active_joint_ids_.begin(), active_joint_ids_.end(), node->getJointId()));
      Eigen::VectorXd twist = node->getLocalTwist();
      tesseract::common::twistChangeRefPoint(twist, total_tf.translation() - local_tf.translation());
      tesseract::common::twistChangeBase(twist, total_tf.inverse());
      jacobian.col(idx) = twist;
    }

    node = node->getParent();
  }

  tesseract::common::jacobianChangeBase(jacobian, total_tf);
  return jacobian;
}

std::vector<std::string> OFKTStateSolver::getJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(joint_ids_.size());
  for (const auto& id : joint_ids_)
    names.push_back(id.name());
  return names;
}

std::vector<std::string> OFKTStateSolver::getFloatingJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(floating_joint_ids_.size());
  for (const auto& id : floating_joint_ids_)
    names.push_back(id.name());
  return names;
}

std::vector<std::string> OFKTStateSolver::getActiveJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(active_joint_ids_.size());
  for (const auto& id : active_joint_ids_)
    names.push_back(id.name());
  return names;
}

std::string OFKTStateSolver::getBaseLinkName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return root_->getLinkName();
}

std::vector<std::string> OFKTStateSolver::getLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(link_ids_.size());
  for (const auto& id : link_ids_)
    names.push_back(id.name());
  return names;
}

std::vector<std::string> OFKTStateSolver::getActiveLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> active_link_ids;
  active_link_ids.reserve(nodes_.size());
  loadActiveLinkIdsRecursive(active_link_ids, root_.get(), false);
  std::vector<std::string> names;
  names.reserve(active_link_ids.size());
  for (const auto& id : active_link_ids)
    names.push_back(id.name());
  return names;
}

std::vector<std::string> OFKTStateSolver::getStaticLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> static_link_ids;
  static_link_ids.reserve(nodes_.size());
  loadStaticLinkIdsRecursive(static_link_ids, root_.get());
  std::vector<std::string> names;
  names.reserve(static_link_ids.size());
  for (const auto& id : static_link_ids)
    names.push_back(id.name());
  return names;
}

bool OFKTStateSolver::isActiveLinkName(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> active_link_ids;
  active_link_ids.reserve(nodes_.size());
  loadActiveLinkIdsRecursive(active_link_ids, root_.get(), false);
  const auto target = LinkId::fromName(link_name);
  return std::find(active_link_ids.begin(), active_link_ids.end(), target) != active_link_ids.end();
}

bool OFKTStateSolver::hasLinkName(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return link_map_.count(LinkId::fromName(link_name)) > 0;
}

tesseract::common::VectorIsometry3d OFKTStateSolver::getLinkTransforms() const
{
  tesseract::common::VectorIsometry3d link_tfs;
  link_tfs.reserve(current_state_.link_transforms.size());
  for (const auto& link_id : link_ids_)
    link_tfs.push_back(current_state_.link_transforms.at(link_id));

  return link_tfs;
}

Eigen::Isometry3d OFKTStateSolver::getLinkTransform(const std::string& link_name) const
{
  return current_state_.link_transforms.at(LinkId::fromName(link_name));
}

Eigen::Isometry3d OFKTStateSolver::getRelativeLinkTransform(const std::string& from_link_name,
                                                            const std::string& to_link_name) const
{
  return current_state_.link_transforms.at(LinkId::fromName(from_link_name)).inverse() *
         current_state_.link_transforms.at(LinkId::fromName(to_link_name));
}

// --- ID-based overloads ---

std::vector<JointId> OFKTStateSolver::getJointIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return joint_ids_;
}

std::vector<JointId> OFKTStateSolver::getFloatingJointIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return floating_joint_ids_;
}

std::vector<JointId> OFKTStateSolver::getActiveJointIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return active_joint_ids_;
}

LinkId OFKTStateSolver::getBaseLinkId() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return root_->getLinkId();
}

std::vector<LinkId> OFKTStateSolver::getLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return link_ids_;
}

std::vector<LinkId> OFKTStateSolver::getActiveLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> ids;
  ids.reserve(nodes_.size());
  loadActiveLinkIdsRecursive(ids, root_.get(), false);
  return ids;
}

std::vector<LinkId> OFKTStateSolver::getStaticLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> ids;
  ids.reserve(nodes_.size());
  loadStaticLinkIdsRecursive(ids, root_.get());
  return ids;
}

bool OFKTStateSolver::isActiveLinkId(const tesseract::common::LinkId& link_id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<LinkId> ids;
  ids.reserve(nodes_.size());
  loadActiveLinkIdsRecursive(ids, root_.get(), false);
  return std::find(ids.begin(), ids.end(), link_id) != ids.end();
}

bool OFKTStateSolver::hasLinkId(const tesseract::common::LinkId& link_id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return link_map_.count(link_id) > 0;
}

Eigen::Isometry3d OFKTStateSolver::getLinkTransform(const tesseract::common::LinkId& link_id) const
{
  return current_state_.link_transforms.at(link_id);
}

Eigen::Isometry3d OFKTStateSolver::getRelativeLinkTransform(const tesseract::common::LinkId& from_link_id,
                                                            const tesseract::common::LinkId& to_link_id) const
{
  return current_state_.link_transforms.at(from_link_id).inverse() *
         current_state_.link_transforms.at(to_link_id);
}

tesseract::common::KinematicLimits OFKTStateSolver::getLimits() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return limits_;
}

bool OFKTStateSolver::addLink(const Link& link, const Joint& joint)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (link_map_.find(link.getId()) != link_map_.end())
  {
    return false;
  }

  if (nodes_.find(joint.getId()) != nodes_.end())
  {
    return false;
  }

  std::vector<JointLimits::ConstPtr> new_joint_limits;
  addNode(joint, joint.getName(), joint.parent_link_name, joint.child_link_name, new_joint_limits);
  addNewJointLimits(new_joint_limits);

  update(root_.get(), false);

  return true;
}

/**
 * @brief Replace and existing joint with the provided one
 * @param joint The replacement joint
 * @return Return False if a joint does not exists, otherwise true
 */
bool OFKTStateSolver::replaceJoint(const Joint& joint)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint.getId());
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to replace joint '%s' which does not exist!",
                            joint.getName().c_str());
    return false;
  }

  if (link_map_.find(LinkId::fromName(joint.parent_link_name)) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to replace joint '%s' with parent link name that does not exist!",
                            joint.getName().c_str());
    return false;
  }

  if (it->second->getLinkName() != joint.child_link_name)
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to replace joint '%s' with different child link name!",
                            joint.getName().c_str());
    return false;
  }

  std::vector<JointLimits::ConstPtr> new_joint_limits;
  replaceJointHelper(new_joint_limits, joint);
  addNewJointLimits(new_joint_limits);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::moveLink(const Joint& joint)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);

  if (link_map_.find(LinkId::fromName(joint.child_link_name)) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to link '%s' that does not exist!", joint.child_link_name.c_str());
    return false;
  }

  if (link_map_.find(LinkId::fromName(joint.parent_link_name)) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to move link to parent link '%s' that does not exist!",
                            joint.parent_link_name.c_str());
    return false;
  }

  std::vector<JointLimits::ConstPtr> new_joint_limits;
  moveLinkHelper(new_joint_limits, joint);
  addNewJointLimits(new_joint_limits);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::removeLink(const std::string& name) { return removeLink(LinkId::fromName(name)); }

bool OFKTStateSolver::removeLink(const LinkId& link_id)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = link_map_.find(link_id);
  if (it == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to remove link '%s' which does not exist!", link_id.name().c_str());
    return false;
  }

  std::vector<LinkId> removed_links;
  removed_links.reserve(nodes_.size());

  std::vector<JointId> removed_joints;
  removed_joints.reserve(nodes_.size());

  std::vector<JointId> removed_active_joints;
  removed_active_joints.reserve(nodes_.size());

  std::vector<long> removed_active_joints_indices;
  removed_active_joints_indices.reserve(nodes_.size());

  removeNode(it->second, removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  // Remove deleted joints
  removeJointHelper(removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::removeJoint(const std::string& name) { return removeJoint(JointId::fromName(name)); }

bool OFKTStateSolver::removeJoint(const JointId& joint_id)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to remove joint '%s' which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  std::vector<LinkId> removed_links;
  removed_links.reserve(nodes_.size());

  std::vector<JointId> removed_joints;
  removed_joints.reserve(nodes_.size());

  std::vector<JointId> removed_active_joints;
  removed_active_joints.reserve(nodes_.size());

  std::vector<long> removed_active_joints_indices;
  removed_active_joints_indices.reserve(nodes_.size());

  removeNode(it->second.get(), removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  // Remove deleted joints
  removeJointHelper(removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::moveJoint(const std::string& name, const std::string& parent_link)
{
  return moveJoint(JointId::fromName(name), LinkId::fromName(parent_link));
}

bool OFKTStateSolver::moveJoint(const JointId& joint_id, const LinkId& parent_link_id)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to move joint '%s' which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  if (link_map_.find(parent_link_id) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to move joint '%s' to parent link '%s' which does not exist!",
                            joint_id.name().c_str(),
                            parent_link_id.name().c_str());
    return false;
  }

  auto& n = it->second;
  n->getParent()->removeChild(n.get());
  OFKTNode* new_parent = link_map_[parent_link_id];
  n->setParent(new_parent);
  new_parent->addChild(n.get());

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin)
{
  return changeJointOrigin(JointId::fromName(name), new_origin);
}

bool OFKTStateSolver::changeJointOrigin(const JointId& joint_id, const Eigen::Isometry3d& new_origin)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' origin which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  it->second->setStaticTransformation(new_origin);
  if (it->second->getType() == JointType::FLOATING)
    current_state_.floating_joints[joint_id] = new_origin;

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::changeJointPositionLimits(const std::string& name, double lower, double upper)
{
  return changeJointPositionLimits(JointId::fromName(name), lower, upper);
}

bool OFKTStateSolver::changeJointPositionLimits(const JointId& joint_id, double lower, double upper)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' positioner limits which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  long idx = std::distance(active_joint_ids_.begin(),
                           std::find(active_joint_ids_.begin(), active_joint_ids_.end(), joint_id));
  limits_.joint_limits(idx, 0) = lower;
  limits_.joint_limits(idx, 1) = upper;
  return true;
}

bool OFKTStateSolver::changeJointVelocityLimits(const std::string& name, double limit)
{
  return changeJointVelocityLimits(JointId::fromName(name), limit);
}

bool OFKTStateSolver::changeJointVelocityLimits(const JointId& joint_id, double limit)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' positioner limits which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  long idx = std::distance(active_joint_ids_.begin(),
                           std::find(active_joint_ids_.begin(), active_joint_ids_.end(), joint_id));
  limits_.velocity_limits(idx, 0) = -limit;
  limits_.velocity_limits(idx, 1) = limit;
  return true;
}

bool OFKTStateSolver::changeJointAccelerationLimits(const std::string& name, double limit)
{
  return changeJointAccelerationLimits(JointId::fromName(name), limit);
}

bool OFKTStateSolver::changeJointAccelerationLimits(const JointId& joint_id, double limit)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' positioner limits which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  long idx = std::distance(active_joint_ids_.begin(),
                           std::find(active_joint_ids_.begin(), active_joint_ids_.end(), joint_id));
  limits_.acceleration_limits(idx, 0) = -limit;
  limits_.acceleration_limits(idx, 1) = limit;
  return true;
}

bool OFKTStateSolver::changeJointJerkLimits(const std::string& name, double limit)
{
  return changeJointJerkLimits(JointId::fromName(name), limit);
}

bool OFKTStateSolver::changeJointJerkLimits(const JointId& joint_id, double limit)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(joint_id);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' positioner limits which does not exist!",
                            joint_id.name().c_str());
    return false;
  }

  long idx = std::distance(active_joint_ids_.begin(),
                           std::find(active_joint_ids_.begin(), active_joint_ids_.end(), joint_id));
  limits_.jerk_limits(idx, 0) = -limit;
  limits_.jerk_limits(idx, 1) = limit;
  return true;
}

bool OFKTStateSolver::insertSceneGraph(const SceneGraph& scene_graph, const Joint& joint, const std::string& prefix)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (root_ == nullptr)
    return false;  // LCOV_EXCL_LINE

  std::string parent_link = joint.parent_link_name;
  std::string child_link = joint.child_link_name;

  // Assumes the joint already contains the prefix in the parent and child link names
  if (!prefix.empty())
    child_link.erase(0, prefix.length());

  if (link_map_.find(LinkId::fromName(parent_link)) == link_map_.end() || scene_graph.getLink(child_link) == nullptr)
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, Failed to add inserted graph, provided joint link names do not exist in "
                            "inserted graph!");
    return false;
  }

  if (nodes_.find(joint.getId()) != nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, Failed to add inserted graph, provided joint name %s already exists!",
                            joint.getName().c_str());
    return false;
  }

  std::vector<JointLimits::ConstPtr> new_joints_limits;
  new_joints_limits.reserve(boost::num_edges(scene_graph));

  addNode(joint, joint.getName(), joint.parent_link_name, joint.child_link_name, new_joints_limits);

  ofkt_builder builder(*this, new_joints_limits, prefix);

  std::map<tesseract::scene_graph::SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<tesseract::scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
      index_map);

  int c = 0;
  tesseract::scene_graph::Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(static_cast<const tesseract::scene_graph::Graph&>(scene_graph),
                            boost::visitor(builder)
                                .root_vertex(scene_graph.getVertex(scene_graph.getRoot()))
                                .vertex_index_map(prop_index_map));

  // Populate Joint Limits
  addNewJointLimits(new_joints_limits);

  update(root_.get(), false);
  return true;
}

void OFKTStateSolver::loadActiveLinkIdsRecursive(std::vector<LinkId>& active_link_ids,
                                                 const OFKTNode* node,
                                                 bool active) const
{
  if (active)
  {
    active_link_ids.push_back(node->getLinkId());
    for (const auto* child : node->getChildren())
      loadActiveLinkIdsRecursive(active_link_ids, child, active);
  }
  else
  {
    if (node->getType() == tesseract::scene_graph::JointType::FIXED ||
        node->getType() == tesseract::scene_graph::JointType::FLOATING)
    {
      for (const auto* child : node->getChildren())
        loadActiveLinkIdsRecursive(active_link_ids, child, active);
    }
    else
    {
      active_link_ids.push_back(node->getLinkId());
      for (const auto* child : node->getChildren())
        loadActiveLinkIdsRecursive(active_link_ids, child, true);
    }
  }
}

void OFKTStateSolver::loadStaticLinkIdsRecursive(std::vector<LinkId>& static_link_ids,
                                                 const OFKTNode* node) const
{
  if (node->getType() == tesseract::scene_graph::JointType::FIXED ||
      node->getType() == tesseract::scene_graph::JointType::FLOATING)
  {
    static_link_ids.push_back(node->getLinkId());
    for (const auto* child : node->getChildren())
      loadStaticLinkIdsRecursive(static_link_ids, child);
  }
}

void OFKTStateSolver::update(OFKTNode* node, bool update_required)
{
  if (node->hasJointValueChanged())
  {
    node->computeAndStoreLocalTransformation();
    update_required = true;
  }

  if (node->updateWorldTransformationRequired())
    update_required = true;

  if (update_required)
  {
    node->computeAndStoreWorldTransformation();
    current_state_.link_transforms[node->getLinkId()] = node->getWorldTransformation();
    current_state_.joint_transforms[node->getJointId()] = node->getWorldTransformation();
  }

  for (auto* child : node->getChildren())
    update(child, update_required);
}

bool OFKTStateSolver::updateRequired(Eigen::Isometry3d& updated_parent_world_tf,
                                     const SceneState::JointValues& joints,
                                     const tesseract::common::JointIdTransformMap& floating_joints,
                                     const OFKTNode* node,
                                     const Eigen::Isometry3d& parent_world_tf)
{
  if (node->getType() == tesseract::scene_graph::JointType::FIXED)
  {
    updated_parent_world_tf = parent_world_tf * node->getLocalTransformation();
    return false;
  }

  if (node->getType() == tesseract::scene_graph::JointType::FLOATING)
  {
    const auto& tf = floating_joints.at(node->getJointId());
    updated_parent_world_tf = parent_world_tf * tf;
    return (!tf.isApprox(node->getLocalTransformation(), 1e-8));
  }

  double jv = joints.at(node->getJointId());
  if (!tesseract::common::almostEqualRelativeAndAbs(node->getJointValue(), jv, 1e-8))
  {
    updated_parent_world_tf = parent_world_tf * node->computeLocalTransformation(jv);
    return true;
  }

  updated_parent_world_tf = parent_world_tf * node->getLocalTransformation();
  return false;
}

void OFKTStateSolver::update(tesseract::common::LinkIdTransformMap& link_transforms,
                             const SceneState::JointValues& joints,
                             const tesseract::common::JointIdTransformMap& floating_joints,
                             const OFKTNode* node,
                             const Eigen::Isometry3d& parent_world_tf,
                             bool update_required) const
{
  Eigen::Isometry3d updated_parent_world_tf{ Eigen::Isometry3d::Identity() };
  const bool local_update_required =
      updateRequired(updated_parent_world_tf, joints, floating_joints, node, parent_world_tf);
  update_required = static_cast<bool>(update_required | local_update_required);  // NOLINT
  if (update_required)
    link_transforms[node->getLinkId()] = updated_parent_world_tf;

  for (const auto* child : node->getChildren())
    update(link_transforms, joints, floating_joints, child, updated_parent_world_tf, update_required);
}

void OFKTStateSolver::update(SceneState& state,
                             const OFKTNode* node,
                             const Eigen::Isometry3d& parent_world_tf,
                             bool update_required) const
{
  Eigen::Isometry3d updated_parent_world_tf{ Eigen::Isometry3d::Identity() };
  const bool local_update_required =
      updateRequired(updated_parent_world_tf, state.joints, state.floating_joints, node, parent_world_tf);
  update_required = static_cast<bool>(update_required | local_update_required);  // NOLINT
  if (update_required)
  {
    state.link_transforms[node->getLinkId()] = updated_parent_world_tf;
    state.joint_transforms[node->getJointId()] = updated_parent_world_tf;
  }

  for (const auto* child : node->getChildren())
    update(state, child, updated_parent_world_tf, update_required);
}

bool OFKTStateSolver::initHelper(const tesseract::scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  clear();

  if (scene_graph.isEmpty())
    return true;

  assert(scene_graph.isTree());

  const std::string& root_name = prefix + scene_graph.getRoot();

  root_ = std::make_unique<OFKTRootNode>(root_name);
  link_map_[root_->getLinkId()] = root_.get();
  current_state_.link_transforms[root_->getLinkId()] = root_->getWorldTransformation();
  link_ids_.push_back(root_->getLinkId());

  std::vector<JointLimits::ConstPtr> new_joints_limits;
  new_joints_limits.reserve(scene_graph.getJoints().size());
  ofkt_builder builder(*this, new_joints_limits, prefix);

  std::map<tesseract::scene_graph::SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<tesseract::scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
      index_map);

  int c = 0;
  tesseract::scene_graph::Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(
      static_cast<const tesseract::scene_graph::Graph&>(scene_graph),
      boost::visitor(builder).root_vertex(scene_graph.getVertex(root_name)).vertex_index_map(prop_index_map));

  // Populate Joint Limits
  addNewJointLimits(new_joints_limits);

  // Update transforms
  update(root_.get(), false);

  return true;
}

void OFKTStateSolver::moveLinkHelper(std::vector<std::shared_ptr<const JointLimits>>& new_joint_limits,
                                     const Joint& joint)
{
  auto* old_node = link_map_[LinkId::fromName(joint.child_link_name)];
  const JointId old_joint_id = old_node->getJointId();
  old_node->getParent()->removeChild(old_node);

  auto it = std::find(active_joint_ids_.begin(), active_joint_ids_.end(), old_joint_id);
  std::vector<LinkId> removed_links;
  removed_links.push_back(old_node->getLinkId());

  std::vector<JointId> removed_joints;
  std::vector<JointId> removed_active_joints;
  std::vector<long> removed_active_joints_indices;
  removed_joints.push_back(old_joint_id);
  if (it != active_joint_ids_.end())
  {
    removed_active_joints.push_back(old_joint_id);
    removed_active_joints_indices.push_back(std::distance(active_joint_ids_.begin(), it));
  }

  // store to add to new node
  std::vector<OFKTNode*> children = old_node->getChildren();

  // erase node
  nodes_.erase(old_joint_id);
  removeJointHelper(removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);
  current_state_.joints.erase(old_joint_id);
  current_state_.floating_joints.erase(old_joint_id);
  current_state_.joint_transforms.erase(old_joint_id);

  addNode(joint, joint.getName(), joint.parent_link_name, joint.child_link_name, new_joint_limits);

  const auto& new_jid = joint.getId();
  auto& replaced_node = nodes_[new_jid];

  // add back original nodes children and update parents
  for (auto* child : children)
  {
    replaced_node->addChild(child);
    child->setParent(replaced_node.get());
  }

  update(replaced_node.get(), true);
}

void OFKTStateSolver::replaceJointHelper(std::vector<std::shared_ptr<const JointLimits>>& new_joint_limits,
                                         const Joint& joint)
{
  const auto& jid = joint.getId();
  auto& n = nodes_[jid];

  if (n->getType() == joint.type && n->getParent()->getLinkName() == joint.parent_link_name)
  {
    n->getParent()->removeChild(n.get());
    n->setStaticTransformation(joint.parent_to_joint_origin_transform);
    if (n->getType() == JointType::FLOATING)
      current_state_.floating_joints[jid] = joint.parent_to_joint_origin_transform;

    OFKTNode* new_parent = link_map_[LinkId::fromName(joint.parent_link_name)];
    n->setParent(new_parent);
    new_parent->addChild(n.get());
  }
  else
  {
    moveLinkHelper(new_joint_limits, joint);
  }
}

void OFKTStateSolver::removeJointHelper(const std::vector<LinkId>& removed_links,
                                        const std::vector<JointId>& removed_joints,
                                        const std::vector<JointId>& removed_active_joints,
                                        const std::vector<long>& removed_active_joints_indices)
{
  if (!removed_links.empty())
  {
    link_ids_.erase(std::remove_if(link_ids_.begin(),
                                   link_ids_.end(),
                                   [&removed_links](const LinkId& lid) {
                                     return (std::find(removed_links.begin(), removed_links.end(), lid) !=
                                             removed_links.end());
                                   }),
                    link_ids_.end());
  }

  if (!removed_joints.empty())
  {
    joint_ids_.erase(std::remove_if(joint_ids_.begin(),
                                    joint_ids_.end(),
                                    [&removed_joints](const JointId& jid) {
                                      return (std::find(removed_joints.begin(), removed_joints.end(), jid) !=
                                              removed_joints.end());
                                    }),
                     joint_ids_.end());

    floating_joint_ids_.erase(std::remove_if(floating_joint_ids_.begin(),
                                             floating_joint_ids_.end(),
                                             [&removed_joints](const JointId& jid) {
                                               return (std::find(removed_joints.begin(),
                                                                 removed_joints.end(),
                                                                 jid) != removed_joints.end());
                                             }),
                              floating_joint_ids_.end());
  }

  if (!removed_active_joints.empty())
  {
    active_joint_ids_.erase(std::remove_if(active_joint_ids_.begin(),
                                           active_joint_ids_.end(),
                                           [&removed_active_joints](const JointId& jid) {
                                             return (std::find(removed_active_joints.begin(),
                                                               removed_active_joints.end(),
                                                               jid) != removed_active_joints.end());
                                           }),
                            active_joint_ids_.end());

    tesseract::common::KinematicLimits l1;
    l1.joint_limits.resize(static_cast<long int>(active_joint_ids_.size()), 2);
    l1.velocity_limits.resize(static_cast<long int>(active_joint_ids_.size()), 2);
    l1.acceleration_limits.resize(static_cast<long int>(active_joint_ids_.size()), 2);
    l1.jerk_limits.resize(static_cast<long int>(active_joint_ids_.size()), 2);

    long cnt = 0;
    for (long i = 0; i < limits_.joint_limits.rows(); ++i)
    {
      if (std::find(removed_active_joints_indices.begin(), removed_active_joints_indices.end(), i) ==
          removed_active_joints_indices.end())
      {
        l1.joint_limits.row(cnt) = limits_.joint_limits.row(i);
        l1.velocity_limits.row(cnt) = limits_.velocity_limits.row(i);
        l1.acceleration_limits.row(cnt) = limits_.acceleration_limits.row(i);
        l1.jerk_limits.row(cnt) = limits_.jerk_limits.row(i);
        ++cnt;
      }
    }

    limits_ = l1;
  }
}

void OFKTStateSolver::addNode(const tesseract::scene_graph::Joint& joint,
                              const std::string& joint_name,
                              const std::string& parent_link_name,
                              const std::string& child_link_name,
                              std::vector<std::shared_ptr<const JointLimits>>& new_joint_limits)
{
  const auto parent_lid = LinkId::fromName(parent_link_name);
  switch (joint.type)
  {
    case tesseract::scene_graph::JointType::FIXED:
    {
      OFKTNode* parent_node = link_map_[parent_lid];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform);
      link_map_[n->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      current_state_.link_transforms[n->getLinkId()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointId()] = n->getWorldTransformation();
      joint_ids_.push_back(n->getJointId());
      link_ids_.push_back(n->getLinkId());
      nodes_[n->getJointId()] = std::move(n);
      break;
    }
    case tesseract::scene_graph::JointType::REVOLUTE:
    {
      OFKTNode* parent_node = link_map_[parent_lid];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTRevoluteNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[n->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[n->getJointId()] = 0;
      current_state_.link_transforms[n->getLinkId()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointId()] = n->getWorldTransformation();
      joint_ids_.push_back(n->getJointId());
      active_joint_ids_.push_back(n->getJointId());
      link_ids_.push_back(n->getLinkId());
      new_joint_limits.push_back(joint.limits);
      nodes_[n->getJointId()] = std::move(n);
      break;
    }
    case tesseract::scene_graph::JointType::CONTINUOUS:
    {
      OFKTNode* parent_node = link_map_[parent_lid];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[n->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[n->getJointId()] = 0;
      current_state_.link_transforms[n->getLinkId()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointId()] = n->getWorldTransformation();
      joint_ids_.push_back(n->getJointId());
      active_joint_ids_.push_back(n->getJointId());
      link_ids_.push_back(n->getLinkId());
      new_joint_limits.push_back(joint.limits);
      nodes_[n->getJointId()] = std::move(n);
      break;
    }
    case tesseract::scene_graph::JointType::PRISMATIC:
    {
      OFKTNode* parent_node = link_map_[parent_lid];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[n->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[n->getJointId()] = 0;
      current_state_.link_transforms[n->getLinkId()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointId()] = n->getWorldTransformation();
      joint_ids_.push_back(n->getJointId());
      active_joint_ids_.push_back(n->getJointId());
      link_ids_.push_back(n->getLinkId());
      new_joint_limits.push_back(joint.limits);
      nodes_[n->getJointId()] = std::move(n);
      break;
    }
    case tesseract::scene_graph::JointType::FLOATING:
    {
      OFKTNode* parent_node = link_map_[parent_lid];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTFloatingNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform);
      link_map_[n->getLinkId()] = n.get();
      parent_node->addChild(n.get());
      current_state_.link_transforms[n->getLinkId()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointId()] = n->getWorldTransformation();
      current_state_.floating_joints[n->getJointId()] = n->getLocalTransformation();
      joint_ids_.push_back(n->getJointId());
      floating_joint_ids_.push_back(n->getJointId());
      link_ids_.push_back(n->getLinkId());
      nodes_[n->getJointId()] = std::move(n);
      break;
    }
    // LCOV_EXCL_START
    default:
    {
      throw std::runtime_error("Unsupported joint type for joint '" + joint_name + "'");
    }
      // LCOV_EXCL_STOP
  }
}

void OFKTStateSolver::removeNode(OFKTNode* node,
                                 std::vector<LinkId>& removed_links,
                                 std::vector<JointId>& removed_joints,
                                 std::vector<JointId>& removed_active_joints,
                                 std::vector<long>& removed_active_joints_indices)
{
  removed_links.push_back(node->getLinkId());
  removed_joints.push_back(node->getJointId());

  auto it = std::find(active_joint_ids_.begin(), active_joint_ids_.end(), node->getJointId());
  if (it != active_joint_ids_.end())
  {
    removed_active_joints.push_back(node->getJointId());
    removed_active_joints_indices.push_back(std::distance(active_joint_ids_.begin(), it));
  }

  current_state_.link_transforms.erase(node->getLinkId());
  current_state_.joints.erase(node->getJointId());
  current_state_.floating_joints.erase(node->getJointId());
  current_state_.joint_transforms.erase(node->getJointId());

  std::vector<OFKTNode*> children = node->getChildren();
  for (auto* child : children)
    removeNode(child, removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  if (node->getParent() != nullptr)
    node->getParent()->removeChild(node);

  link_map_.erase(node->getLinkId());
  nodes_.erase(node->getJointId());
}

void OFKTStateSolver::addNewJointLimits(const std::vector<std::shared_ptr<const JointLimits>>& new_joint_limits)
{
  // Populate Joint Limits
  if (!new_joint_limits.empty())
  {
    tesseract::common::KinematicLimits l;
    long s = limits_.joint_limits.rows() + static_cast<long>(new_joint_limits.size());
    l.joint_limits.resize(s, 2);
    l.velocity_limits.resize(s, 2);
    l.acceleration_limits.resize(s, 2);
    l.jerk_limits.resize(s, 2);

    l.joint_limits.block(0, 0, limits_.joint_limits.rows(), 2) = limits_.joint_limits;
    l.velocity_limits.block(0, 0, limits_.velocity_limits.rows(), 2) = limits_.velocity_limits;
    l.acceleration_limits.block(0, 0, limits_.acceleration_limits.rows(), 2) = limits_.acceleration_limits;
    l.jerk_limits.block(0, 0, limits_.jerk_limits.rows(), 2) = limits_.jerk_limits;

    long cnt = limits_.joint_limits.rows();
    for (const auto& limits : new_joint_limits)
    {
      l.joint_limits(cnt, 0) = limits->lower;
      l.joint_limits(cnt, 1) = limits->upper;
      l.velocity_limits(cnt, 0) = -limits->velocity;
      l.velocity_limits(cnt, 1) = limits->velocity;
      l.acceleration_limits(cnt, 0) = -limits->acceleration;
      l.acceleration_limits(cnt, 1) = limits->acceleration;
      l.jerk_limits(cnt, 0) = -limits->jerk;
      l.jerk_limits(cnt, 1) = limits->jerk;
      ++cnt;
    }
    limits_ = l;
  }
}

}  // namespace tesseract::scene_graph
