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
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_nodes.h>
#include <tesseract_common/utils.h>

namespace tesseract_scene_graph
{
/** @brief Everytime a vertex is visited for the first time add a new node to the tree */
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

    boost::graph_traits<tesseract_scene_graph::Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    tesseract_scene_graph::SceneGraph::Edge e = *ei;
    const tesseract_scene_graph::Joint::ConstPtr& joint = boost::get(boost::edge_joint, graph)[e];
    std::string joint_name = prefix_ + joint->getName();
    std::string parent_link_name = prefix_ + joint->parent_link_name;
    std::string child_link_name = prefix_ + joint->child_link_name;

    tree_.addNode(*joint, joint_name, parent_link_name, child_link_name, new_joints_limits_);
  }

protected:
  OFKTStateSolver& tree_;
  std::vector<JointLimits::ConstPtr>& new_joints_limits_;
  std::string prefix_;
};

void OFKTStateSolver::cloneHelper(OFKTStateSolver& cloned, const OFKTNode* node) const
{
  OFKTNode* parent_node = cloned.link_map_[node->getLinkName()];
  for (const OFKTNode* child : node->getChildren())
  {
    if (child->getType() == tesseract_scene_graph::JointType::FIXED)
    {
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, child->getLinkName(), child->getJointName(), child->getStaticTransformation());
      cloned.link_map_[child->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[child->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::REVOLUTE)
    {
      const auto* cn = static_cast<const OFKTRevoluteNode*>(child);

      auto n = std::make_unique<OFKTRevoluteNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::CONTINUOUS)
    {
      const auto* cn = static_cast<const OFKTContinuousNode*>(child);

      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::PRISMATIC)
    {
      const auto* cn = static_cast<const OFKTPrismaticNode*>(child);

      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, cn->getLinkName(), cn->getJointName(), cn->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[cn->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[cn->getJointName()] = std::move(n);
    }
    else
    {
      throw std::runtime_error("Unsupport OFKTNode type!");
    }

    cloneHelper(cloned, child);
  }
}

OFKTStateSolver::OFKTStateSolver(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  initHelper(scene_graph, prefix);
}

OFKTStateSolver::OFKTStateSolver(const std::string& root_name)
{
  root_ = std::make_unique<OFKTRootNode>(root_name);
  link_map_[root_name] = root_.get();
  current_state_.link_transforms[root_name] = root_->getWorldTransformation();
}

OFKTStateSolver::OFKTStateSolver(const OFKTStateSolver& other) { *this = other; }

OFKTStateSolver& OFKTStateSolver::operator=(const OFKTStateSolver& other)
{
  current_state_ = other.current_state_;
  joint_names_ = other.joint_names_;
  active_joint_names_ = other.active_joint_names_;
  link_names_ = other.link_names_;
  root_ = std::make_unique<OFKTRootNode>(other.root_->getLinkName());
  link_map_[other.root_->getLinkName()] = root_.get();
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
  joint_names_.clear();
  active_joint_names_.clear();
  link_names_.clear();
  nodes_.clear();
  link_map_.clear();
  limits_ = tesseract_common::KinematicLimits();
  root_ = nullptr;
}

void OFKTStateSolver::setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  assert(active_joint_names_.size() == static_cast<std::size_t>(joint_values.size()));
  Eigen::VectorXd jv = joint_values;
  for (std::size_t i = 0; i < active_joint_names_.size(); ++i)
  {
    nodes_[active_joint_names_[i]]->storeJointValue(joint_values(static_cast<long>(i)));
    current_state_.joints[active_joint_names_[i]] = joint_values(static_cast<long>(i));
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::unordered_map<std::string, double>& joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);

  for (const auto& joint : joint_values)
  {
    nodes_[joint.first]->storeJointValue(joint.second);
    current_state_.joints[joint.first] = joint.second;
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  assert(joint_names.size() == static_cast<std::size_t>(joint_values.size()));
  Eigen::VectorXd jv = joint_values;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    nodes_[joint_names[i]]->storeJointValue(joint_values(static_cast<long>(i)));
    current_state_.joints[joint_names[i]] = joint_values(static_cast<long>(i));
  }

  update(root_.get(), false);
}

SceneState OFKTStateSolver::getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  assert(static_cast<Eigen::Index>(active_joint_names_.size()) == joint_values.size());
  auto state = SceneState(current_state_);
  for (std::size_t i = 0; i < active_joint_names_.size(); ++i)
    state.joints[active_joint_names_[i]] = joint_values[static_cast<long>(i)];

  update(state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

SceneState OFKTStateSolver::getState(const std::unordered_map<std::string, double>& joint_values) const
{
  auto state = SceneState(current_state_);
  for (const auto& joint : joint_values)
    state.joints[joint.first] = joint.second;

  update(state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

SceneState OFKTStateSolver::getState(const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  assert(static_cast<Eigen::Index>(joint_names.size()) == joint_values.size());
  auto state = SceneState(current_state_);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    state.joints[joint_names[i]] = joint_values[static_cast<long>(i)];

  update(state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

SceneState OFKTStateSolver::getState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_;
}

SceneState OFKTStateSolver::getRandomState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return getState(active_joint_names_, tesseract_common::generateRandomNumber(limits_.joint_limits));
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::unordered_map<std::string, double> joints = current_state_.joints;
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joints[active_joint_names_[static_cast<std::size_t>(i)]] = joint_values[i];

  return calcJacobianHelper(joints, link_name);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const std::unordered_map<std::string, double>& joints_values,
                                             const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::unordered_map<std::string, double> joints = current_state_.joints;
  for (const auto& joint : joints_values)
    joints[joint.first] = joint.second;

  return calcJacobianHelper(joints, link_name);
}

Eigen::MatrixXd OFKTStateSolver::getJacobian(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::unordered_map<std::string, double> joints = current_state_.joints;
  for (Eigen::Index i = 0; i < joint_values.rows(); ++i)
    joints[joint_names[static_cast<std::size_t>(i)]] = joint_values[i];

  return calcJacobianHelper(joints, link_name);
}

Eigen::MatrixXd OFKTStateSolver::calcJacobianHelper(const std::unordered_map<std::string, double>& joints,
                                                    const std::string& link_name) const
{
  OFKTNode* node = link_map_.at(link_name);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(active_joint_names_.size()));

  Eigen::Isometry3d total_tf = Eigen::Isometry3d::Identity();
  while (node != root_.get())
  {
    if (node->getType() == JointType::FIXED || node->getType() == JointType::FLOATING)
    {
      total_tf = node->getLocalTransformation() * total_tf;
    }
    else
    {
      Eigen::Isometry3d local_tf = node->computeLocalTransformation(joints.at(node->getJointName()));
      total_tf = local_tf * total_tf;

      Eigen::Index idx =
          std::distance(active_joint_names_.begin(),
                        std::find(active_joint_names_.begin(), active_joint_names_.end(), node->getJointName()));
      Eigen::VectorXd twist = node->getLocalTwist();
      tesseract_common::twistChangeRefPoint(twist, total_tf.translation() - local_tf.translation());
      tesseract_common::twistChangeBase(twist, total_tf.inverse());
      jacobian.col(idx) = twist;
    }

    node = node->getParent();
  }

  tesseract_common::jacobianChangeBase(jacobian, total_tf);
  return jacobian;
}

std::vector<std::string> OFKTStateSolver::getJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return joint_names_;
}

std::vector<std::string> OFKTStateSolver::getActiveJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return active_joint_names_;
}

std::string OFKTStateSolver::getBaseLinkName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return root_->getLinkName();
}

std::vector<std::string> OFKTStateSolver::getLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return link_names_;
}

std::vector<std::string> OFKTStateSolver::getActiveLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> link_names;
  link_names.reserve(nodes_.size());
  loadActiveLinkNamesRecursive(link_names, root_.get(), false);
  return link_names;
}

std::vector<std::string> OFKTStateSolver::getStaticLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> link_names;
  link_names.reserve(nodes_.size());
  loadStaticLinkNamesRecursive(link_names, root_.get());
  return link_names;
}

bool OFKTStateSolver::isActiveLinkName(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> active_link_names = getActiveLinkNames();
  return (std::find(active_link_names.begin(), active_link_names.end(), link_name) != active_link_names.end());
}

bool OFKTStateSolver::hasLinkName(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return (std::find(link_names_.begin(), link_names_.end(), link_name) != link_names_.end());
}

tesseract_common::KinematicLimits OFKTStateSolver::getLimits() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return limits_;
}

bool OFKTStateSolver::addLink(const Link& link, const Joint& joint)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (link_map_.find(link.getName()) != link_map_.end())
  {
    return false;
  }

  if (nodes_.find(joint.getName()) != nodes_.end())
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
  auto it = nodes_.find(joint.getName());
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to replace joint '%s' which does not exist!",
                            joint.getName().c_str());
    return false;
  }

  if (link_map_.find(joint.parent_link_name) == link_map_.end())
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

  if (link_map_.find(joint.child_link_name) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to link '%s' that does not exist!", joint.child_link_name.c_str());
    return false;
  }

  if (link_map_.find(joint.child_link_name) == link_map_.end())
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

bool OFKTStateSolver::removeLink(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = link_map_.find(name);
  if (it == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to remove link '%s' which does not exist!", name.c_str());
    return false;
  }

  std::vector<std::string> removed_links;
  removed_links.reserve(nodes_.size());

  std::vector<std::string> removed_joints;
  removed_joints.reserve(nodes_.size());

  std::vector<std::string> removed_active_joints;
  removed_active_joints.reserve(nodes_.size());

  std::vector<long> removed_active_joints_indices;
  removed_active_joints_indices.reserve(nodes_.size());

  removeNode(it->second, removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  // Remove deleted joints
  removeJointHelper(removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::removeJoint(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to remove joint '%s' which does not exist!", name.c_str());
    return false;
  }

  std::vector<std::string> removed_links;
  removed_links.reserve(nodes_.size());

  std::vector<std::string> removed_joints;
  removed_joints.reserve(nodes_.size());

  std::vector<std::string> removed_active_joints;
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
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to move joint '%s' which does not exist!", name.c_str());
    return false;
  }

  if (link_map_.find(parent_link) == link_map_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to move joint '%s' to parent link '%s' which does not exist!",
                            name.c_str(),
                            parent_link.c_str());
    return false;
  }

  auto& n = it->second;
  n->getParent()->removeChild(n.get());
  OFKTNode* new_parent = link_map_[parent_link];
  n->setParent(new_parent);
  new_parent->addChild(n.get());

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' origin which does not exist!", name.c_str());
    return false;
  }

  it->second->setStaticTransformation(new_origin);

  update(root_.get(), false);

  return true;
}

bool OFKTStateSolver::changeJointPositionLimits(const std::string& name, double lower, double upper)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' postionerlimits which does not exist!",
                            name.c_str());
    return false;
  }

  long idx = std::distance(active_joint_names_.begin(),
                           std::find(active_joint_names_.begin(), active_joint_names_.end(), name));
  limits_.joint_limits(idx, 0) = lower;
  limits_.joint_limits(idx, 1) = upper;
  return true;
}

bool OFKTStateSolver::changeJointVelocityLimits(const std::string& name, double limit)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' postionerlimits which does not exist!",
                            name.c_str());
    return false;
  }

  long idx = std::distance(active_joint_names_.begin(),
                           std::find(active_joint_names_.begin(), active_joint_names_.end(), name));
  limits_.velocity_limits(idx) = limit;
  return true;
}

bool OFKTStateSolver::changeJointAccelerationLimits(const std::string& name, double limit)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto it = nodes_.find(name);
  if (it == nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, tried to change joint '%s' postionerlimits which does not exist!",
                            name.c_str());
    return false;
  }

  long idx = std::distance(active_joint_names_.begin(),
                           std::find(active_joint_names_.begin(), active_joint_names_.end(), name));
  limits_.acceleration_limits(idx) = limit;
  return true;
}

bool OFKTStateSolver::insertSceneGraph(const SceneGraph& scene_graph, const Joint& joint, const std::string& prefix)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (root_ == nullptr)
    return false;  // throw std::runtime_error("OFKT State Solver is empty and tried to add scene graph with joint");

  std::string parent_link = joint.parent_link_name;
  std::string child_link = joint.child_link_name;

  // Assumes the joint already contains the prefix in the parent and child link names
  if (!prefix.empty())
    child_link.erase(0, prefix.length());

  if (link_map_.find(parent_link) == link_map_.end() || scene_graph.getLink(child_link) == nullptr)
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, Failed to add inserted graph, provided joint link names do not exist in "
                            "inserted graph!");
    return false;
  }

  if (nodes_.find(joint.getName()) != nodes_.end())
  {
    CONSOLE_BRIDGE_logError("OFKTStateSolver, Failed to add inserted graph, provided joint name %s already exists!",
                            joint.getName().c_str());
    return false;
  }

  std::vector<JointLimits::ConstPtr> new_joints_limits;
  new_joints_limits.reserve(boost::num_edges(scene_graph));

  addNode(joint, joint.getName(), joint.parent_link_name, joint.child_link_name, new_joints_limits);

  ofkt_builder builder(*this, new_joints_limits, prefix);

  std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
      index_map);

  int c = 0;
  tesseract_scene_graph::Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(static_cast<const tesseract_scene_graph::Graph&>(scene_graph),
                            boost::visitor(builder)
                                .root_vertex(scene_graph.getVertex(scene_graph.getRoot()))
                                .vertex_index_map(prop_index_map));

  // Populate Joint Limits
  addNewJointLimits(new_joints_limits);

  update(root_.get(), false);
  return true;
}

void OFKTStateSolver::loadActiveLinkNamesRecursive(std::vector<std::string>& active_link_names,
                                                   const OFKTNode* node,
                                                   bool active) const
{
  if (active)
  {
    active_link_names.push_back(node->getLinkName());
    for (const auto* child : node->getChildren())
      loadActiveLinkNamesRecursive(active_link_names, child, active);
  }
  else
  {
    if (node->getType() == tesseract_scene_graph::JointType::FIXED ||
        node->getType() == tesseract_scene_graph::JointType::FLOATING)
    {
      for (const auto* child : node->getChildren())
        loadActiveLinkNamesRecursive(active_link_names, child, active);
    }
    else
    {
      active_link_names.push_back(node->getLinkName());
      for (const auto* child : node->getChildren())
        loadActiveLinkNamesRecursive(active_link_names, child, true);
    }
  }
}

void OFKTStateSolver::loadStaticLinkNamesRecursive(std::vector<std::string>& static_link_names,
                                                   const OFKTNode* node) const
{
  if (node->getType() == tesseract_scene_graph::JointType::FIXED ||
      node->getType() == tesseract_scene_graph::JointType::FLOATING)
  {
    static_link_names.push_back(node->getLinkName());
    for (const auto* child : node->getChildren())
      loadStaticLinkNamesRecursive(static_link_names, child);
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
    current_state_.link_transforms[node->getLinkName()] = node->getWorldTransformation();
    current_state_.joint_transforms[node->getJointName()] = node->getWorldTransformation();
  }

  for (auto* child : node->getChildren())
    update(child, update_required);
}

void OFKTStateSolver::update(SceneState& state,
                             const OFKTNode* node,
                             Eigen::Isometry3d parent_world_tf,
                             bool update_required) const
{
  if (node->getType() != tesseract_scene_graph::JointType::FIXED)
  {
    double jv = state.joints[node->getJointName()];
    if (!tesseract_common::almostEqualRelativeAndAbs(node->getJointValue(), jv, 1e-8))
    {
      parent_world_tf = parent_world_tf * node->computeLocalTransformation(jv);
      update_required = true;
    }
    else
    {
      parent_world_tf = parent_world_tf * node->getLocalTransformation();
    }
  }
  else
  {
    parent_world_tf = parent_world_tf * node->getLocalTransformation();
  }

  if (update_required)
  {
    state.link_transforms[node->getLinkName()] = parent_world_tf;
    state.joint_transforms[node->getJointName()] = parent_world_tf;
  }

  for (const auto* child : node->getChildren())
    update(state, child, parent_world_tf, update_required);
}

bool OFKTStateSolver::initHelper(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  clear();

  if (scene_graph.isEmpty())
    return true;

  assert(scene_graph.isTree());

  const std::string& root_name = prefix + scene_graph.getRoot();

  root_ = std::make_unique<OFKTRootNode>(root_name);
  link_map_[root_name] = root_.get();
  current_state_.link_transforms[root_name] = root_->getWorldTransformation();
  link_names_.push_back(root_name);

  std::vector<JointLimits::ConstPtr> new_joints_limits;
  new_joints_limits.reserve(scene_graph.getJoints().size());
  ofkt_builder builder(*this, new_joints_limits, prefix);

  std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
      index_map);

  int c = 0;
  tesseract_scene_graph::Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(
      static_cast<const tesseract_scene_graph::Graph&>(scene_graph),
      boost::visitor(builder).root_vertex(scene_graph.getVertex(root_name)).vertex_index_map(prop_index_map));

  // Populate Joint Limits
  addNewJointLimits(new_joints_limits);

  // Update transforms
  update(root_.get(), false);

  return true;
}

void OFKTStateSolver::moveLinkHelper(std::vector<JointLimits::ConstPtr>& new_joint_limits, const Joint& joint)
{
  auto* old_node = link_map_[joint.child_link_name];
  const std::string& old_joint_name = old_node->getJointName();
  old_node->getParent()->removeChild(old_node);

  auto it = std::find(active_joint_names_.begin(), active_joint_names_.end(), old_joint_name);
  std::vector<std::string> removed_links;
  removed_links.push_back(joint.child_link_name);

  std::vector<std::string> removed_joints;
  std::vector<std::string> removed_active_joints;
  std::vector<long> removed_active_joints_indices;
  removed_joints.push_back(old_joint_name);
  if (it != active_joint_names_.end())
  {
    removed_active_joints.push_back(old_joint_name);
    removed_active_joints_indices.push_back(std::distance(active_joint_names_.begin(), it));
  }

  // store to add to new node
  std::vector<OFKTNode*> children = old_node->getChildren();

  // erase node
  nodes_.erase(old_joint_name);
  removeJointHelper(removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);
  current_state_.joints.erase(old_joint_name);
  current_state_.joint_transforms.erase(old_joint_name);

  addNode(joint, joint.getName(), joint.parent_link_name, joint.child_link_name, new_joint_limits);

  auto& replaced_node = nodes_[joint.getName()];

  // add back original nodes children and update parents
  for (auto* child : children)
  {
    replaced_node->addChild(child);
    child->setParent(replaced_node.get());
  }

  update(replaced_node.get(), true);
}

void OFKTStateSolver::replaceJointHelper(std::vector<JointLimits::ConstPtr>& new_joint_limits, const Joint& joint)
{
  auto& n = nodes_[joint.getName()];

  if (n->getType() == joint.type && n->getParent()->getLinkName() == joint.parent_link_name)
  {
    n->getParent()->removeChild(n.get());
    n->setStaticTransformation(joint.parent_to_joint_origin_transform);
    OFKTNode* new_parent = link_map_[joint.parent_link_name];
    n->setParent(new_parent);
    new_parent->addChild(n.get());
  }
  else
  {
    moveLinkHelper(new_joint_limits, joint);
  }
}

void OFKTStateSolver::removeJointHelper(const std::vector<std::string>& removed_links,
                                        const std::vector<std::string>& removed_joints,
                                        const std::vector<std::string>& removed_active_joints,
                                        const std::vector<long>& removed_active_joints_indices)
{
  if (!removed_links.empty())
  {
    link_names_.erase(std::remove_if(link_names_.begin(),
                                     link_names_.end(),
                                     [removed_links](const std::string& link_name) {
                                       return (std::find(removed_links.begin(), removed_links.end(), link_name) !=
                                               removed_links.end());
                                     }),
                      link_names_.end());
  }

  if (!removed_joints.empty())
  {
    joint_names_.erase(std::remove_if(joint_names_.begin(),
                                      joint_names_.end(),
                                      [removed_joints](const std::string& joint_name) {
                                        return (std::find(removed_joints.begin(), removed_joints.end(), joint_name) !=
                                                removed_joints.end());
                                      }),
                       joint_names_.end());
  }

  if (!removed_active_joints.empty())
  {
    active_joint_names_.erase(std::remove_if(active_joint_names_.begin(),
                                             active_joint_names_.end(),
                                             [removed_active_joints](const std::string& joint_name) {
                                               return (std::find(removed_active_joints.begin(),
                                                                 removed_active_joints.end(),
                                                                 joint_name) != removed_active_joints.end());
                                             }),
                              active_joint_names_.end());

    tesseract_common::KinematicLimits l1;
    l1.joint_limits.resize(static_cast<long int>(active_joint_names_.size()), 2);
    l1.velocity_limits.resize(static_cast<long int>(active_joint_names_.size()));
    l1.acceleration_limits.resize(static_cast<long int>(active_joint_names_.size()));

    long cnt = 0;
    for (long i = 0; i < limits_.joint_limits.rows(); ++i)
    {
      if (std::find(removed_active_joints_indices.begin(), removed_active_joints_indices.end(), i) ==
          removed_active_joints_indices.end())
      {
        l1.joint_limits.row(cnt) = limits_.joint_limits.row(i);
        l1.velocity_limits(cnt) = limits_.velocity_limits(i);
        l1.acceleration_limits(cnt) = limits_.acceleration_limits(i);
        ++cnt;
      }
    }

    limits_ = l1;
  }
}

void OFKTStateSolver::addNode(const tesseract_scene_graph::Joint& joint,
                              const std::string& joint_name,
                              const std::string& parent_link_name,
                              const std::string& child_link_name,
                              std::vector<JointLimits::ConstPtr>& new_joint_limits)
{
  switch (joint.type)
  {
    case tesseract_scene_graph::JointType::FIXED:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_.link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointName()] = n->getWorldTransformation();
      joint_names_.push_back(joint_name);
      link_names_.push_back(n->getLinkName());
      nodes_[joint_name] = std::move(n);
      break;
    }
    case tesseract_scene_graph::JointType::REVOLUTE:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTRevoluteNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[joint_name] = 0;
      current_state_.link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointName()] = n->getWorldTransformation();
      joint_names_.push_back(joint_name);
      active_joint_names_.push_back(joint_name);
      link_names_.push_back(n->getLinkName());
      new_joint_limits.push_back(joint.limits);
      nodes_[joint_name] = std::move(n);
      break;
    }
    case tesseract_scene_graph::JointType::CONTINUOUS:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[joint_name] = 0;
      current_state_.link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointName()] = n->getWorldTransformation();
      joint_names_.push_back(joint_name);
      active_joint_names_.push_back(joint_name);
      link_names_.push_back(n->getLinkName());
      new_joint_limits.push_back(joint.limits);
      nodes_[joint_name] = std::move(n);
      break;
    }
    case tesseract_scene_graph::JointType::PRISMATIC:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      assert(parent_node != nullptr);
      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, child_link_name, joint_name, joint.parent_to_joint_origin_transform, joint.axis);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_.joints[joint_name] = 0;
      current_state_.link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_.joint_transforms[n->getJointName()] = n->getWorldTransformation();
      joint_names_.push_back(joint_name);
      active_joint_names_.push_back(joint_name);
      link_names_.push_back(n->getLinkName());
      new_joint_limits.push_back(joint.limits);
      nodes_[joint_name] = std::move(n);
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupported joint type for joint '" + joint_name + "'");
    }
  }
}

void OFKTStateSolver::removeNode(OFKTNode* node,
                                 std::vector<std::string>& removed_links,
                                 std::vector<std::string>& removed_joints,
                                 std::vector<std::string>& removed_active_joints,
                                 std::vector<long>& removed_active_joints_indices)
{
  removed_links.push_back(node->getLinkName());
  removed_joints.push_back(node->getJointName());

  auto it = std::find(active_joint_names_.begin(), active_joint_names_.end(), node->getJointName());
  if (it != active_joint_names_.end())
  {
    removed_active_joints.push_back(node->getJointName());
    removed_active_joints_indices.push_back(std::distance(active_joint_names_.begin(), it));
  }

  current_state_.link_transforms.erase(node->getLinkName());
  current_state_.joints.erase(node->getJointName());
  current_state_.joint_transforms.erase(node->getJointName());

  std::vector<OFKTNode*> children = node->getChildren();
  for (auto* child : node->getChildren())
    removeNode(child, removed_links, removed_joints, removed_active_joints, removed_active_joints_indices);

  if (node->getParent() != nullptr)
    node->getParent()->removeChild(node);

  link_map_.erase(node->getLinkName());
  nodes_.erase(node->getJointName());
}

void OFKTStateSolver::addNewJointLimits(const std::vector<JointLimits::ConstPtr>& new_joint_limits)
{
  // Populate Joint Limits
  if (!new_joint_limits.empty())
  {
    tesseract_common::KinematicLimits l;
    long s = limits_.joint_limits.rows() + static_cast<long>(new_joint_limits.size());
    l.joint_limits.resize(s, 2);
    l.velocity_limits.resize(s);
    l.acceleration_limits.resize(s);

    l.joint_limits.block(0, 0, limits_.joint_limits.rows(), 2) = limits_.joint_limits;
    l.velocity_limits.head(limits_.joint_limits.rows()) = limits_.velocity_limits;
    l.acceleration_limits.head(limits_.joint_limits.rows()) = limits_.acceleration_limits;

    long cnt = limits_.joint_limits.rows();
    for (const auto& limits : new_joint_limits)
    {
      l.joint_limits(cnt, 0) = limits->lower;
      l.joint_limits(cnt, 1) = limits->upper;
      l.velocity_limits(cnt) = limits->velocity;
      l.acceleration_limits(cnt) = limits->acceleration;
      ++cnt;
    }
    limits_ = l;
  }
}

}  // namespace tesseract_scene_graph
