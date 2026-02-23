/**
 * @file graph.cpp
 * @brief A basic scene graph using boost
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/graph/directed_graph.hpp>  // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/copy.hpp>
#include <fstream>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/utils.h>

namespace tesseract::scene_graph
{
struct cycle_detector : public boost::dfs_visitor<>
{
  cycle_detector(bool& ascyclic) : ascyclic_(ascyclic) {}

  template <class e, class g>
  void back_edge(e, g&)
  {
    ascyclic_ = false;
  }

protected:
  bool& ascyclic_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

struct tree_detector : public boost::dfs_visitor<>
{
  tree_detector(bool& tree) : tree_(tree) {}

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));

    if (num_in_edges > 1)
    {
      tree_ = false;
      return;
    }

    // Check if more that one root exist
    if (num_in_edges == 0 && found_root_)
    {
      tree_ = false;
      return;
    }

    if (num_in_edges == 0)
      found_root_ = true;

    // Check if not vertex is unused.
    if (num_in_edges == 0 && boost::out_degree(vertex, graph) == 0)
    {
      tree_ = false;
      return;
    }
  }

  template <class e, class g>
  void back_edge(e, const g&)
  {
    tree_ = false;
  }

protected:
  bool& tree_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  bool found_root_{ false };
};

struct children_detector : public boost::default_bfs_visitor
{
  children_detector(std::vector<std::string>& children) : children_(children) {}

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    children_.push_back(boost::get(boost::vertex_link, graph)[vertex]->getName());
  }

protected:
  std::vector<std::string>& children_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

struct adjacency_detector : public boost::default_bfs_visitor
{
  adjacency_detector(std::unordered_map<std::string, std::string>& adjacency_map,
                     std::map<SceneGraph::Vertex, boost::default_color_type>& color_map,
                     const std::string& base_link_name,
                     const std::vector<std::string>& terminate_on_links)
    : adjacency_map_(adjacency_map)
    , color_map_(color_map)
    , base_link_name_(base_link_name)
    , terminate_on_links_(terminate_on_links)
  {
  }

  template <class u, class g>
  void examine_vertex(u vertex, const g& graph)
  {
    for (auto vd : boost::make_iterator_range(adjacent_vertices(vertex, graph)))
    {
      std::string adj_link = boost::get(boost::vertex_link, graph)[vd]->getName();
      if (std::find(terminate_on_links_.begin(), terminate_on_links_.end(), adj_link) != terminate_on_links_.end())
        color_map_[vd] = boost::default_color_type::black_color;
    }
  }

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    std::string adj_link = boost::get(boost::vertex_link, graph)[vertex]->getName();
    adjacency_map_[adj_link] = base_link_name_;
  }

protected:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  std::unordered_map<std::string, std::string>& adjacency_map_;
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  std::map<SceneGraph::Vertex, boost::default_color_type>& color_map_;
  const std::string& base_link_name_;                   // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  const std::vector<std::string>& terminate_on_links_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

using UGraph =
    boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, VertexProperty, EdgeProperty, GraphProperty>;

struct ugraph_vertex_copier
{
  ugraph_vertex_copier(const Graph& g1, UGraph& g2)
    : vertex_all_map1(get(boost::vertex_all, g1)), vertex_all_map2(get(boost::vertex_all, g2))
  {
  }

  template <typename Vertex1, typename Vertex2>
  void operator()(const Vertex1& v1, Vertex2& v2) const
  {
    boost::put(vertex_all_map2, v2, get(vertex_all_map1, v1));
  }
  typename boost::property_map<Graph, boost::vertex_all_t>::const_type vertex_all_map1;
  mutable typename boost::property_map<UGraph, boost::vertex_all_t>::type vertex_all_map2;
};

SceneGraph::SceneGraph(const std::string& name) : acm_(std::make_shared<tesseract::common::AllowedCollisionMatrix>())
{
  boost::set_property(static_cast<Graph&>(*this), boost::graph_name, name);
}

SceneGraph::SceneGraph(SceneGraph&& other) noexcept
  : Graph(other)  // Graph does not have a move constructor
  , link_map_(std::move(other.link_map_))
  , joint_map_(std::move(other.joint_map_))
  , acm_(std::move(other.acm_))
{
  rebuildLinkAndJointMaps();
}

SceneGraph& SceneGraph::operator=(SceneGraph&& other) noexcept
{
  Graph::operator=(other);  // Graph does not have move assignment operator

  link_map_ = std::move(other.link_map_);
  joint_map_ = std::move(other.joint_map_);
  acm_ = std::move(other.acm_);

  rebuildLinkAndJointMaps();

  return *this;
}

SceneGraph::UPtr SceneGraph::clone() const
{
  auto cloned_graph = std::make_unique<SceneGraph>();

  for (auto& link : getLinks())
  {
    cloned_graph->addLink(link->clone(link->getName()));
    cloned_graph->setLinkVisibility(link->getName(), getLinkVisibility(link->getName()));
    cloned_graph->setLinkCollisionEnabled(link->getName(), getLinkCollisionEnabled(link->getName()));
  }

  for (auto& joint : getJoints())
    cloned_graph->addJoint(joint->clone(joint->getName()));

  cloned_graph->getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(*getAllowedCollisionMatrix());

  cloned_graph->setName(getName());
  cloned_graph->setRoot(getRoot());

  return cloned_graph;
}

void SceneGraph::clear()
{
  Graph::clear();
  link_map_.clear();
  joint_map_.clear();
  acm_->clearAllowedCollisions();
}

void SceneGraph::setName(const std::string& name)
{
  boost::set_property(static_cast<Graph&>(*this), boost::graph_name, name);
}

const std::string& SceneGraph::getName() const
{
  return boost::get_property(static_cast<const Graph&>(*this), boost::graph_name);
}

bool SceneGraph::setRoot(const std::string& name)
{
  auto found = link_map_.find(name);

  if (found == link_map_.end())
    return false;

  boost::set_property(static_cast<Graph&>(*this), boost::graph_root, name);

  return true;
}

const std::string& SceneGraph::getRoot() const
{
  return boost::get_property(static_cast<const Graph&>(*this), boost::graph_root);
}

bool SceneGraph::addLink(const Link& link, bool replace_allowed)
{
  auto link_ptr = std::make_shared<tesseract::scene_graph::Link>(link.clone());
  return addLinkHelper(link_ptr, replace_allowed);
}

bool SceneGraph::addLink(const Link& link, const Joint& joint)
{
  if (getLink(link.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) with same name as an existing link.", link.getName().c_str());
    return false;
  }

  if (getJoint(joint.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add joint (%s) with same name as an existing joint.", joint.getName().c_str());
    return false;
  }

  if (!addLinkHelper(std::make_shared<Link>(link.clone())))
    return false;  // LCOV_EXCL_LINE

  if (!addJointHelper(std::make_shared<Joint>(joint.clone())))
    return false;  // LCOV_EXCL_LINE

  return true;
}

bool SceneGraph::addLinkHelper(const std::shared_ptr<Link>& link_ptr, bool replace_allowed)
{
  auto found = link_map_.find(link_ptr->getName());
  bool link_exists = (found != link_map_.end());
  if (link_exists && !replace_allowed)
    return false;

  if (link_exists && replace_allowed)
  {  // replacing an existing link
    found->second.first = link_ptr;
    boost::property_map<Graph, boost::vertex_link_t>::type param = get(boost::vertex_link, static_cast<Graph&>(*this));
    param[found->second.second] = link_ptr;
  }
  else
  {  // Adding a new link
    VertexProperty info(link_ptr);
    Vertex v = boost::add_vertex(info, static_cast<Graph&>(*this));
    link_map_[link_ptr->getName()] = std::make_pair(link_ptr, v);

    // First link added set as root
    if (link_map_.size() == 1)
      setRoot(link_ptr->getName());
  }

  return true;
}

std::shared_ptr<const Link> SceneGraph::getLink(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    return nullptr;

  return found->second.first;
}

std::vector<std::shared_ptr<const Link>> SceneGraph::getLinks() const
{
  std::vector<Link::ConstPtr> links;
  links.reserve(link_map_.size());
  for (const auto& link : link_map_)
    links.push_back(link.second.first);

  return links;
}

std::vector<std::shared_ptr<const Link>> SceneGraph::getLeafLinks() const
{
  std::vector<Link::ConstPtr> links;
  links.reserve(link_map_.size());
  for (const auto& link : link_map_)
  {
    if (boost::out_degree(link.second.second, *this) == 0)
      links.push_back(link.second.first);
  }

  return links;
}

bool SceneGraph::removeLink(const std::string& name, bool recursive)
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) from scene graph that does not exist.", name.c_str());
    return false;
  }

  std::vector<std::string> adjacent_link_names = getAdjacentLinkNames(name);

  // Need to remove all inbound and outbound edges first
  Vertex vertex = getVertex(name);
  boost::clear_vertex(vertex, *this);

  // rebuild joint_map
  joint_map_.clear();
  Graph::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(*this); ei != ei_end; ++ei)
  {
    Edge e = *ei;
    Joint::Ptr joint = boost::get(boost::edge_joint, *this)[e];
    joint_map_[joint->getName()] = std::make_pair(joint, e);
  }

  // Now remove vertex
  boost::remove_vertex(found->second.second, static_cast<Graph&>(*this));
  link_map_.erase(name);

  // Need to remove any reference to link in allowed collision matrix
  removeAllowedCollision(name);

  if (recursive)
  {
    for (const auto& link_name : adjacent_link_names)
    {
      if (getInboundJoints(link_name).empty())
        removeLink(link_name, true);
    }
  }

  return true;
}

bool SceneGraph::moveLink(const Joint& joint)
{
  if (link_map_.find(joint.child_link_name) == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move link (%s) in scene graph that does not exist.",
                           joint.child_link_name.c_str());
    return false;
  }

  if (link_map_.find(joint.parent_link_name) == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move link (%s) in scene graph that parent link (%s) which does not exist.",
                           joint.child_link_name.c_str(),
                           joint.parent_link_name.c_str());
    return false;
  }

  std::vector<tesseract::scene_graph::Joint::ConstPtr> joints = getInboundJoints(joint.child_link_name);
  for (const auto& joint : joints)
    removeJoint(joint->getName());

  return addJoint(joint);
}

void SceneGraph::setLinkVisibility(const std::string& name, bool visibility)
{
  link_map_.at(name).first->visible = visibility;
}

bool SceneGraph::getLinkVisibility(const std::string& name) const { return link_map_.at(name).first->visible; }

void SceneGraph::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  link_map_.at(name).first->collision_enabled = enabled;
}

bool SceneGraph::getLinkCollisionEnabled(const std::string& name) const
{
  return link_map_.at(name).first->collision_enabled;
}

bool SceneGraph::addJoint(const Joint& joint)
{
  auto joint_ptr = std::make_shared<tesseract::scene_graph::Joint>(joint.clone());
  return addJointHelper(joint_ptr);
}

bool SceneGraph::addJointHelper(const std::shared_ptr<Joint>& joint_ptr)
{
  auto parent = link_map_.find(joint_ptr->parent_link_name);
  auto child = link_map_.find(joint_ptr->child_link_name);
  auto found = joint_map_.find(joint_ptr->getName());

  if (parent == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Parent link (%s) does not exist in scene graph.", joint_ptr->parent_link_name.c_str());
    return false;
  }

  if (child == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Child link (%s) does not exist in scene graph.", joint_ptr->child_link_name.c_str());
    return false;
  }

  if (found != joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Joint with name (%s) already exists in scene graph.", joint_ptr->getName().c_str());
    return false;
  }

  if ((joint_ptr->type != JointType::FIXED) && (joint_ptr->type != JointType::FLOATING) &&
      (joint_ptr->type != JointType::CONTINUOUS) && joint_ptr->limits == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Joint with name (%s) requires limits because it is not of type fixed, floating or "
                           "continuous.",
                           joint_ptr->getName().c_str());
    return false;
  }

  // Need to set limits for continuous joints. TODO: This may not be required
  // by the optimization library but may be nice to have
  if (joint_ptr->type == tesseract::scene_graph::JointType::CONTINUOUS)
  {
    if (joint_ptr->limits == nullptr)
    {
      joint_ptr->limits = std::make_shared<JointLimits>(-4 * M_PI, 4 * M_PI, 0, 2, 1, 1000);
    }
    else if (tesseract::common::almostEqualRelativeAndAbs(joint_ptr->limits->lower, joint_ptr->limits->upper, 1e-5))
    {
      joint_ptr->limits->lower = -4 * M_PI;
      joint_ptr->limits->upper = +4 * M_PI;
    }
  }

  double d = joint_ptr->parent_to_joint_origin_transform.translation().norm();

  EdgeProperty info(joint_ptr, d);
  std::pair<Edge, bool> e =
      boost::add_edge(parent->second.second, child->second.second, info, static_cast<Graph&>(*this));
  assert(e.second == true);
  joint_map_[joint_ptr->getName()] = std::make_pair(joint_ptr, e.first);

  return true;
}

std::shared_ptr<const Joint> SceneGraph::getJoint(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return nullptr;

  return found->second.first;
}

bool SceneGraph::removeJoint(const std::string& name, bool recursive)
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return false;

  if (!recursive)
  {
    boost::remove_edge(found->second.second, static_cast<Graph&>(*this));
    joint_map_.erase(name);
  }
  else
  {
    if (getInboundJoints(found->second.first->child_link_name).size() == 1)
    {
      std::string child_link_name = found->second.first->child_link_name;
      removeLink(child_link_name, true);
    }
  }

  return true;
}

bool SceneGraph::moveJoint(const std::string& name, const std::string& parent_link)
{
  auto found_joint = joint_map_.find(name);
  auto found_parent_link = link_map_.find(parent_link);

  if (found_joint == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move Joint with name (%s) which does not exist in scene graph.", name.c_str());
    return false;
  }

  if (found_parent_link == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move Joint with name (%s) to parent link (%s) which does not exist in scene "
                           "graph.",
                           name.c_str(),
                           parent_link.c_str());
    return false;
  }

  Joint::Ptr joint = found_joint->second.first;
  if (!removeJoint(name))
    return false;

  joint->parent_link_name = parent_link;
  return addJointHelper(joint);
}

std::vector<std::shared_ptr<const Joint>> SceneGraph::getJoints() const
{
  std::vector<Joint::ConstPtr> joints;
  joints.reserve(joint_map_.size());
  for (const auto& joint : joint_map_)
    joints.push_back(joint.second.first);

  return joints;
}

std::vector<std::shared_ptr<const Joint>> SceneGraph::getActiveJoints() const
{
  std::vector<Joint::ConstPtr> joints;
  joints.reserve(joint_map_.size());
  for (const auto& joint : joint_map_)
  {
    if ((joint.second.first->type != JointType::FIXED) && (joint.second.first->type != JointType::FLOATING))
      joints.push_back(joint.second.first);
  }

  return joints;
}

bool SceneGraph::changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint origin with name (%s) which does not exist in scene graph.",
                           name.c_str());
    return false;
  }

  // Update transform associated with the joint
  Joint::Ptr joint = found->second.first;
  joint->parent_to_joint_origin_transform = new_origin;

  // Update the edge value associated with the joint
  Edge e = getEdge(name);
  double d = joint->parent_to_joint_origin_transform.translation().norm();
  boost::put(boost::edge_weight_t(), *this, e, d);

  return true;
}

bool SceneGraph::changeJointLimits(const std::string& name, const JointLimits& limits)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint limit with name (%s) which does not exist in scene graph.",
                           name.c_str());
    return false;
  }

  if (found->second.first->type == JointType::FIXED || found->second.first->type == JointType::FLOATING)
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint limits for a fixed or floating joint type.", name.c_str());
    return false;
  }

  if (found->second.first->limits == nullptr)
    found->second.first->limits = std::make_shared<JointLimits>();

  found->second.first->limits->lower = limits.lower;
  found->second.first->limits->upper = limits.upper;
  found->second.first->limits->effort = limits.effort;
  found->second.first->limits->velocity = limits.velocity;
  found->second.first->limits->acceleration = limits.acceleration;
  found->second.first->limits->jerk = limits.jerk;

  return true;
}

bool SceneGraph::changeJointPositionLimits(const std::string& name, double lower, double upper)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Position limits with name (%s) which does not exist in scene graph.",
                           name.c_str());
    return false;
  }

  if (found->second.first->type == JointType::FIXED || found->second.first->type == JointType::FLOATING)
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Position limits for a fixed or floating joint type.", name.c_str());
    return false;
  }

  found->second.first->limits->lower = lower;
  found->second.first->limits->upper = upper;

  return true;
}

bool SceneGraph::changeJointVelocityLimits(const std::string& name, double limit)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Velocity limit with name (%s) which does not exist in scene graph.",
                           name.c_str());
    return false;
  }

  if (found->second.first->type == JointType::FIXED || found->second.first->type == JointType::FLOATING)
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Velocity limit for a fixed or floating joint type.", name.c_str());
    return false;
  }

  found->second.first->limits->velocity = limit;
  return true;
}

bool SceneGraph::changeJointAccelerationLimits(const std::string& name, double limit)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Acceleration limit with name (%s) which does not exist in scene "
                           "graph.",
                           name.c_str());
    return false;
  }

  if (found->second.first->type == JointType::FIXED || found->second.first->type == JointType::FLOATING)
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Acceleration limit for a fixed or floating joint type.",
                           name.c_str());
    return false;
  }

  if (found->second.first->limits == nullptr)
    found->second.first->limits = std::make_shared<JointLimits>();

  found->second.first->limits->acceleration = limit;

  return true;
}

bool SceneGraph::changeJointJerkLimits(const std::string& name, double limit)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Jerk limit with name (%s) which does not exist in scene "
                           "graph.",
                           name.c_str());
    return false;
  }

  if (found->second.first->type == JointType::FIXED || found->second.first->type == JointType::FLOATING)
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint Jerk limit for a fixed or floating joint type.", name.c_str());
    return false;
  }

  if (found->second.first->limits == nullptr)
    found->second.first->limits = std::make_shared<JointLimits>();

  found->second.first->limits->jerk = limit;

  return true;
}

std::shared_ptr<const JointLimits> SceneGraph::getJointLimits(const std::string& name)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("SceneGraph::getJointLimits tried to find Joint with name (%s) which does not exist in "
                           "scene graph.",
                           name.c_str());
    return nullptr;
  }
  return found->second.first->limits;
}

void SceneGraph::setAllowedCollisionMatrix(std::shared_ptr<tesseract::common::AllowedCollisionMatrix> acm)
{
  acm_ = std::move(acm);
}

void SceneGraph::addAllowedCollision(const std::string& link_name1,
                                     const std::string& link_name2,
                                     const std::string& reason)
{
  acm_->addAllowedCollision(link_name1, link_name2, reason);
}

void SceneGraph::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  acm_->removeAllowedCollision(link_name1, link_name2);
}

void SceneGraph::removeAllowedCollision(const std::string& link_name) { acm_->removeAllowedCollision(link_name); }

void SceneGraph::clearAllowedCollisions() { acm_->clearAllowedCollisions(); }

bool SceneGraph::isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
{
  return acm_->isCollisionAllowed(link_name1, link_name2);
}

std::shared_ptr<const tesseract::common::AllowedCollisionMatrix> SceneGraph::getAllowedCollisionMatrix() const
{
  return acm_;
}

std::shared_ptr<tesseract::common::AllowedCollisionMatrix> SceneGraph::getAllowedCollisionMatrix() { return acm_; }

std::shared_ptr<const Link> SceneGraph::getSourceLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::source(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

std::shared_ptr<const Link> SceneGraph::getTargetLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::target(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

std::vector<std::shared_ptr<const Joint>> SceneGraph::getInboundJoints(const std::string& link_name) const
{
  std::vector<Joint::ConstPtr> joints;
  Vertex vertex = getVertex(link_name);

  // Get incoming edges
  auto num_in_edges = static_cast<int>(boost::in_degree(vertex, *this));
  if (num_in_edges == 0)  // The root of the tree will have not incoming edges
    return joints;

  boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::in_edges(vertex, *this); ei != ei_end; ++ei)
  {
    SceneGraph::Edge e = *ei;
    joints.push_back(boost::get(boost::edge_joint, *this)[e]);
  }

  return joints;
}

std::vector<std::shared_ptr<const Joint>> SceneGraph::getOutboundJoints(const std::string& link_name) const
{
  std::vector<Joint::ConstPtr> joints;
  Vertex vertex = getVertex(link_name);

  // Get incoming edges
  auto num_out_edges = static_cast<int>(boost::out_degree(vertex, *this));
  if (num_out_edges == 0)
    return joints;

  boost::graph_traits<Graph>::out_edge_iterator eo, eo_end;
  for (boost::tie(eo, eo_end) = boost::out_edges(vertex, *this); eo != eo_end; ++eo)
  {
    SceneGraph::Edge e = *eo;
    joints.push_back(boost::get(boost::edge_joint, *this)[e]);
  }

  return joints;
}

bool SceneGraph::isEmpty() const { return link_map_.empty(); }

bool SceneGraph::isAcyclic() const
{
  const auto& graph = static_cast<const Graph&>(*this);
  bool acyclic = true;

  std::map<Vertex, size_t> index_map;
  boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  cycle_detector vis(acyclic);
  boost::depth_first_search(static_cast<const Graph&>(*this), boost::visitor(vis).vertex_index_map(prop_index_map));
  return acyclic;
}

bool SceneGraph::isTree() const
{
  const auto& graph = static_cast<const Graph&>(*this);
  bool tree = true;

  std::map<Vertex, size_t> index_map;
  boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  tree_detector vis(tree);
  boost::depth_first_search(static_cast<const Graph&>(*this), boost::visitor(vis).vertex_index_map(prop_index_map));
  return tree;
}

std::vector<std::string> SceneGraph::getAdjacentLinkNames(const std::string& name) const
{
  std::vector<std::string> link_names;
  Vertex v = getVertex(name);
  for (auto* vd : boost::make_iterator_range(adjacent_vertices(v, *this)))
    link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

  return link_names;
}

std::vector<std::string> SceneGraph::getInvAdjacentLinkNames(const std::string& name) const
{
  std::vector<std::string> link_names;
  Vertex v = getVertex(name);
  for (auto* vd : boost::make_iterator_range(inv_adjacent_vertices(v, *this)))
    link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

  return link_names;
}

std::vector<std::string> SceneGraph::getLinkChildrenNames(const std::string& name) const
{
  Vertex v = getVertex(name);
  std::vector<std::string> child_link_names = getLinkChildrenHelper(v);

  // This always includes the start vertex, so must remove
  child_link_names.erase(child_link_names.begin());
  return child_link_names;
}

std::vector<std::string> SceneGraph::getJointChildrenNames(const std::string& name) const
{
  const auto& graph = static_cast<const Graph&>(*this);
  Edge e = getEdge(name);
  Vertex v = boost::target(e, graph);
  return getLinkChildrenHelper(v);  // NOLINT
}

std::vector<std::string> SceneGraph::getJointChildrenNames(const std::vector<std::string>& names) const
{
  std::set<std::string> link_names;
  for (const auto& name : names)
  {
    std::vector<std::string> joint_children = getJointChildrenNames(name);
    link_names.insert(joint_children.begin(), joint_children.end());
  }

  return std::vector<std::string>{ link_names.begin(), link_names.end() };
}

std::unordered_map<std::string, std::string>
SceneGraph::getAdjacencyMap(const std::vector<std::string>& link_names) const
{
  std::map<Vertex, size_t> index_map;
  boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

  std::map<Vertex, boost::default_color_type> color_map;
  boost::associative_property_map<std::map<Vertex, boost::default_color_type>> prop_color_map(color_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(*this); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  std::unordered_map<std::string, std::string> adjacency_map;
  for (const auto& link_name : link_names)
  {
    Vertex start_vertex = getVertex(link_name);
    adjacency_detector vis(adjacency_map, color_map, link_name, link_names);

    // NOLINTNEXTLINE
    boost::breadth_first_search(
        *this,
        start_vertex,
        boost::visitor(vis).root_vertex(start_vertex).vertex_index_map(prop_index_map).color_map(prop_color_map));
  }

  return adjacency_map;
}

void SceneGraph::saveDOT(const std::string& path) const
{
  std::ofstream dot_file(path);
  if (!dot_file.is_open())
  {
    throw std::runtime_error("Failed to open file: " + path);
  }

  dot_file << "digraph D {\n"
           << "  rankdir=LR\n"
           << "  size=\"4,3\"\n"
           << "  ratio=\"fill\"\n"
           << "  edge[style=\"bold\"]\n"
           << "  node[shape=\"circle\"]\n";

  const auto& graph = static_cast<const Graph&>(*this);
  Graph::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
  {
    Edge e = *ei;
    Vertex u = boost::source(e, graph);
    Vertex v = boost::target(e, graph);
    Joint::ConstPtr joint = boost::get(boost::edge_joint, graph)[e];

    dot_file << '"' << boost::get(boost::vertex_link, graph)[u]->getName() << '"' << " -> " << '"'
             << boost::get(boost::vertex_link, graph)[v]->getName() << '"' << "[label=\"" << joint->getName() << "\n("
             << joint->type << ")\", color=\"black\"]";
  }
  dot_file << "}";
}

ShortestPath SceneGraph::getShortestPath(const std::string& root, const std::string& tip) const
{
  // Must copy to undirected graph because order does not matter for creating kinematics chains.

  // Copy Graph
  UGraph graph;

  std::map<Graph::vertex_descriptor, size_t> index_map;
  boost::associative_property_map<std::map<Graph::vertex_descriptor, size_t>> prop_index_map(index_map);

  {
    int c = 0;
    Graph::vertex_iterator i, iend;
    for (boost::tie(i, iend) = boost::vertices(*this); i != iend; ++i, ++c)
      boost::put(prop_index_map, *i, c);
  }

  ugraph_vertex_copier v_copier(*this, graph);
  boost::copy_graph(*this, graph, boost::vertex_index_map(prop_index_map).vertex_copy(v_copier));

  // Search Graph
  UGraph::vertex_descriptor s_root = getVertex(root);
  UGraph::vertex_descriptor s_tip = getVertex(tip);

  auto prop_weight_map = boost::get(boost::edge_weight, graph);

  std::map<UGraph::vertex_descriptor, UGraph::vertex_descriptor> predicessor_map;
  boost::associative_property_map<std::map<UGraph::vertex_descriptor, UGraph::vertex_descriptor>> prop_predicessor_map(
      predicessor_map);

  std::map<UGraph::vertex_descriptor, double> distance_map;
  boost::associative_property_map<std::map<UGraph::vertex_descriptor, double>> prop_distance_map(distance_map);

  std::map<UGraph::vertex_descriptor, size_t> u_index_map;
  boost::associative_property_map<std::map<UGraph::vertex_descriptor, size_t>> u_prop_index_map(u_index_map);

  {  // Populate index map
    int c = 0;
    UGraph::vertex_iterator i, iend;
    for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
    {
      std::string name = boost::get(boost::vertex_link, graph)[*i]->getName();
      if (name == root)
        s_root = *i;

      if (name == tip)
        s_tip = *i;

      boost::put(u_prop_index_map, *i, c);
    }
  }

  dijkstra_shortest_paths(graph,
                          s_root,
                          prop_predicessor_map,
                          prop_distance_map,
                          prop_weight_map,
                          u_prop_index_map,
                          std::less<>(),
                          boost::closed_plus<double>(),
                          (std::numeric_limits<double>::max)(),
                          0,
                          boost::default_dijkstra_visitor());

  ShortestPath path;
  path.links.reserve(predicessor_map.size());
  path.joints.reserve(predicessor_map.size());
  path.active_joints.reserve(predicessor_map.size());
  // We want to start at the destination and work our way back to the source
  for (Vertex u = predicessor_map[s_tip];      // Start by setting 'u' to the destination node's predecessor
       u != s_tip;                             // Keep tracking the path until we get to the source
       s_tip = u, u = predicessor_map[s_tip])  // Set the current vertex to the current predecessor, and the predecessor
                                               // to one level up
  {
    path.links.push_back(boost::get(boost::vertex_link, graph)[s_tip]->getName());
    const Joint::ConstPtr& joint = boost::get(boost::edge_joint, graph)[boost::edge(u, s_tip, graph).first];

    path.joints.push_back(joint->getName());
    if (joint->type != JointType::FIXED && joint->type != JointType::FLOATING)
      path.active_joints.push_back(joint->getName());
  }
  path.links.push_back(root);
  std::reverse(path.links.begin(), path.links.end());
  std::reverse(path.joints.begin(), path.joints.end());
  std::reverse(path.active_joints.begin(), path.active_joints.end());

#ifndef NDEBUG
  CONSOLE_BRIDGE_logDebug("distances and parents:");
  UGraph::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi)
  {
    CONSOLE_BRIDGE_logDebug("distance(%s) = %f, parent(%s) = %s",
                            boost::get(boost::vertex_link, graph)[*vi]->getName().c_str(),
                            distance_map[*vi],
                            boost::get(boost::vertex_link, graph)[*vi]->getName().c_str(),
                            boost::get(boost::vertex_link, graph)[predicessor_map[*vi]]->getName().c_str());
  }
#endif
  return path;
}

SceneGraph::Vertex SceneGraph::getVertex(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    throw std::runtime_error("SceneGraph, vertex with name '" + name + "' does not exist!");

  return found->second.second;
}

SceneGraph::Edge SceneGraph::getEdge(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    throw std::runtime_error("SceneGraph, edge with name '" + name + "' does not exist!");

  return found->second.second;
}

/** addSceneGraph needs a couple helpers to handle prefixing, we hide them in an anonymous namespace here **/
namespace
{
tesseract::scene_graph::Link clone_prefix(const tesseract::scene_graph::Link::ConstPtr& link, const std::string& prefix)
{
  return link->clone(prefix + link->getName());
}

tesseract::scene_graph::Joint clone_prefix(const tesseract::scene_graph::Joint::ConstPtr& joint,
                                           const std::string& prefix)
{
  auto ret = joint->clone(prefix + joint->getName());
  ret.child_link_name = prefix + joint->child_link_name;
  ret.parent_link_name = prefix + joint->parent_link_name;
  return ret;
}

tesseract::common::AllowedCollisionMatrix::Ptr
clone_prefix(const tesseract::common::AllowedCollisionMatrix::ConstPtr& acm, const std::string& prefix)
{
  if (prefix.empty())
    return std::make_shared<tesseract::common::AllowedCollisionMatrix>(*acm);

  auto new_acm = std::make_shared<tesseract::common::AllowedCollisionMatrix>();
  for (const auto& entry : acm->getAllAllowedCollisions())
    new_acm->addAllowedCollision(prefix + entry.first.first, prefix + entry.first.second, entry.second);

  return new_acm;
}
}  // namespace

bool SceneGraph::insertSceneGraph(const tesseract::scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  bool is_empty = isEmpty();

  // Verify that link names are unique
  for (const auto& link : scene_graph.getLinks())
  {
    if (link_map_.find(prefix + link->getName()) != link_map_.end())
    {
      CONSOLE_BRIDGE_logError("Failed to add inserted graph, link names are not unique: %s",
                              (prefix + link->getName()).c_str());
      return false;
    }
  }

  // Verify that joint names are unique
  for (const auto& joint : scene_graph.getJoints())
  {
    if (joint_map_.find(prefix + joint->getName()) != joint_map_.end())
    {
      CONSOLE_BRIDGE_logError("Failed to add inserted graph, joint names are not unique: %s",
                              (prefix + joint->getName()).c_str());
      return false;
    }
  }

  for (const auto& link : scene_graph.getLinks())
  {
    auto new_link = std::make_shared<Link>(clone_prefix(link, prefix));
    bool res = addLinkHelper(new_link);
    if (!res)
    {
      CONSOLE_BRIDGE_logError("Failed to add inserted graph link: %s", link->getName().c_str());
      return false;
    }

    // Set link collision enabled and visibility
    setLinkCollisionEnabled(new_link->getName(), scene_graph.getLinkCollisionEnabled(link->getName()));
    setLinkVisibility(new_link->getName(), scene_graph.getLinkVisibility(link->getName()));
  }

  for (const auto& joint : scene_graph.getJoints())
  {
    auto new_joint = std::make_shared<Joint>(clone_prefix(joint, prefix));
    bool res = addJointHelper(new_joint);
    if (!res)
    {
      CONSOLE_BRIDGE_logError("Failed to add inserted graph joint: %s", joint->getName().c_str());
      return false;
    }
  }

  acm_->insertAllowedCollisionMatrix(*clone_prefix(scene_graph.getAllowedCollisionMatrix(), prefix));

  // If the this graph was empty to start we will set the root link to the same as the inserted one.
  if (is_empty)
    setRoot(scene_graph.getRoot());

  return true;
}

bool SceneGraph::insertSceneGraph(const tesseract::scene_graph::SceneGraph& scene_graph,
                                  const Joint& joint,
                                  const std::string& prefix)
{
  std::string parent_link = joint.parent_link_name;
  std::string child_link = joint.child_link_name;

  // Assumes the joint already contains the prefix in the parent and child link names
  if (!prefix.empty())
    child_link.erase(0, prefix.length());

  if (getLink(parent_link) == nullptr || scene_graph.getLink(child_link) == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inserted graph, provided joint link names do not exist in inserted graph!");
    return false;
  }

  if (getJoint(joint.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inserted graph, provided joint name %s already exists!",
                            joint.getName().c_str());
    return false;
  }

  if (!insertSceneGraph(scene_graph, prefix))
    return false;

  return addJointHelper(std::make_shared<Joint>(joint.clone()));
}

void SceneGraph::rebuildLinkAndJointMaps()
{
  link_map_.clear();
  joint_map_.clear();

  {  // Rebuild link map
    Graph::vertex_iterator i, iend;
    for (boost::tie(i, iend) = boost::vertices(*this); i != iend; ++i)
    {
      Link::Ptr link = boost::get(boost::vertex_link, *this)[*i];
      link_map_[link->getName()] = std::make_pair(link, *i);
    }
  }

  {  // Rebuild joint map
    Graph::edge_iterator i, iend;
    for (boost::tie(i, iend) = boost::edges(*this); i != iend; ++i)
    {
      Joint::Ptr joint = boost::get(boost::edge_joint, *this)[*i];
      joint_map_[joint->getName()] = std::make_pair(joint, *i);
    }
  }
}

std::vector<std::string> SceneGraph::getLinkChildrenHelper(Vertex start_vertex) const
{
  const auto& graph = static_cast<const Graph&>(*this);
  std::vector<std::string> child_link_names;

  std::map<Vertex, size_t> index_map;
  boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

  std::map<Vertex, boost::default_color_type> color_map;
  boost::associative_property_map<std::map<Vertex, boost::default_color_type>> prop_color_map(color_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  children_detector vis(child_link_names);
  // NOLINTNEXTLINE
  boost::breadth_first_search(
      graph,
      start_vertex,
      boost::visitor(vis).root_vertex(start_vertex).vertex_index_map(prop_index_map).color_map(prop_color_map));

  return child_link_names;
}

bool SceneGraph::operator==(const SceneGraph& rhs) const
{
  using namespace tesseract::common;
  // Currently these only compare the Link/Joint.
  auto link_pair_equal = [](const std::pair<const Link::Ptr, Vertex>& v1, const std::pair<Link::Ptr, Vertex>& v2) {
    return pointersEqual(v1.first, v2.first);
  };
  auto joint_pair_equal = [](const std::pair<const Joint::Ptr, Edge>& v1, const std::pair<Joint::Ptr, Edge>& v2) {
    return pointersEqual(v1.first, v2.first);
  };

  bool equal = true;
  equal &= pointersEqual(acm_, rhs.acm_);
  equal &= isIdenticalMap<std::unordered_map<std::string, std::pair<Link::Ptr, Vertex>>, std::pair<Link::Ptr, Vertex>>(
      link_map_, rhs.link_map_, link_pair_equal);
  equal &= isIdenticalMap<std::unordered_map<std::string, std::pair<Joint::Ptr, Edge>>, std::pair<Joint::Ptr, Edge>>(
      joint_map_, rhs.joint_map_, joint_pair_equal);

  return equal;
}
bool SceneGraph::operator!=(const SceneGraph& rhs) const { return !operator==(rhs); }

std::ostream& operator<<(std::ostream& os, const ShortestPath& path)
{
  os << "Links:" << "\n";
  for (const auto& l : path.links)
    os << "  " << l << "\n";

  os << "Joints:" << "\n";
  for (const auto& j : path.joints)
    os << "  " << j << "\n";

  os << "Active Joints:" << "\n";
  for (const auto& j : path.active_joints)
    os << "  " << j << "\n";
  return os;
}

}  // namespace tesseract::scene_graph
