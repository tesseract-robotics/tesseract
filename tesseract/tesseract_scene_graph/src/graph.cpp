/**
 * @file graph.cpp
 * @brief A basic scene graph using boost
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

#include <tesseract_scene_graph/macros.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_PUSH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <fstream>
#include <console_bridge/console.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>

namespace tesseract_scene_graph
{

SceneGraph::SceneGraph() : Graph(), acm_(std::make_shared<AllowedCollisionMatrix>())
{
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

bool SceneGraph::addLink(Link link)
{
  auto link_ptr = std::make_shared<tesseract_scene_graph::Link>(std::move(link));
  auto found = link_map_.find(link_ptr->getName());
  if (found != link_map_.end())
    return false;

  VertexProperty info(link_ptr);
  Vertex v = boost::add_vertex(info, static_cast<Graph&>(*this));
  link_map_[link_ptr->getName()] = std::make_pair(link_ptr, v);
  return true;
}

LinkConstPtr SceneGraph::getLink(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    return nullptr;

  return found->second.first;
}

std::vector<LinkConstPtr> SceneGraph::getLinks() const
{
  std::vector<LinkConstPtr> links;
  links.reserve(link_map_.size());
  for (const auto& link : link_map_)
    links.push_back(link.second.first);

  return links;
}

bool SceneGraph::removeLink(const std::string& name)
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) from scene graph that does not exist.", name.c_str());
    return false;
  }

  // Needt to remove all inbound and outbound edges first
  Vertex vertex = getVertex(name);
  boost::clear_vertex(vertex, *this);

  // rebuild joint_map
  joint_map_.clear();
  Graph::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(*this); ei != ei_end; ++ei)
  {
    Edge e = *ei;
    JointPtr joint = boost::get(boost::edge_joint, *this)[e];
    joint_map_[joint->getName()] = std::make_pair(joint, e);
  }

  // Now remove vertex
  boost::remove_vertex(found->second.second, static_cast<Graph&>(*this));
  link_map_.erase(name);

  // Need to remove any reference to link in allowed collision matrix
  removeAllowedCollision(name);

  return true;
}

void SceneGraph::setLinkVisibility(const std::string& name, bool visibility)
{
  boost::property_map<Graph, boost::vertex_link_visible_t>::type param = get(boost::vertex_link_visible, static_cast<Graph&>(*this));
  param[getVertex(name)] = visibility;
}

bool SceneGraph::getLinkVisibility(const std::string& name) const
{
  boost::property_map<Graph, boost::vertex_link_visible_t>::const_type param = get(boost::vertex_link_visible, static_cast<const Graph&>(*this));
  return param[getVertex(name)];
}

void SceneGraph::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  boost::property_map<Graph, boost::vertex_link_collision_enabled_t>::type param = get(boost::vertex_link_collision_enabled, static_cast<Graph&>(*this));
  param[getVertex(name)] = enabled;
}

bool SceneGraph::getLinkCollisionEnabled(const std::string& name) const
{
  boost::property_map<Graph, boost::vertex_link_collision_enabled_t>::const_type param = get(boost::vertex_link_collision_enabled, static_cast<const Graph&>(*this));
  return param[getVertex(name)];
}

bool SceneGraph::addJoint(tesseract_scene_graph::Joint joint)
{
  auto joint_ptr = std::make_shared<tesseract_scene_graph::Joint>(std::move(joint));
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

  double d = joint_ptr->parent_to_joint_origin_transform.translation().norm();

  EdgeProperty info(joint_ptr, d);
  std::pair<Edge, bool> e = boost::add_edge(parent->second.second, child->second.second, info, static_cast<Graph&>(*this));
  assert(e.second == true);
  joint_map_[joint_ptr->getName()] = std::make_pair(joint_ptr, e.first);

  return true;
}

JointConstPtr SceneGraph::getJoint(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return nullptr;

  return found->second.first;
}

bool SceneGraph::removeJoint(const std::string& name)
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return false;

  boost::remove_edge(found->second.second, static_cast<Graph&>(*this));
  joint_map_.erase(name);

  return true;
}

bool SceneGraph::moveJoint(const std::string& name, const std::string& parent_link)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move Joint with name (%s) which does not exist in scene graph.", name.c_str());
    return false;
  }

  JointPtr joint = found->second.first;
  if (!removeJoint(name))
    return false;

  joint->parent_link_name = parent_link;
  return addJoint(*joint);
}

std::vector<JointConstPtr> SceneGraph::getJoints() const
{
  std::vector<JointConstPtr> joints;
  joints.reserve(joint_map_.size());
  for (const auto& joint : joint_map_)
    joints.push_back(joint.second.first);

  return joints;
}

bool SceneGraph::changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to change Joint origin with name (%s) which does not exist in scene graph.", name.c_str());
    return false;
  }

  // Update transform associated with the joint
  JointPtr joint = found->second.first;
  joint->parent_to_joint_origin_transform = new_origin;

  // Update the edge value associated with the joint
  Edge e = getEdge(name);
  double d = joint->parent_to_joint_origin_transform.translation().norm();
  boost::put(boost::edge_weight_t(), *this, e, d);

  return true;
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

void SceneGraph::removeAllowedCollision(const std::string& link_name)
{
  acm_->removeAllowedCollision(link_name);
}

bool SceneGraph::isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const
{
  return acm_->isCollisionAllowed(link_name1, link_name2);
}

AllowedCollisionMatrixConstPtr SceneGraph::getAllowedCollisionMatrix() const
{
  return acm_;
}

LinkConstPtr SceneGraph::getSourceLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::source(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

LinkConstPtr SceneGraph::getTargetLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::target(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

std::vector<JointConstPtr> SceneGraph::getInboundJoints(const std::string& link_name) const
{
  std::vector<JointConstPtr> joints;
  Vertex vertex = getVertex(link_name);

  // Get incomming edges
  int num_in_edges = static_cast<int>(boost::in_degree(vertex, *this));
  if (num_in_edges == 0) // The root of the tree will have not incoming edges
    return joints;

  boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::in_edges(vertex, *this); ei != ei_end; ++ei)
  {
    SceneGraph::Edge e = *ei;
    joints.push_back(boost::get(boost::edge_joint, *this)[e]);
  }

  return joints;
}

std::vector<JointConstPtr> SceneGraph::getOutboundJoints(const std::string& link_name) const
{
  std::vector<JointConstPtr> joints;
  Vertex vertex = getVertex(link_name);

  // Get incomming edges
  int num_out_edges = static_cast<int>(boost::out_degree(vertex, *this));
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

bool SceneGraph::isAcyclic() const
{
  const Graph& graph = static_cast<const Graph&>(*this);
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
  const Graph& graph = static_cast<const Graph&>(*this);
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
  for (auto vd : boost::make_iterator_range(adjacent_vertices(v, *this)))
    link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

  return link_names;
}

std::vector<std::string> SceneGraph::getInvAdjacentLinkNames(const std::string& name) const
{
  std::vector<std::string> link_names;
  Vertex v = getVertex(name);
  for (auto vd : boost::make_iterator_range(inv_adjacent_vertices(v, *this)))
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
  const Graph& graph = static_cast<const Graph&>(*this);
  Edge e = getEdge(name);
  Vertex v = boost::target(e, graph);
  return getLinkChildrenHelper(v);
}

void SceneGraph::saveDOT(std::string path) const
{
  std::ofstream dot_file(path);

  dot_file << "digraph D {\n"
    << "  rankdir=LR\n"
    << "  size=\"4,3\"\n"
    << "  ratio=\"fill\"\n"
    << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

  const Graph& graph = static_cast<const Graph&>(*this);
  Graph::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
  {
    Edge e = *ei;
    Vertex u = boost::source(e, graph);
    Vertex v = boost::target(e, graph);
    JointConstPtr joint = boost::get(boost::edge_joint, graph)[e];

    dot_file << '"' << boost::get(boost::vertex_link, graph)[u]->getName() << '"'
             << " -> " << '"' << boost::get(boost::vertex_link, graph)[v]->getName() << '"'
             << "[label=\"" << joint->getName() << "\n(" << joint->type << ")\", color=\"black\"]";

  }
  dot_file << "}";
}

SceneGraph::Path SceneGraph::getShortestPath(const std::string& root, const std::string& tip)
{
  const Graph& graph = static_cast<const Graph&>(*this);
  Vertex s = getVertex(root);

  std::map<Vertex, Vertex> predicessor_map;
  boost::associative_property_map<std::map<Vertex, Vertex>> prop_predicessor_map(predicessor_map);

  std::map<Vertex, double> distance_map;
  boost::associative_property_map<std::map<Vertex, double>> prop_distance_map(distance_map);


  std::map<Vertex, size_t> index_map;
  boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  std::map<Edge, double> weight_map;
  boost::associative_property_map<std::map<Edge, double>> prop_weight_map(weight_map);
  Graph::edge_iterator j, jend;
  for (boost::tie(j, jend) = boost::edges(graph); j != jend; ++j)
    boost::put(prop_weight_map, *j, boost::get(boost::edge_weight, graph)[*j]);

  dijkstra_shortest_paths(graph, s, prop_predicessor_map, prop_distance_map, prop_weight_map, prop_index_map, std::less<double>(), boost::closed_plus<double>(),(std::numeric_limits<double>::max)(), 0, boost::default_dijkstra_visitor());

  std::vector<std::string> links;
  std::vector<std::string> joints;
  Vertex v = getVertex(tip); // We want to start at the destination and work our way back to the source
  for(Vertex u = predicessor_map[v]; // Start by setting 'u' to the destintaion node's predecessor
      u != v; // Keep tracking the path until we get to the source
      v = u, u = predicessor_map[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    links.push_back(boost::get(boost::vertex_link, graph)[v]->getName());
    joints.push_back(boost::get(boost::edge_joint, graph)[boost::edge(u, v, graph).first]->getName());
  }
  links.push_back(root);
  std::reverse(links.begin(), links.end());
  std::reverse(joints.begin(), joints.end());

#ifdef NDEBUG
  std::cout << "distances and parents:" << std::endl;
  Graph::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi)
  {
    std::cout << "distance(" << boost::get(boost::vertex_link, g)[*vi]->getName() << ") = " << d[*vi] << ", ";
    std::cout << "parent(" << boost::get(boost::vertex_link, g)[*vi]->getName() << ") = " << boost::get(boost::vertex_link, g)[p[*vi]]->getName() << std::endl;
  }
  std::cout << std::endl;
#endif
  return Path(links, joints);
}

//static inline Graph copyGraph(const Graph& graph)
//{
//  Graph new_graph;
//  boost::copy_graph(graph, new_graph);
//  return new_graph;
//}

SceneGraph::Vertex SceneGraph::getVertex(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    return Vertex();

  return found->second.second;
}

SceneGraph::Edge SceneGraph::getEdge(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return Edge();

  return found->second.second;
}

} // namespace tesseract_scene_graph
