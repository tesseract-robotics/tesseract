/**
 * @file graph.h
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
#ifndef TESSERACT_SCENE_GRAPH_GRAPH_H
#define TESSERACT_SCENE_GRAPH_GRAPH_H

#include <tesseract_scene_graph/macros.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_PUSH
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/properties.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <fstream>
//#include <boost/graph/copy.hpp>
#include <string>
#include <unordered_map>
#include <console_bridge/console.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

/* definition of basic boost::graph properties */
namespace boost {
    enum vertex_link_t { vertex_link };
    enum vertex_link_visible_t { vertex_link_visible };
    enum vertex_link_collision_enabled_t { vertex_link_collision_enabled };
    enum edge_joint_t { edge_joint };
    enum graph_root_t { graph_root };

    BOOST_INSTALL_PROPERTY(vertex, link);
    BOOST_INSTALL_PROPERTY(vertex, link_visible);
    BOOST_INSTALL_PROPERTY(vertex, link_collision_enabled);
    BOOST_INSTALL_PROPERTY(edge, joint);
    BOOST_INSTALL_PROPERTY(graph, root);
}

namespace tesseract_scene_graph
{

/** @brief Defines the boost graph property. */
typedef boost::property<boost::graph_name_t, std::string,
                        boost::property<boost::graph_root_t, std::string>> GraphProperty;

/** @brief Defines the boost graph vertex property. */
typedef boost::property<boost::vertex_link_t, LinkPtr,
        boost::property<boost::vertex_link_visible_t, bool,
        boost::property<boost::vertex_link_collision_enabled_t, bool>>> VertexProperty;

/**
 * @brief EdgeProperty
 *
 * The edge_weight represents the distance between the two links
 */
typedef boost::property<boost::edge_joint_t, JointPtr,
        boost::property<boost::edge_weight_t, double> > EdgeProperty;


typedef boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS, VertexProperty, EdgeProperty, GraphProperty> Graph;
class SceneGraph : public Graph
{
public:

  /**
   * @brief Holds the shortest path information.
   *
   * The first vector is a list of links along the shortest path
   * The second vector is a list of joints along the shortest path
   */
  typedef std::pair<std::vector<std::string>, std::vector<std::string>> Path;
  typedef SceneGraph::vertex_descriptor Vertex;
  typedef SceneGraph::edge_descriptor Edge;

  /**
   * @brief Sets the graph name
   * @param name The name of the graph
   */
  void setName(const std::string& name);

  /**
   * @brief Sets the graph name
   * @param name The name of the graph
   */
  const std::string& getName() const;

  /**
   * @brief Sets the root link name (aka. World Coordinate Frame)
   * @param name The name of the link
   * @return Return False if a link does not exists, otherwise true
   */
  bool setRoot(const std::string& name);

  /**
   * @brief Gets the root link name (aka. World Coordinate Frame)
   * @return The root link name
   */
  const std::string& getRoot() const;

  /**
   * @brief Adds a link to the graph
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  bool addLink(LinkPtr link);

  /**
   * @brief Get a link in the graph
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  LinkConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get a vector links in the scene graph
   * @return A vector of links
   */
  std::vector<LinkConstPtr> getLinks() const;

  /**
   * @brief Removes a link from the graph
   *
   * Note: this will remove all inbound and outbound edges
   *
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  bool removeLink(const std::string& name);

  /**
   * @brief Set a links visibility
   * @param visibility True if should be visible, otherwise false
   */
  void setLinkVisibility(const std::string& name, bool visibility);

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  bool getLinkVisiblity(const std::string& name) const;

  /**
   * @brief Set whether a link should be considered during collision checking
   * @param visibility True if should be condisdered during collision checking, otherwise false
   */
  void setLinkCollisionVisibility(const std::string& name, bool visibility);

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be condisdered during collision checking, otherwise false
   */
  bool getLinkCollisionVisibility(const std::string& name) const;

  /**
   * @brief Adds joint to the graph
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph, otherwise true
   */
  bool addJoint(JointPtr joint);

  /**
   * @brief Get a joint in the graph
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
   */
  JointConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Removes a joint from the graph
   * @param name Name of the joint to be removed
   * @return Return False if a joint does not exists, otherwise true
   */
  bool removeJoint(const std::string& name);

  /**
   * @brief Move joint to new parent link
   * @param name Name of the joint to move
   * @param parent_link Name of parent link to move to
   * @return Returns true if successfull, otherwise false.
   */
  bool moveJoint(const std::string& name, const std::string& parent_link);

  /**
   * @brief Get a vector joints in the scene graph
   * @return A vector of joints
   */
  std::vector<JointConstPtr> getJoints() const;

  LinkConstPtr getSourceLink(const std::string& joint_name) const;

  LinkConstPtr getTargetLink(const std::string& joint_name) const;

  std::vector<JointConstPtr> getInboundJoints(const std::string& link_name) const;

  std::vector<JointConstPtr> getOutboundJoints(const std::string& link_name) const;

  /**
   * @brief Determine if the graph contains cycles
   * @return True if graph is acyclic otherwise false
   */
  bool isAcyclic() const;

  /**
   * @brief Determine if the graph is a tree
   * @return True if graph is tree otherwise false
   */
  bool isTree() const;

  /**
   * @brief Get a vector of adjacent link names provided a link name
   * @param name Name of link
   * @return A vector of adjacent link names
   */
  std::vector<std::string> getAdjacentLinkNames(const std::string& name) const;

  /**
   * @brief Geta a vectpr pf inverse adjacent link names provided a link name
   * @param name
   * @return
   */
  std::vector<std::string> getInvAdjacentLinkNames(const std::string& name) const;

  /**
   * @brief Get all children for a given link name
   * @param name Name of Link
   * @return A vector of child link names
   */
  std::vector<std::string> getLinkChildrenNames(const std::string& name) const;

  /**
   * @brief Get all children for a given link name
   * @param name Name of Link
   * @return A vector of child link names
   */
  std::vector<std::string> getJointChildrenNames(const std::string& name) const;

  /**
   * @brief Saves Graph as Graph Description Language (DOT)
   * @param path The file path
   */
  void saveDOT(std::string path) const;

  Path getShortestPath(const std::string& root, const std::string& tip);

  //static inline Graph copyGraph(const Graph& graph)
  //{
  //  Graph new_graph;
  //  boost::copy_graph(graph, new_graph);
  //  return new_graph;
  //}

  /**
   * @brief Get the graph vertex by name
   * @param name The vertex/link name
   * @return Graph Vertex
   */
  Vertex getVertex(const std::string& name) const;

  /**
   * @brief Get the graph edge by name
   * @param name The edge/joint name
   * @return Graph Edge
   */
  Edge getEdge(const std::string& name) const;

private:

  std::unordered_map<std::string, std::pair<LinkPtr, Vertex>> link_map_;
  std::unordered_map<std::string, std::pair<JointPtr, Edge>> joint_map_;

  struct cycle_detector : public boost::dfs_visitor<>
  {
    cycle_detector( bool& ascyclic)
      : ascyclic_(ascyclic) { }

    template <class e, class g>
    void back_edge(e, g&) {
      ascyclic_ = false;
    }
  protected:
    bool& ascyclic_;
  };

  struct tree_detector : public boost::dfs_visitor<>
  {
    tree_detector( bool& tree)
      : tree_(tree) { }

    template <class u, class g>
    void discover_vertex(u vertex, g graph)
    {
      int num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));

      if (num_in_edges > 1)
      {
        tree_ = false;
        return;
      }

      // Check if not vertex is unused.
      if (num_in_edges == 0 && boost::out_degree(vertex, graph) == 0)
      {
        tree_ = false;
        return;
      }
    }

    template <class e, class g>
    void back_edge(e, g&)
    {
      tree_ = false;
    }
  protected:
    bool& tree_;
  };

  struct children_detector : public boost::default_bfs_visitor
  {
    children_detector(std::vector<std::string>& children)
      : children_(children) { }

    template <class u, class g>
    void discover_vertex(u vertex, g graph)
    {
      children_.push_back(boost::get(boost::vertex_link, graph)[vertex]->getName());
      return;
    }

  protected:
    std::vector<std::string>& children_;
  };

  /**
   * @brief Get the children of a vertex starting with start_vertex
   *
   * Note: This list will include the start vertex
   *
   * @param start_vertex The vertex to find childeren for.
   * @return A list of child link names including the start vertex
   */
  std::vector<std::string> getLinkChildrenHelper(Vertex start_vertex) const
  {
    const Graph& graph = static_cast<const Graph&>(*this);
    std::vector<std::string> child_link_names;

    std::map<Vertex, size_t> index_map;
    boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

    int c = 0;
    Graph::vertex_iterator i, iend;
    for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
      boost::put(prop_index_map, *i, c);

    children_detector vis(child_link_names);
    boost::breadth_first_search(graph, start_vertex, boost::visitor(vis).root_vertex(start_vertex).vertex_index_map(prop_index_map));

    return child_link_names;
  }

};
typedef std::shared_ptr<SceneGraph> SceneGraphPtr;
typedef std::shared_ptr<const SceneGraph> SceneGraphConstPtr;


///////////////////////////////////////////
/// Implementation of SceneGraph //////////
///////////////////////////////////////////

inline void SceneGraph::setName(const std::string& name)
{
  boost::set_property(static_cast<Graph&>(*this), boost::graph_name, name);
}

inline const std::string& SceneGraph::getName() const
{
  return boost::get_property(static_cast<const Graph&>(*this), boost::graph_name);
}

inline bool SceneGraph::setRoot(const std::string& name)
{
  auto found = link_map_.find(name);

  if (found == link_map_.end())
    return false;

  boost::set_property(static_cast<Graph&>(*this), boost::graph_root, name);

  return true;
}

inline const std::string& SceneGraph::getRoot() const
{
  return boost::get_property(static_cast<const Graph&>(*this), boost::graph_root);
}

inline bool SceneGraph::addLink(LinkPtr link)
{
  auto found = link_map_.find(link->getName());
  if (found != link_map_.end())
    return false;

  VertexProperty info(link);
  Vertex v = boost::add_vertex(info, static_cast<Graph&>(*this));
  link_map_[link->getName()] = std::make_pair(link, v);
  return true;
}

inline LinkConstPtr SceneGraph::getLink(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    return nullptr;

  return found->second.first;
}

inline std::vector<LinkConstPtr> SceneGraph::getLinks() const
{
  std::vector<LinkConstPtr> links;
  links.reserve(link_map_.size());
  for (const auto& link : link_map_)
    links.push_back(link.second.first);

  return links;
}

inline bool SceneGraph::removeLink(const std::string& name)
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) from scene graph that does not exist.", name);
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

  return true;
}

inline void SceneGraph::setLinkVisibility(const std::string& name, bool visibility)
{
  boost::property_map<Graph, boost::vertex_link_visible_t>::type param = get(boost::vertex_link_visible, static_cast<Graph&>(*this));
  param[getVertex(name)] = visibility;
}

inline bool SceneGraph::getLinkVisiblity(const std::string& name) const
{
  boost::property_map<Graph, boost::vertex_link_visible_t>::const_type param = get(boost::vertex_link_visible, static_cast<const Graph&>(*this));
  return param[getVertex(name)];
}

inline void SceneGraph::setLinkCollisionVisibility(const std::string& name, bool visibility)
{
  boost::property_map<Graph, boost::vertex_link_collision_enabled_t>::type param = get(boost::vertex_link_collision_enabled, static_cast<Graph&>(*this));
  param[getVertex(name)] = visibility;
}

inline bool SceneGraph::getLinkCollisionVisibility(const std::string& name) const
{
  boost::property_map<Graph, boost::vertex_link_collision_enabled_t>::const_type param = get(boost::vertex_link_collision_enabled, static_cast<const Graph&>(*this));
  return param[getVertex(name)];
}

inline bool SceneGraph::addJoint(JointPtr joint)
{
  auto parent = link_map_.find(joint->parent_link_name);
  auto child = link_map_.find(joint->child_link_name);
  auto found = joint_map_.find(joint->getName());

  if (parent == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Parent link (%s) does not exist in scene graph.", joint->parent_link_name);
    return false;
  }

  if (child == link_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Child link (%s) does not exist in scene graph.", joint->child_link_name);
    return false;
  }

  if (found != joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Joint with name (%s) already exists in scene graph.", joint->getName());
    return false;
  }

  double d = joint->parent_to_joint_origin_transform.translation().norm();
  EdgeProperty info(joint, d);
  std::pair<Edge, bool> e = boost::add_edge(parent->second.second, child->second.second, info, static_cast<Graph&>(*this));
  assert(e.second == true);
  joint_map_[joint->getName()] = std::make_pair(joint, e.first);

  return true;
}

inline JointConstPtr SceneGraph::getJoint(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return nullptr;

  return found->second.first;
}

inline bool SceneGraph::removeJoint(const std::string& name)
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return false;

  boost::remove_edge(found->second.second, static_cast<Graph&>(*this));
  joint_map_.erase(name);

  return true;
}

inline bool SceneGraph::moveJoint(const std::string& name, const std::string& parent_link)
{
  auto found = joint_map_.find(name);

  if (found == joint_map_.end())
  {
    CONSOLE_BRIDGE_logWarn("Tried to move Joint with name (%s) which does not exist in scene graph.", name);
    return false;
  }

  JointPtr joint = found->second.first;
  if (!removeJoint(name))
    return false;

  joint->parent_link_name = parent_link;
  return addJoint(joint);
}

inline std::vector<JointConstPtr> SceneGraph::getJoints() const
{
  std::vector<JointConstPtr> joints;
  joints.reserve(joint_map_.size());
  for (const auto& joint : joint_map_)
    joints.push_back(joint.second.first);

  return joints;
}

inline LinkConstPtr SceneGraph::getSourceLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::source(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

inline LinkConstPtr SceneGraph::getTargetLink(const std::string& joint_name) const
{
  Edge e = getEdge(joint_name);
  Vertex v = boost::target(e, *this);
  return boost::get(boost::vertex_link, *this)[v];
}

inline std::vector<JointConstPtr> SceneGraph::getInboundJoints(const std::string& link_name) const
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

inline std::vector<JointConstPtr> SceneGraph::getOutboundJoints(const std::string& link_name) const
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

inline bool SceneGraph::isAcyclic() const
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

inline bool SceneGraph::isTree() const
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

inline std::vector<std::string> SceneGraph::getAdjacentLinkNames(const std::string& name) const
{
  std::vector<std::string> link_names;
  Vertex v = getVertex(name);
  for (auto vd : boost::make_iterator_range(adjacent_vertices(v, *this)))
    link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

  return link_names;
}

inline std::vector<std::string> SceneGraph::getInvAdjacentLinkNames(const std::string& name) const
{
  std::vector<std::string> link_names;
  Vertex v = getVertex(name);
  for (auto vd : boost::make_iterator_range(inv_adjacent_vertices(v, *this)))
    link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

  return link_names;
}

inline std::vector<std::string> SceneGraph::getLinkChildrenNames(const std::string& name) const
{
  Vertex v = getVertex(name);
  std::vector<std::string> child_link_names = getLinkChildrenHelper(v);

  // This always includes the start vertex, so must remove
  child_link_names.erase(child_link_names.begin());
  return child_link_names;
}

inline std::vector<std::string> SceneGraph::getJointChildrenNames(const std::string& name) const
{
  const Graph& graph = static_cast<const Graph&>(*this);
  Edge e = getEdge(name);
  Vertex v = boost::target(e, graph);
  return getLinkChildrenHelper(v);
}

inline void SceneGraph::saveDOT(std::string path) const
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

    dot_file << boost::get(boost::vertex_link, graph)[u]->getName()
             << " -> " << boost::get(boost::vertex_link, graph)[v]->getName()
             << "[label=\"" << joint->getName() << "\n(" << joint->type << ")\", color=\"black\"]";

  }
  dot_file << "}";
}

inline SceneGraph::Path SceneGraph::getShortestPath(const std::string& root, const std::string& tip)
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

inline SceneGraph::Vertex SceneGraph::getVertex(const std::string& name) const
{
  auto found = link_map_.find(name);
  if (found == link_map_.end())
    return Vertex();

  return found->second.second;
}

inline SceneGraph::Edge SceneGraph::getEdge(const std::string& name) const
{
  auto found = joint_map_.find(name);
  if (found == joint_map_.end())
    return Edge();

  return found->second.second;
}

inline std::ostream& operator<<(std::ostream& os, const SceneGraph::Path& path)
{
  os << "Links:" << std::endl;
  for (auto l : path.first)
    os << "  " << l << std::endl;

  os << "Joints:" << std::endl;
  for (auto j : path.second)
    os << "  " << j << std::endl;

  return os;
}

} // namespace tesseract_scene_graph

#endif // TESSERACT_SCENE_GRAPH_GRAPH_H
