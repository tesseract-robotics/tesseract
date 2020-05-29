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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/graph/adjacency_list.hpp>  // for customizable graphs
#include <boost/graph/directed_graph.hpp>  // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <string>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>

/* definition of basic boost::graph properties */
namespace boost
{
enum vertex_link_t
{
  vertex_link
};
enum vertex_link_visible_t
{
  vertex_link_visible
};
enum vertex_link_collision_enabled_t
{
  vertex_link_collision_enabled
};
enum edge_joint_t
{
  edge_joint
};
enum graph_root_t
{
  graph_root
};

BOOST_INSTALL_PROPERTY(vertex, link);
BOOST_INSTALL_PROPERTY(vertex, link_visible);
BOOST_INSTALL_PROPERTY(vertex, link_collision_enabled);
BOOST_INSTALL_PROPERTY(edge, joint);
BOOST_INSTALL_PROPERTY(graph, root);
}  // namespace boost

namespace tesseract_scene_graph
{
/** @brief Defines the boost graph property. */
using GraphProperty =
    boost::property<boost::graph_name_t, std::string, boost::property<boost::graph_root_t, std::string>>;

/** @brief Defines the boost graph vertex property. */
using VertexProperty = boost::property<
    boost::vertex_link_t,
    Link::Ptr,
    boost::property<boost::vertex_link_visible_t, bool, boost::property<boost::vertex_link_collision_enabled_t, bool>>>;

/**
 * @brief EdgeProperty
 *
 * The edge_weight represents the distance between the two links
 */
using EdgeProperty = boost::property<boost::edge_joint_t, Joint::Ptr, boost::property<boost::edge_weight_t, double>>;

using Graph = boost::
    adjacency_list<boost::listS, boost::listS, boost::bidirectionalS, VertexProperty, EdgeProperty, GraphProperty>;
class SceneGraph : public Graph
{
public:
  /**
   * @brief Holds the shortest path information.
   *
   * The first vector is a list of links along the shortest path
   * The second vector is a list of joints along the shortest path
   */
  using Path = std::pair<std::vector<std::string>, std::vector<std::string>>;
  using Vertex = SceneGraph::vertex_descriptor;
  using Edge = SceneGraph::edge_descriptor;

  using Ptr = std::shared_ptr<SceneGraph>;
  using ConstPtr = std::shared_ptr<const SceneGraph>;

  SceneGraph();
  ~SceneGraph() = default;
  // SceneGraphs are non-copyable
  SceneGraph(const SceneGraph& other) = delete;
  SceneGraph& operator=(const SceneGraph& other) = delete;

  SceneGraph(SceneGraph&& other) = default;
  SceneGraph& operator=(SceneGraph&& other) = default;

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
   *
   * The first link added to the graph is set as the root by default. Use setRoot to change the root link of the graph.
   *
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  bool addLink(Link link);

  /**
   * @brief Get a link in the graph
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  Link::ConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get a vector links in the scene graph
   * @return A vector of links
   */
  std::vector<Link::ConstPtr> getLinks() const;

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
  bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Set whether a link should be considered during collision checking
   * @param enabled True if should be condisdered during collision checking, otherwise false
   */
  void setLinkCollisionEnabled(const std::string& name, bool enabled);

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be condisdered during collision checking, otherwise false
   */
  bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Adds joint to the graph
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph,
   * otherwise true
   */
  bool addJoint(Joint joint);

  /**
   * @brief Get a joint in the graph
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
   */
  Joint::ConstPtr getJoint(const std::string& name) const;

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
  std::vector<Joint::ConstPtr> getJoints() const;

  /** @brief Changes the "origin" transform of the joint and recomputes the associated edge
   * @param name Name of the joint to be changed
   * @param new_origin The new transform associated with the joint
   * @return
   */
  bool changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin);

  /**
   * @brief Disable collision between two collision objects
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collison
   */
  void addAllowedCollision(const std::string& link_name1, const std::string& link_name2, const std::string& reason);

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   */
  void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  void removeAllowedCollision(const std::string& link_name);

  /** @brief Remove all allowed collisions */
  void clearAllowedCollisions();

  /**
   * @brief Check if two links are allowed to be in collision
   * @param link_name1 link name
   * @param link_name2 link name
   * @return True if the two links are allowed to be in collision, otherwise false
   */
  bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixPtr
   */
  AllowedCollisionMatrix::Ptr getAllowedCollisionMatrix();

  /**
   * @brief Get the source link (parent link) for a joint
   * @param joint_name The name of the joint
   * @return The source link
   */
  Link::ConstPtr getSourceLink(const std::string& joint_name) const;

  /**
   * @brief Get the target link (child link) for a joint
   * @param joint_name The name of the joint
   * @return The target link
   */
  Link::ConstPtr getTargetLink(const std::string& joint_name) const;

  /**
   * @brief Get inbound joints for a link
   *
   * The inbound joints are all joints that have the
   * link identified as the parent link
   *
   * @param link_name The name of the link
   * @return Vector of joints
   */
  std::vector<Joint::ConstPtr> getInboundJoints(const std::string& link_name) const;

  /**
   * @brief Get outbound joints for a link
   *
   * The outbound joints are all joins that have the
   * link identified as the child link
   *
   * @param link_name The name of the link
   * @return Vector of joints
   */
  std::vector<Joint::ConstPtr> getOutboundJoints(const std::string& link_name) const;

  /**
   * @brief Determine if the graph contains cycles
   * @return True if graph is acyclic (no cycles) otherwise false
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
  void saveDOT(const std::string& path) const;

  Path getShortestPath(const std::string& root, const std::string& tip);

  // static inline Graph copyGraph(const Graph& graph)
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

protected:
  /**
   * @brief Adds a link to the graph
   *
   * The first link added to the graph is set as the root by default. Use setRoot to change the root link of the graph.
   *
   * @param link_ptr Shared pointer to the link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  bool addLink(Link::Ptr link_ptr);

  /**
   * @brief Adds joint to the graph
   * @param joint_ptr Shared pointer to the joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph,
   * otherwise true
   */
  bool addJoint(Joint::Ptr joint_ptr);

private:
  std::unordered_map<std::string, std::pair<Link::Ptr, Vertex>> link_map_;
  std::unordered_map<std::string, std::pair<Joint::Ptr, Edge>> joint_map_;
  AllowedCollisionMatrix::Ptr acm_;

  struct cycle_detector : public boost::dfs_visitor<>
  {
    cycle_detector(bool& ascyclic) : ascyclic_(ascyclic) {}

    template <class e, class g>
    void back_edge(e, g&)
    {
      ascyclic_ = false;
    }

  protected:
    bool& ascyclic_;
  };

  struct tree_detector : public boost::dfs_visitor<>
  {
    tree_detector(bool& tree) : tree_(tree) {}

    template <class u, class g>
    void discover_vertex(u vertex, g graph)
    {
      auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));

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
    children_detector(std::vector<std::string>& children) : children_(children) {}

    template <class u, class g>
    void discover_vertex(u vertex, g graph)
    {
      children_.push_back(boost::get(boost::vertex_link, graph)[vertex]->getName());
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
    const auto& graph = static_cast<const Graph&>(*this);
    std::vector<std::string> child_link_names;

    std::map<Vertex, size_t> index_map;
    boost::associative_property_map<std::map<Vertex, size_t>> prop_index_map(index_map);

    int c = 0;
    Graph::vertex_iterator i, iend;
    for (boost::tie(i, iend) = boost::vertices(graph); i != iend; ++i, ++c)
      boost::put(prop_index_map, *i, c);

    children_detector vis(child_link_names);
    boost::breadth_first_search(
        graph, start_vertex, boost::visitor(vis).root_vertex(start_vertex).vertex_index_map(prop_index_map));

    return child_link_names;
  }
};

inline std::ostream& operator<<(std::ostream& os, const SceneGraph::Path& path)
{
  os << "Links:" << std::endl;
  for (const auto& l : path.first)
    os << "  " << l << std::endl;

  os << "Joints:" << std::endl;
  for (const auto& j : path.second)
    os << "  " << j << std::endl;

  return os;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_GRAPH_H
