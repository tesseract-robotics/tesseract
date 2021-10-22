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
#include <list>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>

#ifdef SWIG

%shared_ptr(tesseract_scene_graph::SceneGraph)
%template(LinkVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link> >;
%template(JointVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint> >;
%template(LinkConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link const> >;
%template(JointConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint const> >;

#endif  // SWIG

#ifndef SWIG

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

#endif  // SWIG

namespace tesseract_scene_graph
{
#ifndef SWIG

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

#endif  // SWIG

/** @brief Holds the shortest path information.*/
struct ShortestPath
{
  /** @brief a list of links along the shortest path */
  std::vector<std::string> links;

  /** @brief A list of joints along the shortest path */
  std::vector<std::string> joints;

  /** @brief A list of active joints along the shortest path */
  std::vector<std::string> active_joints;
};

class SceneGraph
#ifndef SWIG
  : public Graph
#endif  // SWIG
{
public:
  using Vertex = SceneGraph::vertex_descriptor;
  using Edge = SceneGraph::edge_descriptor;

  using Ptr = std::shared_ptr<SceneGraph>;
  using ConstPtr = std::shared_ptr<const SceneGraph>;
  using UPtr = std::unique_ptr<SceneGraph>;
  using ConstUPtr = std::unique_ptr<const SceneGraph>;

  SceneGraph(const std::string& name = "");
  ~SceneGraph() = default;
  // SceneGraphs are non-copyable
  SceneGraph(const SceneGraph& other) = delete;
  SceneGraph& operator=(const SceneGraph& other) = delete;

  SceneGraph(SceneGraph&& other) = default;
  SceneGraph& operator=(SceneGraph&& other) = default;

  /**
   * @brief Clone the scene graph
   * @return The cloned scene graph
   */
  SceneGraph::UPtr clone() const;

  /** @brief Clear the scene graph */
  void clear();

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
   * @param replace_allowed If true and the link exist it will be replaced
   * @return Return False if a link with the same name already exists and replace is not allowed, otherwise true
   */
  bool addLink(const Link& link, bool replace_allowed = false);

  /**
   * @brief Adds a link/joint to the graph
   *
   * The first link added to the graph is set as the root by default. Use setRoot to change the root link of the graph.
   *
   * @param link The link to be added to the graph
   * @param joint The associated joint to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  bool addLink(const Link& link, const Joint& joint);

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
   * @brief Get a vector leaf links in the scene graph
   * @return A vector of links
   */
  std::vector<Link::ConstPtr> getLeafLinks() const;

  /**
   * @brief Removes a link from the graph
   *
   * Note: this will remove all inbound and outbound edges
   *
   * @param name Name of the link to be removed
   * @param recursive If true all children are removed if it only has a single joint
   * @return Return False if a link does not exists, otherwise true
   */
  bool removeLink(const std::string& name, bool recursive = false);

  /**
   * @brief Move link defined by provided joint
   * This deletes all inbound joints on the parent link defined by the joint
   * @param joint The joint defining the link move
   * @return Returns true if successful, otherwise false.
   */
  bool moveLink(const Joint& joint);

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
   * @param enabled True if should be considered during collision checking, otherwise false
   */
  void setLinkCollisionEnabled(const std::string& name, bool enabled);

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be considered during collision checking, otherwise false
   */
  bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Adds joint to the graph
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph,
   * otherwise true
   */
  bool addJoint(const Joint& joint);

  /**
   * @brief Get a joint in the graph
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
   */
  Joint::ConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Removes a joint from the graph
   * @param name Name of the joint to be removed
   * @param recursive If true all children are removed if this this is the only joint of the child link
   * @return Return False if a joint does not exists, otherwise true
   */
  bool removeJoint(const std::string& name, bool recursive = false);

  /**
   * @brief Move joint to new parent link
   * @param name Name of the joint to move
   * @param parent_link Name of parent link to move to
   * @return Returns true if successful, otherwise false.
   */
  bool moveJoint(const std::string& name, const std::string& parent_link);

  /**
   * @brief Get a vector of joints in the scene graph
   * @return A vector of joints
   */
  std::vector<Joint::ConstPtr> getJoints() const;

  /**
   * @brief Get a vector of active joints in the scene graph
   * @return A vector of active joints
   */
  std::vector<Joint::ConstPtr> getActiveJoints() const;

  /** @brief Changes the "origin" transform of the joint and recomputes the associated edge
   * @param name Name of the joint to be changed
   * @param new_origin The new transform associated with the joint
   * @return True if successful.
   */
  bool changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin);

  /**
   * @brief Changes the limits of a joint. The JointLimits::Ptr remains the same, but the values passed in are assigned
   * @param name Name of the joint to be changed
   * @param limits The new limits associated with the joint
   * @return True if successful.
   */
  bool changeJointLimits(const std::string& name, const JointLimits& limits);

  /**
   * @brief Changes the position limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New position limits to be set as the joint limits
   * @returnTrue if successful.
   */
  bool changeJointPositionLimits(const std::string& name, double lower, double upper);

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New velocity limits to be set as the joint limits
   * @return
   */
  bool changeJointVelocityLimits(const std::string& name, double limit);

  /**
   * @brief Changes the acceleration limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New acceleration limits to be set as the joint limits
   * @return
   */
  bool changeJointAccelerationLimits(const std::string& name, double limit);

  /**
   * @brief Gets the limits of the joint specified by name
   * @param name Name of the joint which limits will be retrieved
   * @return Limits of the joint. Returns nullptr is joint is not found.
   */
  JointLimits::ConstPtr getJointLimits(const std::string& name);

  /**
   * @brief Disable collision between two collision objects
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collision
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
   * link identified as the child link
   *
   * @param link_name The name of the link
   * @return Vector of joints
   */
  std::vector<Joint::ConstPtr> getInboundJoints(const std::string& link_name) const;

  /**
   * @brief Get outbound joints for a link
   *
   * The outbound joints are all joins that have the
   * link identified as the parent link
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
   * @brief Check if the graph is empty
   * @return True if empty, otherwise false
   */
  bool isEmpty() const;

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
   * @brief Get all children link names for a given joint name
   * @param name Name of joint
   * @return A vector of child link names
   */
  std::vector<std::string> getJointChildrenNames(const std::string& name) const;

  /**
   * @brief Create mapping between links in the scene to the provided links if they are directly affected if the link
   * moves
   * @param link_names The links to map other links to
   * @return A map of affected links to on of the provided link names.
   */
  std::unordered_map<std::string, std::string> getAdjacencyMap(const std::vector<std::string>& link_names) const;

  /**
   * @brief Get all children link names for the given joint names
   * @todo Need to create custom visitor so already process joint_names do not get processed again.
   * @param names Name of joints
   * @return A vector of child link names
   */
  std::vector<std::string> getJointChildrenNames(const std::vector<std::string>& names) const;

  /**
   * @brief Saves Graph as Graph Description Language (DOT)
   * @param path The file path
   */
  void saveDOT(const std::string& path) const;

  /**
   * @brief Get the shortest path between two links
   * @param root The base link
   * @param tip The tip link
   * @return The shortest path between the two links
   */
  ShortestPath getShortestPath(const std::string& root, const std::string& tip) const;

#ifndef SWIG
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
#endif
  /**
   * @brief Merge a graph into the current graph
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param prefix string Will prepend to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a sub-graph into the current environment, considering that the root of the merged graph is attached to the
   * root of the environment by a fixed joint and no displacement. Every joint and link of the sub-graph will be copied
   * into the environment graph. The prefix argument is meant to allow adding multiple copies of the same sub-graph with
   * different names
   */
  bool insertSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix = "");

  /**
   * @brief Merge a graph into the current environment
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param joint The joint that connects current environment with the inserted graph
   * @param prefix string Will prepend to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a sub-graph into the current environment. Every joint and link of the sub-graph will be copied into the
   * environment graph. The prefix argument is meant to allow adding multiple copies of the same sub-graph with
   * different names
   */
  bool insertSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph,
                        const tesseract_scene_graph::Joint& joint,
                        const std::string& prefix = "");

protected:
  /**
   * @brief Adds a link to the graph
   *
   * The first link added to the graph is set as the root by default. Use setRoot to change the root link of the graph.
   *
   * @param link_ptr Shared pointer to the link to be added to the graph
   * @param replace_allowed If true and the link exist it will be replaced
   * @return Return False if a link with the same name already exists and replace is not allowed, otherwise true
   */
  bool addLinkHelper(const Link::Ptr& link_ptr, bool replace_allowed = false);

  /**
   * @brief Adds joint to the graph
   * @param joint_ptr Shared pointer to the joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph,
   * otherwise true
   */
  bool addJointHelper(const Joint::Ptr& joint_ptr);

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
    bool& tree_;
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
    std::vector<std::string>& children_;
  };

  struct adjacency_detector : public boost::default_bfs_visitor
  {
    adjacency_detector(std::unordered_map<std::string, std::string>& adjacency_map,
                       std::map<Vertex, boost::default_color_type>& color_map,
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
    std::unordered_map<std::string, std::string>& adjacency_map_;
    std::map<Vertex, boost::default_color_type>& color_map_;
    const std::string& base_link_name_;
    const std::vector<std::string>& terminate_on_links_;
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
};

inline std::ostream& operator<<(std::ostream& os, const ShortestPath& path)
{
  os << "Links:" << std::endl;
  for (const auto& l : path.links)
    os << "  " << l << std::endl;

  os << "Joints:" << std::endl;
  for (const auto& j : path.joints)
    os << "  " << j << std::endl;

  os << "Active Joints:" << std::endl;
  for (const auto& j : path.active_joints)
    os << "  " << j << std::endl;
  return os;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_GRAPH_H
