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
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

/* definition of basic boost::graph properties */
namespace boost {
    enum vertex_link_t { vertex_link };
    enum edge_joint_t { edge_joint };
    enum graph_root_t { graph_root };
    enum graph_link_map_t { graph_link_map };
    enum graph_joint_map_t { graph_joint_map };

    BOOST_INSTALL_PROPERTY(vertex, link);
    BOOST_INSTALL_PROPERTY(edge, joint);
    BOOST_INSTALL_PROPERTY(graph, root);
    BOOST_INSTALL_PROPERTY(graph, link_map);
    BOOST_INSTALL_PROPERTY(graph, joint_map);
}

namespace tesseract_scene_graph
{

/** @brief Defines the boost graph property. */
typedef boost::property<boost::graph_name_t, std::string,
                        boost::property<boost::graph_root_t, std::string,
                        boost::property<boost::graph_link_map_t, std::unordered_map<std::string, std::pair<LinkPtr, long unsigned>>,
                        boost::property<boost::graph_joint_map_t, std::unordered_map<std::string, std::pair<JointPtr, boost::detail::edge_desc_impl<boost::bidirectional_tag, long unsigned>>>>>>> GraphProperty;

/** @brief Defines the boost graph vertex property. */
typedef boost::property<boost::vertex_link_t, LinkConstPtr> VertexProperty;

/**
 * @brief EdgeProperty
 *
 * The edge_weight represents the distance between the two links
 */
typedef boost::property<boost::edge_joint_t, JointConstPtr,
        boost::property<boost::edge_weight_t, double> > EdgeProperty;


typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperty, EdgeProperty, GraphProperty> Graph;
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
  void setName(const std::string& name)
  {
    boost::set_property(static_cast<Graph&>(*this), boost::graph_name, name);
  }

  /**
   * @brief Sets the graph name
   * @param name The name of the graph
   */
  const std::string& getName() const
  {
    return boost::get_property(static_cast<const Graph&>(*this), boost::graph_name);
  }

  /**
   * @brief Sets the root link name (aka. World Coordinate Frame)
   * @param name The name of the link
   * @return Return False if a link does not exists, otherwise true
   */
  bool setRoot(const std::string& name)
  {
    auto& map = get_property(static_cast<const Graph&>(*this), boost::graph_link_map);
    auto found = map.find(name);

    if (found == map.end())
      return false;

    boost::set_property(static_cast<Graph&>(*this), boost::graph_root, name);

    return true;
  }

  /**
   * @brief Gets the root link name (aka. World Coordinate Frame)
   * @return The root link name
   */
  const std::string& getRoot() const
  {
    return boost::get_property(static_cast<const Graph&>(*this), boost::graph_root);
  }

  /**
   * @brief Adds a link to the graph
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  bool addLink(LinkPtr link)
  {
    auto& map = boost::get_property(static_cast<Graph&>(*this), boost::graph_link_map);
    auto found = map.find(link->getName());

    if (found != map.end())
      return false;

    VertexProperty info(link);
    Vertex v = boost::add_vertex(info, static_cast<Graph&>(*this));
    map[link->getName()] = std::make_pair(link, v);
    return true;
  }

  /**
   * @brief Get a link in the graph
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  LinkConstPtr getLink(const std::string& name) const
  {
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_link_map);
    auto found = map.find(name);

    if (found == map.end())
      return nullptr;

    return found->second.first;
  }

  /**
   * @brief Get a vector links in the scene graph
   * @return A vector of links
   */
  std::vector<LinkConstPtr> getLinks() const
  {
    std::vector<LinkConstPtr> links;
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_link_map);

    links.reserve(map.size());
    for (const auto& link : map)
      links.push_back(link.second.first);

    return links;
  }

  /**
   * @brief Removes a link from the graph
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  bool removeLink(const std::string& name)
  {
    auto& map = boost::get_property(static_cast<Graph&>(*this), boost::graph_link_map);
    auto found = map.find(name);

    if (found == map.end())
      return false;

    boost::remove_vertex(found->second.second, static_cast<Graph&>(*this));
    map.erase(name);

    return true;
  }

  /**
   * @brief Adds joint to the graph
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph, otherwise true
   */
  bool addJoint(JointPtr joint)
  {
    auto& link_map = boost::get_property(static_cast<Graph&>(*this), boost::graph_link_map);
    auto& joint_map = boost::get_property(static_cast<Graph&>(*this), boost::graph_joint_map);
    auto parent = link_map.find(joint->parent_link_name);
    auto child = link_map.find(joint->child_link_name);
    auto found = joint_map.find(joint->getName());

    if ( (parent == link_map.end()) || (child == link_map.end()) || (found != joint_map.end()) )
      return false;

    double d = joint->parent_to_joint_origin_transform.translation().norm();
    EdgeProperty info(joint, d);
    std::pair<Edge, bool> e = boost::add_edge(parent->second.second, child->second.second, info, static_cast<Graph&>(*this));
    assert(e.second == true);
    joint_map[joint->getName()] = std::make_pair(joint, e.first);

    return true;
  }

  /**
   * @brief Get a joint in the graph
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
   */
  JointConstPtr getJoint(const std::string& name) const
  {
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_joint_map);
    auto found = map.find(name);

    if (found == map.end())
      return nullptr;

    return found->second.first;
  }

  /**
   * @brief Removes a joint from the graph
   * @param name Name of the joint to be removed
   * @return Return False if a joint does not exists, otherwise true
   */
  bool removeJoint(const std::string& name)
  {
    auto& map = boost::get_property(static_cast<Graph&>(*this), boost::graph_joint_map);
    auto found = map.find(name);

    if (found == map.end())
      return false;

    boost::remove_edge(found->second.second, static_cast<Graph&>(*this));
    map.erase(name);

    return true;
  }

  /**
   * @brief Move joint to new parent link
   * @param name Name of the joint to move
   * @param parent_link Name of parent link to move to
   * @return Returns true if successfull, otherwise false.
   */
  bool moveJoint(const std::string& name, const std::string& parent_link)
  {
    auto& map = boost::get_property(static_cast<Graph&>(*this), boost::graph_joint_map);
    auto found = map.find(name);

    if (found == map.end())
      return false;

    JointPtr joint = found->second.first;
    if (!removeJoint(name))
      return false;

    joint->parent_link_name = parent_link;
    return addJoint(joint);
  }

  /**
   * @brief Get a vector joints in the scene graph
   * @return A vector of joints
   */
  std::vector<JointConstPtr> getJoints() const
  {
    std::vector<JointConstPtr> joints;
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_joint_map);

    joints.reserve(map.size());
    for (const auto& joint : map)
      joints.push_back(joint.second.first);

    return joints;
  }

  LinkConstPtr getSourceLink(const std::string& joint_name) const
  {
    Edge e = getEdge(joint_name);
    Vertex v = boost::source(e, *this);
    return boost::get(boost::vertex_link, *this)[v];
  }

  LinkConstPtr getTargetLink(const std::string& joint_name) const
  {
    Edge e = getEdge(joint_name);
    Vertex v = boost::target(e, *this);
    return boost::get(boost::vertex_link, *this)[v];
  }

  std::vector<JointConstPtr> getInboundJoints(const std::string& link_name) const
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

  std::vector<JointConstPtr> getOutboundJoints(const std::string& link_name) const
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

  /**
   * @brief Determine if the graph contains cycles
   * @return True if graph is acyclic otherwise false
   */
  bool isAcyclic() const
  {
    bool acyclic = true;
    cycle_detector vis(acyclic);
    boost::depth_first_search(static_cast<const Graph&>(*this), boost::visitor(vis));
    return acyclic;
  }

  /**
   * @brief Determine if the graph is a tree
   * @return True if graph is tree otherwise false
   */
  bool isTree() const
  {
    bool tree = true;
    tree_detector vis(tree);
    boost::depth_first_search(static_cast<const Graph&>(*this), boost::visitor(vis));
    return tree;
  }

  std::vector<std::string> getAdjacentLinkNames(const std::string& name) const
  {
    std::vector<std::string> link_names;
    Vertex v = getVertex(name);
    for (auto vd : boost::make_iterator_range(adjacent_vertices(v, *this)))
      link_names.push_back(boost::get(boost::vertex_link, *this)[vd]->getName());

    return link_names;
  }

  /**
   * @brief Saves Graph as Graph Description Language (DOT)
   * @param path The file path
   */
  void saveDOT(std::string path)
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

  Path getShortestPath(const std::string& root, const std::string& tip)
  {
    const Graph& graph = static_cast<const Graph&>(*this);
    std::vector<Vertex> p(boost::num_vertices(graph));
    std::vector<double> d(boost::num_vertices(graph));
    Vertex s = getVertex(root);
    dijkstra_shortest_paths(graph, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));

    std::vector<std::string> links;
    std::vector<std::string> joints;
    Vertex v = getVertex(tip); // We want to start at the destination and work our way back to the source
    for(Vertex u = p[v]; // Start by setting 'u' to the destintaion node's predecessor
        u != v; // Keep tracking the path until we get to the source
        v = u, u = p[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
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

  /**
   * @brief Get the graph vertex by name
   * @param name The vertex/link name
   * @return Graph Vertex
   */
  Vertex getVertex(const std::string& name) const
  {
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_link_map);
    auto found = map.find(name);

    if (found == map.end())
      return Vertex();

    return found->second.second;
  }

  /**
   * @brief Get the graph edge by name
   * @param name The edge/joint name
   * @return Graph Edge
   */
  Edge getEdge(const std::string& name) const
  {
    auto& map = boost::get_property(static_cast<const Graph&>(*this), boost::graph_joint_map);
    auto found = map.find(name);

    if (found == map.end())
      return Edge();

    return found->second.second;
  }

private:

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

};
typedef std::shared_ptr<SceneGraph> SceneGraphPtr;
typedef std::shared_ptr<const SceneGraph> SceneGraphConstPtr;

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
