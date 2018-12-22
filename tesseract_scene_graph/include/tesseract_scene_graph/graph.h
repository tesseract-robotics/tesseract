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
#ifndef TESSERACT_SCENE_GRAPH_H
#define TESSERACT_SCENE_GRAPH_H

#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/properties.hpp>
#include <string>
#include <unordered_map>
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

namespace tesseract
{
namespace graph
{

/** @brief Defines the boost graph property. */
typedef boost::property<boost::graph_name_t, std::string,
                        boost::property<boost::graph_root_t, std::string,
                        boost::property<boost::graph_link_map_t, std::unordered_map<std::string, std::pair<LinkConstPtr, long unsigned>>,
                        boost::property<boost::graph_joint_map_t, std::unordered_map<std::string, std::pair<JointConstPtr, boost::detail::edge_desc_impl<boost::bidirectional_tag, long unsigned>>>>>>> GraphProperty;

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
typedef std::shared_ptr<Graph> GraphPtr;
typedef std::shared_ptr<const Graph> GraphConstPtr;

typedef Graph::vertex_descriptor Vertex;
typedef Graph::edge_descriptor Edge;

/**
 * @brief Sets the graph name
 * @param name The name of the graph
 * @param graph The graph
 */
static inline void setName(const std::string& name, Graph& graph)
{
  boost::get_property(graph, boost::graph_name) = name;
}

/**
 * @brief Sets the graph name
 * @param name The name of the graph
 * @param graph The graph
 */
static inline const std::string& getName(const Graph& graph)
{
  return boost::get_property(graph, boost::graph_name);
}

/**
 * @brief Sets the root link name (aka. World Coordinate Frame)
 * @param name The name of the link
 * @param graph The graph
 * @return Return False if a link does not exists, otherwise true
 */
static inline bool setRoot(const std::string& name, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_link_map);
  auto found = map.find(name);

  if (found != map.end())
    return false;

  boost::get_property(graph, boost::graph_root) = name;
}

/**
 * @brief Gets the root link name (aka. World Coordinate Frame)
 * @param graph The graph
 * @return The root link name
 */
static inline const std::string& getRoot(const Graph& graph)
{
  return boost::get_property(graph, boost::graph_root);
}

/**
 * @brief Adds a link to the graph
 * @param link The link to be added to the graph
 * @param graph The graph
 * @return Return False if a link with the same name allready exists, otherwise true
 */
static inline bool addLink(LinkPtr link, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_link_map);
  auto found = map.find(link->getName());

  if (found != map.end())
    return false;

  VertexProperty info(link);
  Vertex v = boost::add_vertex(info, graph);
  map[link->getName()] = std::make_pair(link, v);
  return true;
}

/**
 * @brief Get a link in the graph
 * @param name The name of the link
 * @param graph The graph
 * @return Return nullptr if link name does not exists, otherwise a pointer to the link
 */
static inline LinkConstPtr getLink(const std::string& name, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_link_map);
  auto found = map.find(name);

  if (found == map.end())
    return nullptr;

  return found->second.first;
}

/**
 * @brief Removes a link from the graph
 * @param name Name of the link to be removed
 * @param graph The graph
 * @return Return False if a link does not exists, otherwise true
 */
static inline bool removeLink(const std::string& name, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_link_map);
  auto found = map.find(name);

  if (found == map.end())
    return false;

  boost::remove_vertex(found->second.second, graph);
  map.erase(name);

  return true;
}

/**
 * @brief Adds joint to the graph
 * @param joint The joint to be added
 * @param graph The graph
 * @return Return False if parent or child link does not exists and if joint name already exists in the graph, otherwise true
 */
static inline bool addJoint(JointPtr joint, Graph& graph)
{
  auto& link_map = boost::get_property(graph, boost::graph_link_map);
  auto& joint_map = boost::get_property(graph, boost::graph_joint_map);
  auto parent = link_map.find(joint->parent_link_name);
  auto child = link_map.find(joint->child_link_name);
  auto found = joint_map.find(joint->getName());

  if ( (parent == link_map.end()) || (child == link_map.end()) || (found != joint_map.end()) )
    return false;

  double d = joint->parent_to_joint_origin_transform.translation().norm();
  EdgeProperty info(joint, d);
  std::pair<Edge, bool> e = boost::add_edge(parent->second.second, child->second.second, info, graph);
  assert(e.second == true);
  joint_map[joint->getName()] = std::make_pair(joint, e.first);

  return true;
}

/**
 * @brief Get a joint in the graph
 * @param name The name of the joint
 * @param graph The graph
 * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
 */
static inline JointConstPtr getJoint(const std::string& name, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_joint_map);
  auto found = map.find(name);

  if (found == map.end())
    return nullptr;

  return found->second.first;
}

/**
 * @brief Removes a joint from the graph
 * @param name Name of the joint to be removed
 * @param graph The graph
 * @return Return False if a joint does not exists, otherwise true
 */
static inline bool removeJoint(const std::string& name, Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_joint_map);
  auto found = map.find(name);

  if (found == map.end())
    return false;

  boost::remove_edge(found->second.second, graph);
  map.erase(name);

  return true;
}

static inline Vertex getVertex(const std::string& name, const Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_link_map);
  auto found = map.find(name);

  if (found == map.end())
    return Vertex();

  return found->second.second;
}

static inline Edge getEdge(const std::string& name, const Graph& graph)
{
  auto& map = boost::get_property(graph, boost::graph_joint_map);
  auto found = map.find(name);

  if (found == map.end())
    return Edge();

  return found->second.second;
}

} // namespace graph

} // namespace tesseract
#endif // TESSERACT_SCENE_GRAPH_H
