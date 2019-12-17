/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen & Levi Armstrong */

#ifndef TESSERACT_SCENE_GRAPH_KDL_PARSER_H
#define TESSERACT_SCENE_GRAPH_KDL_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>

#include <kdl/tree.hpp>
#include <console_bridge/console.h>

#include <boost/utility.hpp>  // for boost::tie
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>

namespace tesseract_scene_graph
{
/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @return frame Output KDL Frame
 */
inline KDL::Frame convert(const Eigen::Isometry3d& transform)
{
  KDL::Frame frame;
  frame.Identity();

  for (int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);

  return frame;
}

/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @return frame Output KDL Frame
 */
inline KDL::Vector convert(const Eigen::Vector3d& vector) { return KDL::Vector(vector(0), vector(1), vector(2)); }

/**
 * @brief Convert Tesseract Joint to KDL Joint
 * @param joint Tesseract Joint
 * @return A KDL Joint
 */
inline KDL::Joint convert(const Joint::ConstPtr& joint)
{
  KDL::Frame parent_joint = convert(joint->parent_to_joint_origin_transform);
  const std::string& name = joint->getName();

  switch (joint->type)
  {
    case JointType::FIXED:
    {
      return KDL::Joint(name, KDL::Joint::None);
    }
    case JointType::REVOLUTE:
    {
      KDL::Vector axis = convert(joint->axis);
      return KDL::Joint(name, parent_joint.p, parent_joint.M * axis, KDL::Joint::RotAxis);
    }
    case JointType::CONTINUOUS:
    {
      KDL::Vector axis = convert(joint->axis);
      return KDL::Joint(name, parent_joint.p, parent_joint.M * axis, KDL::Joint::RotAxis);
    }
    case JointType::PRISMATIC:
    {
      KDL::Vector axis = convert(joint->axis);
      return KDL::Joint(name, parent_joint.p, parent_joint.M * axis, KDL::Joint::TransAxis);
    }
    default:
    {
      CONSOLE_BRIDGE_logWarn("Converting unknown joint type of joint '%s' into a fixed joint", name.c_str());
      return KDL::Joint(name, KDL::Joint::None);
    }
  }
}

/**
 * @brief Convert Tesseract Inertail to KDL Inertial
 * @param inertial
 * @return
 */
inline KDL::RigidBodyInertia convert(const Inertial::ConstPtr& inertial)
{
  KDL::Frame origin = convert(inertial->origin);

  // the mass is frame independent
  double kdl_mass = inertial->mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  KDL::Vector kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  KDL::RotationalInertia urdf_inertia =
      KDL::RotationalInertia(inertial->ixx, inertial->iyy, inertial->izz, inertial->ixy, inertial->ixz, inertial->iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
      origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  KDL::RotationalInertia kdl_inertia_wrt_com = kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

/**
 * @brief Everytime a vertex is visited for the first time add a new
 *        segment to the KDL Tree;
 */
struct kdl_tree_builder : public boost::dfs_visitor<>
{
  kdl_tree_builder(KDL::Tree& tree) : tree_(tree) {}

  template <class u, class g>
  void discover_vertex(u vertex, g graph)
  {
    const Link::ConstPtr& link = boost::get(boost::vertex_link, graph)[vertex];

    // constructs the optional inertia
    KDL::RigidBodyInertia inert(0);
    if (link->inertial)
      inert = convert(link->inertial);

    // Get incomming edges
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));
    if (num_in_edges == 0)  // The root of the tree will have not incoming edges
      return;

    boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    SceneGraph::Edge e = *ei;
    const Joint::ConstPtr& parent_joint = boost::get(boost::edge_joint, graph)[e];
    KDL::Joint kdl_jnt = convert(parent_joint);

    // construct the kdl segment
    KDL::Segment sgm(link->getName(), kdl_jnt, convert(parent_joint->parent_to_joint_origin_transform), inert);

    // add segment to tree
    tree_.addSegment(sgm, parent_joint->parent_link_name);
  }

protected:
  KDL::Tree& tree_;
};

/**
 * @brief Convert a Tesseract SceneGraph into a KDL Tree
 *        If graph is not a tree it will return false.
 * @param scene_graph The Tesseract Scene Graph
 * @param tree The KDL Tree to populate.
 * @return Returns Flase if error occured, otherwise true
 */
inline bool parseSceneGraph(const SceneGraph& scene_graph, KDL::Tree& tree)
{
  if (!scene_graph.isTree())
  {
    CONSOLE_BRIDGE_logError("Tesseract KDL Parser can only parse Scene Graphs that are trees");
    return false;
  }

  const std::string& root_name = scene_graph.getRoot();
  const Link::ConstPtr& root_link = scene_graph.getLink(root_name);

  tree = KDL::Tree(root_name);

  // warn if root link has inertia. KDL does not support this
  if (root_link->inertial)
  {
    CONSOLE_BRIDGE_logWarn("The root link %s has an inertia specified in the URDF, but KDL does not "
                           "support a root link with an inertia.  As a workaround, you can add an extra "
                           "dummy link to your URDF.",
                           root_name.c_str());
  }

  kdl_tree_builder builder(tree);

  std::map<SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<SceneGraph::Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(
      static_cast<const Graph&>(scene_graph),
      boost::visitor(builder).root_vertex(scene_graph.getVertex(root_name)).vertex_index_map(prop_index_map));
  return true;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_KDL_PARSER_H
