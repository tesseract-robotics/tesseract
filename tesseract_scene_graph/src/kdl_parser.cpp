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

#include <tesseract_scene_graph/kdl_parser.h>

namespace tesseract_scene_graph
{
KDL::Frame convert(const Eigen::Isometry3d& transform)
{
  KDL::Frame frame;
  frame.Identity();

  for (int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);

  return frame;
}

Eigen::Isometry3d convert(const KDL::Frame& frame)
{
  Eigen::Isometry3d transform{ Eigen::Isometry3d::Identity() };

  // translation
  for (int i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (int i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];

  return transform;
}

KDL::Vector convert(const Eigen::Vector3d& vector) { return KDL::Vector{ vector(0), vector(1), vector(2) }; }

Eigen::Vector3d convert(const KDL::Vector& vector) { return Eigen::Vector3d{ vector(0), vector(1), vector(2) }; }

Eigen::MatrixXd convert(const KDL::Jacobian& jacobian) { return jacobian.data; }

KDL::Jacobian convert(const Eigen::MatrixXd& jacobian)
{
  if (jacobian.rows() != 6)
    throw std::runtime_error("Eigen Jacobian must have six rows!");

  KDL::Jacobian matrix;
  matrix.data = jacobian;

  return matrix;
}

Eigen::MatrixXd convert(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs)
{
  Eigen::MatrixXd matrix(jacobian.rows(), q_nrs.size());

  for (int j = 0; j < static_cast<int>(q_nrs.size()); ++j)
  {
    auto c = static_cast<unsigned>(q_nrs[static_cast<size_t>(j)]);
    matrix.col(j) = jacobian.data.col(c);
  }

  return matrix;
}

KDL::Joint convert(const Joint::ConstPtr& joint)
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
      return KDL::Joint{ name, parent_joint.p, parent_joint.M * axis, KDL::Joint::RotAxis };
    }
    case JointType::CONTINUOUS:
    {
      KDL::Vector axis = convert(joint->axis);
      return KDL::Joint{ name, parent_joint.p, parent_joint.M * axis, KDL::Joint::RotAxis };
    }
    case JointType::PRISMATIC:
    {
      KDL::Vector axis = convert(joint->axis);
      return KDL::Joint{ name, parent_joint.p, parent_joint.M * axis, KDL::Joint::TransAxis };
    }
    default:
    {
      CONSOLE_BRIDGE_logWarn("Converting unknown joint type of joint '%s' into a fixed joint", name.c_str());
      return KDL::Joint(name, KDL::Joint::None);
    }
  }
}

KDL::RigidBodyInertia convert(const Inertial::ConstPtr& inertial)
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
 * @brief Every time a vertex is visited for the first time add a new
 *        segment to the KDL Tree;
 */
struct kdl_tree_builder : public boost::dfs_visitor<>
{
  kdl_tree_builder(KDLTreeData& data) : data_(data) {}

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    const Link::ConstPtr& link = boost::get(boost::vertex_link, graph)[vertex];

    // constructs the optional inertia
    KDL::RigidBodyInertia inert(0);
    if (link->inertial)
      inert = convert(link->inertial);

    // Get incoming edges
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));
    if (num_in_edges == 0)  // The root of the tree will have not incoming edges
    {
      std::size_t num_v = boost::num_vertices(graph);
      std::size_t num_e = boost::num_edges(graph);
      data_.link_names.reserve(num_v);
      data_.active_link_names.reserve(num_v);
      data_.static_link_names.reserve(num_v);
      data_.joint_names.reserve(num_e);
      data_.active_joint_names.reserve(num_e);

      data_.link_names.push_back(link->getName());
      data_.static_link_names.push_back(link->getName());
      data_.base_link_name = link->getName();
      return;
    }

    data_.link_names.push_back(link->getName());

    boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    SceneGraph::Edge e = *ei;
    const Joint::ConstPtr& parent_joint = boost::get(boost::edge_joint, graph)[e];
    data_.joint_names.push_back(parent_joint->getName());

    KDL::Joint kdl_jnt = convert(parent_joint);
    if (kdl_jnt.getType() != KDL::Joint::None)
    {
      data_.active_joint_names.push_back(parent_joint->getName());
      data_.active_link_names.push_back(link->getName());
    }
    else
    {
      auto it =
          std::find(data_.active_link_names.begin(), data_.active_link_names.end(), parent_joint->parent_link_name);
      if (it != data_.active_link_names.end())
        data_.active_link_names.push_back(link->getName());
      else
        data_.static_link_names.push_back(link->getName());
    }

    // construct the kdl segment
    KDL::Segment sgm(link->getName(), kdl_jnt, convert(parent_joint->parent_to_joint_origin_transform), inert);

    // add segment to tree
    data_.tree.addSegment(sgm, parent_joint->parent_link_name);
  }

protected:
  KDLTreeData& data_;
};

/**
 * @brief Every time a vertex is visited for the first time add a new
 *        segment to the KDL Tree;
 */
struct kdl_sub_tree_builder : public boost::dfs_visitor<>
{
  kdl_sub_tree_builder(KDLTreeData& data,
                       const std::vector<std::string>& joint_names,
                       const std::unordered_map<std::string, double>& joint_values)
    : data_(data), joint_names_(joint_names), joint_values_(joint_values)
  {
  }

  template <class u, class g>
  void discover_vertex(u vertex, const g& graph)
  {
    const Link::ConstPtr& link = boost::get(boost::vertex_link, graph)[vertex];

    // constructs the optional inertia
    KDL::RigidBodyInertia inert(0);
    if (link->inertial)
      inert = convert(link->inertial);

    // Get incoming edges
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));
    if (num_in_edges == 0)  // The root of the tree will have not incoming edges
    {
      data_.base_link_name = link->getName();
      data_.tree = KDL::Tree(link->getName());
      segment_transforms_[link->getName()] = KDL::Frame::Identity();

      std::size_t num_v = boost::num_vertices(graph);
      std::size_t num_e = boost::num_edges(graph);
      data_.link_names.reserve(num_v);
      data_.static_link_names.reserve(num_v);
      data_.active_link_names.reserve(num_v);
      data_.active_joint_names.reserve(num_e);

      data_.link_names.push_back(link->getName());
      data_.static_link_names.push_back(link->getName());
      return;
    }

    boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    SceneGraph::Edge e = *ei;
    const Joint::ConstPtr& parent_joint = boost::get(boost::edge_joint, graph)[e];
    bool found = (std::find(joint_names_.begin(), joint_names_.end(), parent_joint->getName()) != joint_names_.end());
    KDL::Joint kdl_jnt = convert(parent_joint);
    KDL::Frame parent_to_joint = convert(parent_joint->parent_to_joint_origin_transform);
    KDL::Segment kdl_sgm(link->getName(), kdl_jnt, parent_to_joint, inert);
    std::string parent_link_name = parent_joint->parent_link_name;

    if (parent_joint->type != JointType::FIXED)
      segment_transforms_[parent_joint->child_link_name] =
          segment_transforms_[parent_link_name] * kdl_sgm.pose(joint_values_.at(parent_joint->getName()));
    else
      segment_transforms_[parent_joint->child_link_name] = segment_transforms_[parent_link_name] * kdl_sgm.pose(0.0);

    if (!started_ && found)
    {
      started_ = found;

      // construct the kdl segment to root if needed
      if (parent_link_name != data_.base_link_name)
      {
        std::string world_joint_name = data_.base_link_name + "_" + parent_link_name;
        KDL::Segment world_sgm = KDL::Segment(parent_link_name,
                                              KDL::Joint(world_joint_name, KDL::Joint::None),
                                              segment_transforms_[parent_link_name],
                                              KDL::RigidBodyInertia(0));
        data_.tree.addSegment(world_sgm, data_.base_link_name);
      }
      link_names_.push_back(parent_link_name);
      link_names_.push_back(link->getName());
      data_.static_link_names.push_back(parent_link_name);
      data_.link_names.push_back(parent_link_name);
      data_.link_names.push_back(link->getName());
      data_.active_link_names.push_back(link->getName());
      data_.active_joint_names.push_back(parent_joint->getName());

      // construct the kdl segment
      KDL::Segment sgm = KDL::Segment(link->getName(), kdl_jnt, parent_to_joint, inert);

      // add segment to tree
      data_.tree.addSegment(sgm, parent_link_name);
    }
    else if (started_)
    {
      auto it = std::find(link_names_.begin(), link_names_.end(), parent_link_name);

      if (it == link_names_.end() && !found)
        return;

      if (it == link_names_.end() && found)
      {
        data_.link_names.push_back(parent_link_name);
        link_names_.push_back(parent_link_name);
        data_.static_link_names.push_back(parent_link_name);

        KDL::Frame new_tree_parent_to_joint =
            segment_transforms_[data_.base_link_name].Inverse() * segment_transforms_[parent_joint->parent_link_name];

        // construct the kdl segment
        std::string new_joint_name = data_.base_link_name + "_to_" + parent_link_name + "_joint";
        KDL::Segment sgm = KDL::Segment(parent_link_name,
                                        KDL::Joint(new_joint_name, KDL::Joint::None),
                                        new_tree_parent_to_joint,
                                        KDL::RigidBodyInertia(0));

        // add segment to tree
        data_.tree.addSegment(sgm, data_.base_link_name);
      }
      else if (it != link_names_.end() && !found)
      {
        if (parent_joint->type != JointType::FIXED)
          parent_to_joint = kdl_sgm.pose(joint_values_.at(parent_joint->getName()));
        else
          parent_to_joint = kdl_sgm.pose(0.0);

        kdl_jnt = KDL::Joint(parent_joint->getName(), KDL::Joint::None);
      }

      data_.link_names.push_back(link->getName());
      link_names_.push_back(link->getName());

      auto active_it = std::find(data_.active_link_names.begin(), data_.active_link_names.end(), parent_link_name);
      if (active_it != data_.active_link_names.end() || kdl_jnt.getType() != KDL::Joint::None)
        data_.active_link_names.push_back(link->getName());
      else
        data_.static_link_names.push_back(link->getName());

      if (kdl_jnt.getType() != KDL::Joint::None)
        data_.active_joint_names.push_back(parent_joint->getName());

      // construct the kdl segment
      KDL::Segment sgm = KDL::Segment(link->getName(), kdl_jnt, parent_to_joint, inert);

      // add segment to tree
      data_.tree.addSegment(sgm, parent_link_name);
    }
  }

protected:
  KDLTreeData& data_;
  int search_cnt_{ -1 };
  bool started_{ false };
  std::map<std::string, KDL::Frame> segment_transforms_;
  std::vector<std::string> link_names_;

  const std::vector<std::string>& joint_names_;
  const std::unordered_map<std::string, double>& joint_values_;
};

KDLTreeData parseSceneGraph(const SceneGraph& scene_graph)
{
  if (!scene_graph.isTree())
    throw std::runtime_error("parseSubSceneGraph: currently only works if the scene graph is a tree.");

  const std::string& root_name = scene_graph.getRoot();
  const Link::ConstPtr& root_link = scene_graph.getLink(root_name);

  KDLTreeData data;
  data.tree = KDL::Tree(root_name);

  // warn if root link has inertia. KDL does not support this
  if (root_link->inertial)
  {
    CONSOLE_BRIDGE_logWarn("The root link %s has an inertia specified in the URDF, but KDL does not "
                           "support a root link with an inertia.  As a workaround, you can add an extra "
                           "dummy link to your URDF.",
                           root_name.c_str());
  }

  kdl_tree_builder builder(data);

  std::map<SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<SceneGraph::Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(
      static_cast<const Graph&>(scene_graph),
      boost::visitor(builder).root_vertex(scene_graph.getVertex(root_name)).vertex_index_map(prop_index_map));

  assert(data.link_names.size() == scene_graph.getLinks().size());
  assert(data.active_joint_names.size() <= scene_graph.getJoints().size());
  assert(data.active_link_names.size() < scene_graph.getLinks().size());
  return data;
}

KDLTreeData parseSceneGraph(const SceneGraph& scene_graph,
                            const std::vector<std::string>& joint_names,
                            const std::unordered_map<std::string, double>& joint_values)
{
  if (!scene_graph.isTree())
    throw std::runtime_error("parseSubSceneGraph: currently only works if the scene graph is a tree.");

  KDLTreeData data;
  data.tree = KDL::Tree(scene_graph.getRoot());

  kdl_sub_tree_builder builder(data, joint_names, joint_values);

  std::map<SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<SceneGraph::Vertex, size_t>> prop_index_map(index_map);

  int c = 0;
  Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(static_cast<const Graph&>(scene_graph),
                            boost::visitor(builder)
                                .root_vertex(scene_graph.getVertex(scene_graph.getRoot()))
                                .vertex_index_map(prop_index_map));

  if (data.tree.getNrOfJoints() != joint_names.size())
    throw std::runtime_error("parseSubSceneGraph: failed to generate sub-tree given the provided joint names.");

  return data;
}

}  // namespace tesseract_scene_graph
