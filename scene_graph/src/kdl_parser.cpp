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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/graph/depth_first_search.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/scene_graph/kdl_parser.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/utils.h>

namespace tesseract::scene_graph
{
using tesseract::common::JointId;
using tesseract::common::LinkId;
KDL::Frame convert(const Eigen::Isometry3d& transform)
{
  KDL::Frame frame;

  // translation
  frame.p[0] = transform(0, 3);
  frame.p[1] = transform(1, 3);
  frame.p[2] = transform(2, 3);

  // rotation matrix
  frame.M.data[0] = transform(0, 0);
  frame.M.data[1] = transform(0, 1);
  frame.M.data[2] = transform(0, 2);
  frame.M.data[3] = transform(1, 0);
  frame.M.data[4] = transform(1, 1);
  frame.M.data[5] = transform(1, 2);
  frame.M.data[6] = transform(2, 0);
  frame.M.data[7] = transform(2, 1);
  frame.M.data[8] = transform(2, 2);

  return frame;
}

Eigen::Isometry3d convert(const KDL::Frame& frame)
{
  Eigen::Isometry3d transform;

  // translation
  transform(0, 3) = frame.p[0];
  transform(1, 3) = frame.p[1];
  transform(2, 3) = frame.p[2];

  // rotation matrix
  transform(0, 0) = frame.M.data[0];
  transform(0, 1) = frame.M.data[1];
  transform(0, 2) = frame.M.data[2];
  transform(1, 0) = frame.M.data[3];
  transform(1, 1) = frame.M.data[4];
  transform(1, 2) = frame.M.data[5];
  transform(2, 0) = frame.M.data[6];
  transform(2, 1) = frame.M.data[7];
  transform(2, 2) = frame.M.data[8];

  // Bottom row
  transform(3, 0) = 0;
  transform(3, 1) = 0;
  transform(3, 2) = 0;
  transform(3, 3) = 1;

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

KDL::Joint convert(const std::shared_ptr<const Joint>& joint)
{
  KDL::Frame parent_joint = convert(joint->parent_to_joint_origin_transform);
  const std::string& name = joint->getName();

  switch (joint->type)
  {
    case JointType::FIXED:
    case JointType::FLOATING:
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

KDL::RigidBodyInertia convert(const std::shared_ptr<const Inertial>& inertial)
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

bool KDLTreeData::operator==(const KDLTreeData& rhs) const
{
  bool equal = true;
  equal &= (base_link_id == rhs.base_link_id);
  equal &= (joint_ids == rhs.joint_ids);
  equal &= (active_joint_ids == rhs.active_joint_ids);
  equal &= (floating_joint_ids == rhs.floating_joint_ids);
  equal &= (link_ids == rhs.link_ids);
  equal &= (active_link_ids == rhs.active_link_ids);
  equal &= (static_link_ids == rhs.static_link_ids);

  auto isometry_equal = [](const Eigen::Isometry3d& iso_1, const Eigen::Isometry3d& iso_2) {
    return iso_1.isApprox(iso_2, 1e-5);
  };
  equal &= tesseract::common::isIdenticalMap<tesseract::common::JointIdTransformMap, Eigen::Isometry3d>(
      floating_joint_values, rhs.floating_joint_values, isometry_equal);

  return equal;
}

bool KDLTreeData::operator!=(const KDLTreeData& rhs) const { return !operator==(rhs); }

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
      data_.link_ids.reserve(num_v);
      data_.active_link_ids.reserve(num_v);
      data_.static_link_ids.reserve(num_v);
      data_.joint_ids.reserve(num_e);
      data_.active_joint_ids.reserve(num_e);
      data_.floating_joint_ids.reserve(num_e);

      data_.link_ids.push_back(link->getId());
      data_.static_link_ids.push_back(link->getId());
      data_.base_link_id = link->getId();
      return;
    }

    data_.link_ids.push_back(link->getId());

    boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    SceneGraph::Edge e = *ei;
    const Joint::ConstPtr& parent_joint = boost::get(boost::edge_joint, graph)[e];
    data_.joint_ids.push_back(parent_joint->getId());

    if (parent_joint->type == JointType::FLOATING)
    {
      data_.floating_joint_ids.push_back(parent_joint->getId());
      data_.floating_joint_values[parent_joint->getId()] = parent_joint->parent_to_joint_origin_transform;
    }

    KDL::Joint kdl_jnt = convert(parent_joint);
    if (kdl_jnt.getType() != KDL::Joint::None)
    {
      data_.active_joint_ids.push_back(parent_joint->getId());
      data_.active_link_ids.push_back(link->getId());
    }
    else
    {
      auto it = std::find(data_.active_link_ids.begin(), data_.active_link_ids.end(), parent_joint->parent_link_id);
      if (it != data_.active_link_ids.end())
        data_.active_link_ids.push_back(link->getId());
      else
        data_.static_link_ids.push_back(link->getId());
    }

    // construct the kdl segment
    KDL::Segment sgm(link->getName(), kdl_jnt, convert(parent_joint->parent_to_joint_origin_transform), inert);

    // add segment to tree
    data_.tree.addSegment(sgm, parent_joint->parent_link_id.name());
  }

protected:
  KDLTreeData& data_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

/**
 * @brief Every time a vertex is visited for the first time add a new
 *        segment to the KDL Tree;
 */
struct kdl_sub_tree_builder : public boost::dfs_visitor<>
{
  kdl_sub_tree_builder(KDLTreeData& data,
                       const std::vector<JointId>& joint_ids,
                       const std::unordered_map<JointId, double>& joint_values,
                       const tesseract::common::JointIdTransformMap& floating_joint_values)
    : data_(data), joint_ids_(joint_ids), joint_values_(joint_values), floating_joint_values_(floating_joint_values)
  {
    for (const auto& [id, tf] : floating_joint_values)
      data_.floating_joint_values[id] = tf;
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
      data_.base_link_id = link->getId();
      data_.tree = KDL::Tree(link->getName());
      segment_transforms_[link->getId()] = KDL::Frame::Identity();

      std::size_t num_v = boost::num_vertices(graph);
      std::size_t num_e = boost::num_edges(graph);
      data_.link_ids.reserve(num_v);
      data_.static_link_ids.reserve(num_v);
      data_.active_link_ids.reserve(num_v);
      data_.active_joint_ids.reserve(num_e);

      data_.link_ids.push_back(link->getId());
      data_.static_link_ids.push_back(link->getId());
      return;
    }

    boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    SceneGraph::Edge e = *ei;
    const Joint::ConstPtr& parent_joint = boost::get(boost::edge_joint, graph)[e];
    bool found = (std::find(joint_ids_.begin(), joint_ids_.end(), parent_joint->getId()) != joint_ids_.end());
    KDL::Joint kdl_jnt = convert(parent_joint);
    KDL::Frame parent_to_joint = (parent_joint->type == JointType::FLOATING) ?
                                     convert(getFloatingTransformOrThrow(parent_joint)) :
                                     convert(parent_joint->parent_to_joint_origin_transform);

    KDL::Segment kdl_sgm(link->getName(), kdl_jnt, parent_to_joint, inert);
    const LinkId parent_link_id = parent_joint->parent_link_id;

    if (parent_joint->type == JointType::FIXED || parent_joint->type == JointType::FLOATING)
      segment_transforms_[parent_joint->child_link_id] = segment_transforms_[parent_link_id] * kdl_sgm.pose(0.0);
    else
      segment_transforms_[parent_joint->child_link_id] =
          segment_transforms_[parent_link_id] * kdl_sgm.pose(getJointValueOrThrow(parent_joint));

    if (!started_ && found)
    {
      started_ = found;

      // construct the kdl segment to root if needed
      if (parent_link_id != data_.base_link_id)
      {
        std::string world_joint_name = data_.base_link_id.name() + "_" + parent_link_id.name();
        KDL::Segment world_sgm = KDL::Segment(parent_link_id.name(),
                                              KDL::Joint(world_joint_name, KDL::Joint::None),
                                              segment_transforms_[parent_link_id],
                                              KDL::RigidBodyInertia(0));
        data_.tree.addSegment(world_sgm, data_.base_link_id.name());
      }
      link_ids_.push_back(parent_link_id);
      link_ids_.push_back(link->getId());
      data_.static_link_ids.push_back(parent_link_id);
      data_.link_ids.push_back(parent_link_id);
      data_.link_ids.push_back(link->getId());
      data_.active_link_ids.push_back(link->getId());
      data_.active_joint_ids.push_back(parent_joint->getId());

      // construct the kdl segment
      KDL::Segment sgm = KDL::Segment(link->getName(), kdl_jnt, parent_to_joint, inert);

      // add segment to tree
      data_.tree.addSegment(sgm, parent_link_id.name());
    }
    else if (started_)
    {
      auto it = std::find(link_ids_.begin(), link_ids_.end(), parent_link_id);

      if (it == link_ids_.end() && !found)
        return;

      if (it == link_ids_.end() && found)
      {
        data_.link_ids.push_back(parent_link_id);
        link_ids_.push_back(parent_link_id);
        data_.static_link_ids.push_back(parent_link_id);

        KDL::Frame new_tree_parent_to_joint =
            segment_transforms_[data_.base_link_id].Inverse() * segment_transforms_[parent_joint->parent_link_id];

        // construct the kdl segment
        std::string new_joint_name = data_.base_link_id.name() + "_to_" + parent_link_id.name() + "_joint";
        KDL::Segment sgm = KDL::Segment(parent_link_id.name(),
                                        KDL::Joint(new_joint_name, KDL::Joint::None),
                                        new_tree_parent_to_joint,
                                        KDL::RigidBodyInertia(0));

        // add segment to tree
        data_.tree.addSegment(sgm, data_.base_link_id.name());
      }
      else if (it != link_ids_.end() && !found)
      {
        if (parent_joint->type == JointType::FIXED || parent_joint->type == JointType::FLOATING)
          parent_to_joint = kdl_sgm.pose(0.0);
        else
          parent_to_joint = kdl_sgm.pose(getJointValueOrThrow(parent_joint));

        kdl_jnt = KDL::Joint(parent_joint->getName(), KDL::Joint::None);
      }

      data_.link_ids.push_back(link->getId());
      link_ids_.push_back(link->getId());

      auto active_it = std::find(data_.active_link_ids.begin(), data_.active_link_ids.end(), parent_link_id);
      if (active_it != data_.active_link_ids.end() || kdl_jnt.getType() != KDL::Joint::None)
        data_.active_link_ids.push_back(link->getId());
      else
        data_.static_link_ids.push_back(link->getId());

      if (kdl_jnt.getType() != KDL::Joint::None)
        data_.active_joint_ids.push_back(parent_joint->getId());

      // construct the kdl segment
      KDL::Segment sgm = KDL::Segment(link->getName(), kdl_jnt, parent_to_joint, inert);

      // add segment to tree
      data_.tree.addSegment(sgm, parent_link_id.name());
    }
  }

protected:
  /** @brief Look up a joint value or throw a descriptive error naming the offending joint.
   *
   * Callers of parseSceneGraph (sub-tree overload) must supply a value in @p joint_values_ for every
   * non-FIXED, non-FLOATING joint reachable from the root during the DFS — not only those listed in
   * @p joint_ids_.  A missing entry here means the caller's map is incomplete; we throw rather than
   * crash with a bare std::out_of_range so the caller can identify and fix the omission.
   */
  double getJointValueOrThrow(const Joint::ConstPtr& parent_joint) const
  {
    auto it = joint_values_.find(parent_joint->getId());
    if (it == joint_values_.end())
      throw std::runtime_error(
          "kdl_sub_tree_builder: joint_values is missing a value for non-fixed, non-floating joint '" +
          parent_joint->getName() +
          "'. Caller must supply values for every joint reachable from the root, not only those in joint_ids_.");
    return it->second;
  }

  /** @brief Look up a floating joint transform or throw a descriptive error naming the offending joint.
   *
   * Callers of parseSceneGraph (sub-tree overload) must supply a transform in @p floating_joint_values for
   * every FLOATING joint reachable from the root during the DFS.  A missing entry here means the caller's
   * map is incomplete; we throw rather than crash with a bare std::out_of_range so the caller can identify
   * and fix the omission.  The constructor copies @p floating_joint_values straight into
   * @p data_.floating_joint_values, so the precondition is symmetric — query the post-constructor copy.
   */
  const Eigen::Isometry3d& getFloatingTransformOrThrow(const Joint::ConstPtr& parent_joint) const
  {
    auto it = data_.floating_joint_values.find(parent_joint->getId());
    if (it == data_.floating_joint_values.end())
      throw std::runtime_error("kdl_sub_tree_builder: floating_joint_values is missing a transform for FLOATING "
                               "joint '" +
                               parent_joint->getName() +
                               "'. Caller must supply a transform for every FLOATING joint reachable from the root.");
    return it->second;
  }

  KDLTreeData& data_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  int search_cnt_{ -1 };
  bool started_{ false };
  std::map<LinkId, KDL::Frame> segment_transforms_;
  std::vector<LinkId> link_ids_;

  const std::vector<JointId>& joint_ids_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const std::unordered_map<JointId, double>& joint_values_;
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const tesseract::common::JointIdTransformMap& floating_joint_values_;
};

KDLTreeData parseSceneGraph(const SceneGraph& scene_graph)
{
  if (!scene_graph.isTree())
    throw std::runtime_error("parseSubSceneGraph: currently only works if the scene graph is a tree.");

  const auto& root = scene_graph.getRoot();
  const Link::ConstPtr& root_link = scene_graph.getLink(root);

  KDLTreeData data;
  data.tree = KDL::Tree(root.name());

  // warn if root link has inertia. KDL does not support this
  if (root_link->inertial)
  {
    CONSOLE_BRIDGE_logWarn("The root link %s has an inertia specified in the URDF, but KDL does not "
                           "support a root link with an inertia.  As a workaround, you can add an extra "
                           "dummy link to your URDF.",
                           root.name().c_str());
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
      boost::visitor(builder).root_vertex(scene_graph.getVertex(root)).vertex_index_map(prop_index_map));

  assert(data.link_ids.size() == scene_graph.getLinks().size());
  assert(data.active_joint_ids.size() <= scene_graph.getJoints().size());
  assert(data.active_link_ids.size() < scene_graph.getLinks().size());
  return data;
}

KDLTreeData parseSceneGraph(const SceneGraph& scene_graph,
                            const std::vector<JointId>& joint_ids,
                            const std::unordered_map<JointId, double>& joint_values,
                            const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  if (!scene_graph.isTree())
    throw std::runtime_error("parseSubSceneGraph: currently only works if the scene graph is a tree.");

  KDLTreeData data;
  data.tree = KDL::Tree(scene_graph.getRoot().name());

  kdl_sub_tree_builder builder(data, joint_ids, joint_values, floating_joint_values);

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

  if (data.tree.getNrOfJoints() != joint_ids.size())
    throw std::runtime_error("parseSubSceneGraph: failed to generate sub-tree given the provided joint names.");

  return data;
}

}  // namespace tesseract::scene_graph
