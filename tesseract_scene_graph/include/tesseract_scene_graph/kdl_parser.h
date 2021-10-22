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
#include <kdl/jacobian.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>

namespace tesseract_scene_graph
{
/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @return frame Output KDL Frame
 */
KDL::Frame convert(const Eigen::Isometry3d& transform);

/**
 * @brief Convert KDL::Frame to Eigen::Isometry3d
 * @param frame Input KDL Frame
 * @return Eigen transform (Isometry3d)
 */
Eigen::Isometry3d convert(const KDL::Frame& frame);

/**
 * @brief Convert Eigen::Vector3d to KDL::Vector
 * @param vector Input Eigen Vector3d
 * @return vector Output KDL Vector
 */
KDL::Vector convert(const Eigen::Vector3d& vector);

/**
 * @brief Convert KDL::Vector to Eigen::Vector3d
 * @param transform Input KDL Vector
 * @return frame Output Eigen Vector3d
 */
Eigen::Vector3d convert(const KDL::Vector& vector);

/**
 * @brief Convert KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @return Eigen MatrixXd
 */
Eigen::MatrixXd convert(const KDL::Jacobian& jacobian);

/**
 * @brief Convert Eigen::Matrix to KDL::Jacobian
 * @param jacobian Input Eigen MatrixXd
 * @return KDL Jacobian
 */
KDL::Jacobian convert(const Eigen::MatrixXd& jacobian);

/**
 * @brief Convert a subset of KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param q_nrs Input the columns to use
 * @return Eigen MatrixXd
 */
Eigen::MatrixXd convert(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs);

/**
 * @brief Convert Tesseract Joint to KDL Joint
 * @param joint Tesseract Joint
 * @return A KDL Joint
 */
KDL::Joint convert(const Joint::ConstPtr& joint);

/**
 * @brief Convert Tesseract Inertail to KDL Inertial
 * @param inertial
 * @return
 */
KDL::RigidBodyInertia convert(const Inertial::ConstPtr& inertial);

/** @brief The KDLTreeData populated when parsing scene graph */
struct KDLTreeData
{
  KDL::Tree tree;
  std::string base_link_name;
  std::vector<std::string> joint_names;
  std::vector<std::string> active_joint_names;
  std::vector<std::string> link_names;
  std::vector<std::string> active_link_names;
  std::vector<std::string> static_link_names;
};

/**
 * @brief Convert a Tesseract SceneGraph into a KDL Tree
 * @throws If graph is not a tree
 * @param scene_graph The Tesseract Scene Graph
 * @param tree The KDL Tree to populate.
 * @return Returns KDL tree data representation of the scene graph
 */
KDLTreeData parseSceneGraph(const SceneGraph& scene_graph);

/**
 * @brief Convert a portion of a Tesseract SceneGraph into a KDL Tree
 * @details This will create a new tree from multiple sub tree defined by the provided joint names
 * The values are used to convert non fixed joints that are not listed in joint_names to a
 * fixed joint. The first tree found a link is defined attaching world to the base link and all
 * other trees are attached to this link by a fixed joint.
 * @throws If graph is not a tree it will return false.
 * @param scene_graph The Tesseract Scene Graph
 * @param tree The KDL Tree to populate.
 * @return Returns KDL tree representation of the sub scene graph
 */
KDLTreeData parseSceneGraph(const SceneGraph& scene_graph,
                            const std::vector<std::string>& joint_names,
                            const std::unordered_map<std::string, double>& joint_values);

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_KDL_PARSER_H
