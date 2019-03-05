/**
 * @file kdl_env.cpp
 * @brief Tesseract environment kdl implementation.
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
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <iostream>
#include <limits>
#include <octomap/octomap.h>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_environment/kdl/kdl_env.h"
#include "tesseract_environment/kdl/kdl_utils.h"

namespace tesseract_environment
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool KDLEnv::init(tesseract_scene_graph::SceneGraphPtr scene_graph)
{
  initialized_ = false;
  scene_graph_ = scene_graph;

  if (scene_graph_ == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return initialized_;
  }

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  kdl_tree_.reset(new KDL::Tree());
  if (!tesseract_scene_graph::parseSceneGraph(*scene_graph_, *kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  initialized_ = true;

  if (initialized_)
  {
    std::vector<tesseract_scene_graph::LinkConstPtr> links = scene_graph_->getLinks();
    link_names_.reserve(links.size());
    for (const auto& link : links)
      link_names_.push_back(link->getName());

    current_state_ = EnvStatePtr(new EnvState());
    kdl_jnt_array_.resize(kdl_tree_->getNrOfJoints());
    joint_names_.resize(kdl_tree_->getNrOfJoints());
    size_t j = 0;
    for (const auto& seg : kdl_tree_->getSegments())
    {
      const KDL::Joint& jnt = seg.second.segment.getJoint();

      if (jnt.getType() == KDL::Joint::None)
        continue;
      joint_names_[j] = jnt.getName();
      joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
      kdl_jnt_array_(seg.second.q_nr) = 0.0;
      current_state_->joints.insert(std::make_pair(jnt.getName(), 0.0));

      j++;
    }

    calculateTransforms(
        current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  }

  // Now get the active link names
  getActiveLinkNamesRecursive(active_link_names_, scene_graph_, scene_graph_->getRoot(), false);

  return initialized_;
}

void KDLEnv::setState(const std::unordered_map<std::string, double>& joints)
{
  current_state_->joints.insert(joints.begin(), joints.end());

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
    {
      current_state_->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  continuous_manager_->setCollisionObjectsTransform(current_state_->transforms);
}

void KDLEnv::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  continuous_manager_->setCollisionObjectsTransform(current_state_->transforms);
}

void KDLEnv::setState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  for (const auto& tf : current_state_->transforms)
  {
    if (std::find(active_link_names_.begin(), active_link_names_.end(), tf.first) != active_link_names_.end())
    {
      continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second, tf.second);
    }
    else
    {
      continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second);
    }
  }
}

EnvStatePtr KDLEnv::getState(const std::unordered_map<std::string, double>& joints) const
{
  EnvStatePtr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
    {
      state->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

EnvStatePtr KDLEnv::getState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) const
{
  EnvStatePtr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
    {
      state->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

EnvStatePtr KDLEnv::getState(const std::vector<std::string>& joint_names,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  EnvStatePtr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
    {
      state->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

bool KDLEnv::addLink(tesseract_scene_graph::LinkPtr link)
{
  assert(false); // TODO Need to update collision environment
  return scene_graph_->addLink(link);
}

bool KDLEnv::removeLink(const std::string& name)
{
  assert(false); // TODO Need to loop through and remove children
  return scene_graph_->removeLink(name);
}

bool KDLEnv::moveLink(tesseract_scene_graph::JointPtr joint)
{
  assert(false); // TODO Need to update joint to kdl array
  std::vector<tesseract_scene_graph::JointConstPtr> joints = scene_graph_->getInboundJoints(joint->child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  return scene_graph_->addJoint(joint);
}

tesseract_scene_graph::LinkConstPtr KDLEnv::getLink(const std::string& name) const
{
  return scene_graph_->getLink(name);
}

bool KDLEnv::addJoint(tesseract_scene_graph::JointPtr joint)
{
  assert(false); // TODO Need to update joint to kdl array
  return scene_graph_->addJoint(joint);
}

bool KDLEnv::removeJoint(const std::string& name)
{
  assert(false); // TODO Need to loop through and remove children and update joint to kdl array
  return scene_graph_->removeJoint(name);
}

bool KDLEnv::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  assert(false); // TODO Need to update joint to kdl array
  return scene_graph_->moveJoint(joint_name, parent_link);
}

tesseract_scene_graph::JointConstPtr KDLEnv::getJoint(const std::string& name) const
{
  return scene_graph_->getJoint(name);
}

Eigen::VectorXd KDLEnv::getCurrentJointValues() const
{
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names_.size()));
  for (auto j = 0u; j < joint_names_.size(); ++j)
  {
    jv(j) = current_state_->joints[joint_names_[j]];
  }
  return jv;
}

VectorIsometry3d KDLEnv::getLinkTransforms() const
{
  VectorIsometry3d link_tfs;
  link_tfs.resize(link_names_.size());
  for (const auto& link_name : link_names_)
  {
    link_tfs.push_back(current_state_->transforms[link_name]);
  }
  return link_tfs;
}

const Eigen::Isometry3d& KDLEnv::getLinkTransform(const std::string& link_name) const
{
  return current_state_->transforms[link_name];
}

bool KDLEnv::setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_name);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    return true;
  }
  else
  {
    CONSOLE_BRIDGE_logError("Tried to set joint name %s which does not exist!", joint_name.c_str());
    return false;
  }
}

void KDLEnv::calculateTransformsHelper(TransformMap& transforms,
                                       const KDL::JntArray& q_in,
                                       const KDL::SegmentMap::const_iterator& it,
                                       const Eigen::Isometry3d& parent_frame) const
{
  if (it != kdl_tree_->getSegments().end())
  {
    const KDL::TreeElementType& current_element = it->second;
    KDL::Frame current_frame = GetTreeElementSegment(current_element).pose(q_in(GetTreeElementQNr(current_element)));

    Eigen::Isometry3d local_frame, global_frame;
    KDLToEigen(current_frame, local_frame);
    global_frame = parent_frame * local_frame;
    transforms[current_element.segment.getName()] = global_frame;

    for (auto& child : current_element.children)
    {
      calculateTransformsHelper(transforms, q_in, child, global_frame);
    }
  }
}

void KDLEnv::calculateTransforms(TransformMap& transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const
{
  calculateTransformsHelper(transforms, q_in, it, parent_frame);
}

bool KDLEnv::defaultIsContactAllowedFn(const std::string& link_name1, const std::string& link_name2) const
{
  if (allowed_collision_matrix_ != nullptr && allowed_collision_matrix_->isCollisionAllowed(link_name1, link_name2))
    return true;

  return false;
}

bool KDLEnv::setDiscreteContactManager(tesseract_collision::DiscreteContactManagerConstPtr manager)
{
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided manager is null.");
    return false;
  }

  discrete_manager_ = manager->clone(); // TODO add a clone Empty
  discrete_manager_->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_collision::VectorIsometry3d shape_poses;
        for (const auto& c : link->collision)
        {
          shapes.push_back(c->geometry);
          shape_poses.push_back(c->origin);
        }
        discrete_manager_->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    discrete_manager_->setActiveCollisionObjects(active_link_names_);
  }

  return true;
}

bool KDLEnv::setContinuousContactManager(tesseract_collision::ContinuousContactManagerConstPtr manager)
{
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided manager is null.");
    return false;
  }


  continuous_manager_ = manager->clone();
  continuous_manager_->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_collision::VectorIsometry3d shape_poses;
        for (const auto& c : link->collision)
        {
          shapes.push_back(c->geometry);
          shape_poses.push_back(c->origin);
        }
        continuous_manager_->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    continuous_manager_->setActiveCollisionObjects(active_link_names_);
  }

  return true;
}
}
