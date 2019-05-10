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
#include <tesseract_collision/core/common.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_environment/kdl/kdl_env.h"
#include "tesseract_environment/kdl/kdl_utils.h"

namespace tesseract_environment
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

void KDLEnv::createKDETree()
{
  initialized_ = false;

  kdl_tree_.reset(new KDL::Tree());
  if (!tesseract_scene_graph::parseSceneGraph(*scene_graph_, *kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return;
  }

  initialized_ = true;

  if (initialized_)
  {
    std::vector<tesseract_scene_graph::LinkConstPtr> links = scene_graph_->getLinks();
    link_names_.clear();
    link_names_.reserve(links.size());
    for (const auto& link : links)
      link_names_.push_back(link->getName());

    std::vector<tesseract_scene_graph::JointConstPtr> joints = scene_graph_->getJoints();
    joint_names_.clear();
    joint_names_.reserve(joints.size());
    for (const auto& joint : joints)
      joint_names_.push_back(joint->getName());

    current_state_ = EnvStatePtr(new EnvState());
    kdl_jnt_array_.resize(kdl_tree_->getNrOfJoints());
    active_joint_names_.resize(kdl_tree_->getNrOfJoints());
    size_t j = 0;
    for (const auto& seg : kdl_tree_->getSegments())
    {
      const KDL::Joint& jnt = seg.second.segment.getJoint();

      if (jnt.getType() == KDL::Joint::None)
        continue;

      active_joint_names_[j] = jnt.getName();
      joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
      kdl_jnt_array_(seg.second.q_nr) = 0.0;
      current_state_->joints.insert(std::make_pair(jnt.getName(), 0.0));

      j++;
    }

    calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  }

  // Now get the active link names
  active_link_names_.clear();
  getActiveLinkNamesRecursive(active_link_names_, scene_graph_, scene_graph_->getRoot(), false);

  if (discrete_manager_ != nullptr) discrete_manager_->setActiveCollisionObjects(active_link_names_);
  if (continuous_manager_ != nullptr) continuous_manager_->setActiveCollisionObjects(active_link_names_);
}

bool KDLEnv::init(tesseract_scene_graph::SceneGraphPtr scene_graph)
{
  initialized_ = false;
  scene_graph_ = scene_graph;

  if (scene_graph_ == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  // This rebuilds the KDTre and updates link_names, joint_names, and active links
  createKDETree();

  is_contact_allowed_fn_ = std::bind(&tesseract_scene_graph::SceneGraph::isCollisionAllowed, scene_graph_, std::placeholders::_1, std::placeholders::_2);

  if (discrete_manager_ != nullptr) setActiveDiscreteContactManager(discrete_manager_name_);
  if (continuous_manager_ != nullptr) setActiveContinuousContactManager(continuous_manager_name_);

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

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  if (discrete_manager_ != nullptr) discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  if (continuous_manager_ != nullptr) continuous_manager_->setCollisionObjectsTransform(current_state_->transforms);
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

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  if (discrete_manager_ != nullptr) discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  if (continuous_manager_ != nullptr) continuous_manager_->setCollisionObjectsTransform(current_state_->transforms);
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

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  if (discrete_manager_ != nullptr) discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  if (continuous_manager_ != nullptr)
  {
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

bool KDLEnv::addLink(tesseract_scene_graph::Link link)
{
  std::string joint_name = "joint_" + link.getName();
  tesseract_scene_graph::Joint joint(joint_name);
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.child_link_name = link.getName();
  joint.parent_link_name = getRootLinkName();

  return addLink(link, joint);
}

bool KDLEnv::addLink(tesseract_scene_graph::Link link, tesseract_scene_graph::Joint joint)
{
  if (scene_graph_->getLink(link.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) with same name as an existing link.", link.getName().c_str());
    return false;
  }

  if (scene_graph_->getJoint(joint.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add joint (%s) with same name as an existing joint.", joint.getName().c_str());
    return false;
  }

  if (!scene_graph_->addLink(link))
    return false;

  if (!scene_graph_->addJoint(joint))
    return false;

  if (link.collision.size() > 0)
  {
    tesseract_collision::CollisionShapesConst shapes;
    tesseract_collision::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, link);

    if (discrete_manager_ != nullptr) discrete_manager_->addCollisionObject(link.getName(), 0, shapes, shape_poses, true);
    if (continuous_manager_ != nullptr) continuous_manager_->addCollisionObject(link.getName(), 0, shapes, shape_poses, true);
  }

  // This rebuilds the KDTree and updates link_names, joint_names, and active links
  createKDETree();

  return initialized_;
}

bool KDLEnv::removeLink(const std::string& name)
{
  if (scene_graph_->getLink(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) that does not exist", name.c_str());
    return false;
  }
  std::vector<tesseract_scene_graph::JointConstPtr> joints = scene_graph_->getInboundJoints(name);
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<std::string> child_link_names = scene_graph_->getLinkChildrenNames(name);

  scene_graph_->removeLink(name);
  if (discrete_manager_ != nullptr) discrete_manager_->removeCollisionObject(name);
  if (continuous_manager_ != nullptr) continuous_manager_->removeCollisionObject(name);

  for (const auto& link_name : child_link_names)
  {
    scene_graph_->removeLink(link_name);

    if (discrete_manager_ != nullptr) discrete_manager_->removeCollisionObject(link_name);
    if (continuous_manager_ != nullptr) continuous_manager_->removeCollisionObject(link_name);
  }

  // This rebuilds the KDTree and updates link_names, joint_names, and active links
  createKDETree();

  return initialized_;
}

bool KDLEnv::moveLink(tesseract_scene_graph::Joint joint)
{
  std::vector<tesseract_scene_graph::JointConstPtr> joints = scene_graph_->getInboundJoints(joint.child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  if (!scene_graph_->addJoint(joint))
    return false;

  // This rebuilds the KDTree and updates link_names, joint_names, and active links
  createKDETree();

  return initialized_;
}

tesseract_scene_graph::LinkConstPtr KDLEnv::getLink(const std::string& name) const
{
  return scene_graph_->getLink(name);
}

bool KDLEnv::addJoint(tesseract_scene_graph::Joint joint)
{
  if (!scene_graph_->addJoint(joint))
    return false;

  // This rebuilds the KDTree and updates link_names, joint_names, and active links
  createKDETree();

  return initialized_;
}

bool KDLEnv::removeJoint(const std::string& name)
{
  if (scene_graph_->getJoint(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove Joint (%s) that does not exist", name.c_str());
    return false;
  }

  std::string target_link_name = scene_graph_->getTargetLink(name)->getName();

  return removeLink(target_link_name);
}

bool KDLEnv::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  if (!scene_graph_->moveJoint(joint_name, parent_link))
    return false;

  // This rebuilds the KDTree and updates link_names, joint_names, and active links
  createKDETree();

  return initialized_;
}

bool KDLEnv::enableCollision(const std::string& name)
{
  bool result = true;
  if (discrete_manager_ != nullptr)
  {
    if(!discrete_manager_->enableCollisionObject(name))
      result = false;
  }

  if (continuous_manager_ != nullptr)
  {
    if(!continuous_manager_->enableCollisionObject(name))
      result = false;
  }
  return result;
}

bool KDLEnv::disableCollision(const std::string& name)
{
  bool result = true;
  if (discrete_manager_ != nullptr)
  {
    if(!discrete_manager_->disableCollisionObject(name))
      result = false;
  }

  if (continuous_manager_ != nullptr)
  {
    if(!continuous_manager_->disableCollisionObject(name))
      result = false;
  }
  return result;
}

void KDLEnv::addAllowedCollision(const std::string& link_name1,
                                 const std::string& link_name2,
                                 const std::string& reason)
{
  scene_graph_->addAllowedCollision(link_name1, link_name2, reason);
}

void KDLEnv::removeAllowedCollision(const std::string& link_name1,
                                    const std::string& link_name2)
{
  scene_graph_->removeAllowedCollision(link_name1, link_name2);
}

void KDLEnv::removeAllowedCollision(const std::string& link_name)
{
  scene_graph_->removeAllowedCollision(link_name);
}

const tesseract_scene_graph::AllowedCollisionMatrixConstPtr& KDLEnv::getAllowedCollisionMatrix() const
{
  return scene_graph_->getAllowedCollisionMatrix();
}

tesseract_scene_graph::JointConstPtr KDLEnv::getJoint(const std::string& name) const
{
  return scene_graph_->getJoint(name);
}

Eigen::VectorXd KDLEnv::getCurrentJointValues() const
{
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(active_joint_names_.size()));
  for (auto j = 0u; j < active_joint_names_.size(); ++j)
  {
    jv(j) = current_state_->joints[active_joint_names_[j]];
  }
  return jv;
}

Eigen::VectorXd KDLEnv::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    jv(j) = current_state_->joints[joint_names[j]];
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

void KDLEnv::getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                                tesseract_collision::VectorIsometry3d& shape_poses,
                                const tesseract_scene_graph::Link& link) const
{
  for (const auto& c : link.collision)
  {
    // Need to force convex hull TODO: Remove after URDF dom has been updated.
    if (c->geometry->getType() == tesseract_geometry::MESH)
    {
      // This is required because convex hull cannot have multiple faces on the same plane.
      std::shared_ptr<VectorVector3d> ch_verticies(new VectorVector3d());
      std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
      int ch_num_faces = tesseract_collision::createConvexHull(*ch_verticies, *ch_faces,*(std::static_pointer_cast<const tesseract_geometry::Mesh>(c->geometry)->getVertices()));
      shapes.push_back(tesseract_geometry::ConvexMeshPtr(new tesseract_geometry::ConvexMesh(ch_verticies, ch_faces, ch_num_faces)));
    }
    else
    {
      shapes.push_back(c->geometry);
    }
    shape_poses.push_back(c->origin);
  }
}

bool KDLEnv::setActiveDiscreteContactManager(const std::string& name)
{
  tesseract_collision::DiscreteContactManagerPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return false;
  }

  discrete_manager_name_ = name;
  discrete_manager_ = std::move(manager);
  return true;
}

tesseract_collision::DiscreteContactManagerPtr KDLEnv::getDiscreteContactManager(const std::string& name) const
{
  tesseract_collision::DiscreteContactManagerPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

bool KDLEnv::setActiveContinuousContactManager(const std::string& name)
{
  tesseract_collision::ContinuousContactManagerPtr manager = getContinuousContactManagerHelper(name);

  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return false;
  }

  continuous_manager_name_ = name;
  continuous_manager_ = std::move(manager);
  return true;
}

tesseract_collision::ContinuousContactManagerPtr KDLEnv::getContinuousContactManager(const std::string& name) const
{
  tesseract_collision::ContinuousContactManagerPtr manager = getContinuousContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

tesseract_collision::DiscreteContactManagerPtr KDLEnv::getDiscreteContactManagerHelper(const std::string& name) const
{
  tesseract_collision::DiscreteContactManagerPtr manager = discrete_factory_.create(name);
  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_collision::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(active_link_names_);
  }

  return manager;
}

tesseract_collision::ContinuousContactManagerPtr KDLEnv::getContinuousContactManagerHelper(const std::string& name) const
{
  tesseract_collision::ContinuousContactManagerPtr manager = continuous_factory_.create(name);

  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_collision::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(active_link_names_);
  }

  return manager;
}
}
