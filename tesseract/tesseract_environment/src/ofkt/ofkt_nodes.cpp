/**
 * @file ofkt_nodes.cpp
 * @brief A implementation of the Optimized Forward Kinematic Tree Nodes.
 *
 * This is based on the paper "A Forward Kinematics Data Structure for Efficient Evolutionary Inverse Kinematics".
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 * @author Levi Armstrong
 * @date August 24, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_environment/ofkt/ofkt_nodes.h>
#include <tesseract_common/utils.h>

namespace tesseract_environment
{
/*********************************************************************/
/*************************** BASE NODE *******************************/
/*********************************************************************/
OFKTBaseNode::OFKTBaseNode(tesseract_scene_graph::JointType type, OFKTNode* parent, std::string link_name)
  : type_(type), parent_(parent), link_name_(std::move(link_name))
{
}

OFKTBaseNode::OFKTBaseNode(tesseract_scene_graph::JointType type,
                           OFKTNode* parent,
                           std::string link_name,
                           std::string joint_name,
                           Eigen::Isometry3d static_tf)
  : type_(type)
  , parent_(parent)
  , link_name_(std::move(link_name))
  , joint_name_(std::move(joint_name))
  , static_tf_(static_tf)
  , local_tf_(static_tf)
{
}

tesseract_scene_graph::JointType OFKTBaseNode::getType() const { return type_; }

void OFKTBaseNode::setParent(OFKTNode* parent)
{
  parent_ = parent;
  update_world_required_ = true;
}
OFKTNode* OFKTBaseNode::getParent() { return parent_; }
const OFKTNode* OFKTBaseNode::getParent() const { return parent_; }

const std::string& OFKTBaseNode::getLinkName() const { return link_name_; }
const std::string& OFKTBaseNode::getJointName() const { return joint_name_; }

void OFKTBaseNode::storeJointValue(double joint_value)
{
  if (!tesseract_common::almostEqualRelativeAndAbs(joint_value_, joint_value, 1e-8))
  {
    joint_value_ = joint_value;
    joint_value_changed_ = true;
  }
}

double OFKTBaseNode::getJointValue() const { return joint_value_; }

bool OFKTBaseNode::hasJointValueChanged() const { return joint_value_changed_; }

void OFKTBaseNode::setStaticTransformation(Eigen::Isometry3d static_tf)
{
  static_tf_ = static_tf;
  local_tf_ = static_tf_ * joint_tf_;
  update_world_required_ = true;
}

const Eigen::Isometry3d& OFKTBaseNode::getStaticTransformation() const { return static_tf_; }
const Eigen::Isometry3d& OFKTBaseNode::getLocalTransformation() const { return local_tf_; }
Eigen::Isometry3d OFKTBaseNode::computeLocalTransformation(double /*joint_value*/) const
{
  throw std::runtime_error("OFKTBaseNode: computeLocalTransformation should never be called!");
}

void OFKTBaseNode::computeAndStoreWorldTransformation()
{
  world_tf_ = parent_->getWorldTransformation() * local_tf_;
  update_world_required_ = false;
}
const Eigen::Isometry3d& OFKTBaseNode::getWorldTransformation() const { return world_tf_; }
bool OFKTBaseNode::updateWorldTransformationRequired() const { return update_world_required_; }

void OFKTBaseNode::addChild(OFKTNode* node)
{
  children_.push_back(node);
  children_const_.push_back(node);
}

void OFKTBaseNode::removeChild(const OFKTNode* node)
{
  children_.erase(std::remove(children_.begin(), children_.end(), node));
  children_const_.erase(std::remove(children_const_.begin(), children_const_.end(), node));
}

std::vector<OFKTNode*>& OFKTBaseNode::getChildren() { return children_; }

const std::vector<const OFKTNode*>& OFKTBaseNode::getChildren() const { return children_const_; }

/*********************************************************************/
/*************************** ROOT NODE *******************************/
/*********************************************************************/
OFKTRootNode::OFKTRootNode(std::string link_name)
  : OFKTBaseNode(tesseract_scene_graph::JointType::FIXED, nullptr, std::move(link_name))
{
  update_world_required_ = false;
}

void OFKTRootNode::setParent(OFKTNode* /*parent*/)
{
  throw std::runtime_error("OFKTRootNode: does not have a parent!");
}

void OFKTRootNode::storeJointValue(double /*joint_value*/)
{
  throw std::runtime_error("OFKTRootNode: does not have a joint value!");
}

void OFKTRootNode::setStaticTransformation(Eigen::Isometry3d /*static_tf*/)
{
  throw std::runtime_error("OFKTRootNode: does not have a static transform!");
}

void OFKTRootNode::computeAndStoreLocalTransformation() { return; }
void OFKTRootNode::computeAndStoreWorldTransformation() { return; }
bool OFKTRootNode::updateWorldTransformationRequired() const { return false; }

Eigen::Isometry3d OFKTRootNode::computeLocalTransformation(double /*joint_value*/) const { return static_tf_; }

/**********************************************************************/
/*************************** FIXED NODE *******************************/
/**********************************************************************/
OFKTFixedNode::OFKTFixedNode(OFKTNode* parent,
                             std::string link_name,
                             std::string joint_name,
                             Eigen::Isometry3d static_tf)
  : OFKTBaseNode(tesseract_scene_graph::JointType::FIXED,
                 parent,
                 std::move(link_name),
                 std::move(joint_name),
                 static_tf)
{
  computeAndStoreWorldTransformation();
}

void OFKTFixedNode::storeJointValue(double /*joint_value*/)
{
  throw std::runtime_error("OFKTFixedNode: does not have a joint value!");
}

double OFKTFixedNode::getJointValue() const { throw std::runtime_error("OFKTFixedNode: does not have a joint value!"); }

void OFKTFixedNode::setStaticTransformation(Eigen::Isometry3d static_tf)
{
  static_tf_ = static_tf;
  local_tf_ = static_tf;
  update_world_required_ = true;
}

void OFKTFixedNode::computeAndStoreLocalTransformation() { return; }

Eigen::Isometry3d OFKTFixedNode::computeLocalTransformation(double /*joint_value*/) const { return local_tf_; }

/*********************************************************************/
/************************* REVOLUTE NODE *****************************/
/*********************************************************************/
OFKTRevoluteNode::OFKTRevoluteNode(OFKTNode* parent,
                                   std::string link_name,
                                   std::string joint_name,
                                   Eigen::Isometry3d static_tf,
                                   Eigen::Vector3d axis,
                                   Eigen::Vector2d joint_limits)
  : OFKTBaseNode(tesseract_scene_graph::JointType::REVOLUTE,
                 parent,
                 std::move(link_name),
                 std::move(joint_name),
                 static_tf)
  , axis_(axis.normalized())
  , joint_limits_(joint_limits)
{
  if (joint_limits_[1] < joint_limits_[0])
    throw std::runtime_error("OFKTRevoluteNode: Invalid joint limits!");

  if (tesseract_common::almostEqualRelativeAndAbs(joint_limits_[0], joint_limits_[1], 1e-5))
    throw std::runtime_error("OFKTRevoluteNode: Invalid joint limits!");

  computeAndStoreLocalTransformation();
  computeAndStoreWorldTransformation();
}

void OFKTRevoluteNode::storeJointValue(double joint_value)
{
  if (joint_value > joint_limits_[1])
  {
    CONSOLE_BRIDGE_logDebug("OFKTRevoluteNode: Joint value (%f) is above the upper limits (%f), setting to upper "
                            "limit.",
                            joint_value,
                            joint_limits_[1]);
    joint_value = joint_limits_[1];
  }
  else if (joint_value < joint_limits_[0])
  {
    CONSOLE_BRIDGE_logDebug("OFKTRevoluteNode: Joint value (%f) is below the lower limits (%f), setting to lower "
                            "limit.",
                            joint_value,
                            joint_limits_[0]);
    joint_value = joint_limits_[0];
  }

  OFKTBaseNode::storeJointValue(joint_value);
}

void OFKTRevoluteNode::computeAndStoreLocalTransformation()
{
  joint_tf_ = Eigen::AngleAxisd(joint_value_, axis_);
  local_tf_ = static_tf_ * joint_tf_;
  joint_value_changed_ = false;
}

Eigen::Isometry3d OFKTRevoluteNode::computeLocalTransformation(double joint_value) const
{
  if (joint_value > joint_limits_[1])
  {
    CONSOLE_BRIDGE_logDebug("OFKTRevoluteNode: Joint value (%f) is above the upper limits (%f), setting to upper "
                            "limit.",
                            joint_value,
                            joint_limits_[1]);
    joint_value = joint_limits_[1];
  }
  else if (joint_value < joint_limits_[0])
  {
    CONSOLE_BRIDGE_logDebug("OFKTRevoluteNode: Joint value (%f) is below the lower limits (%f), setting to lower "
                            "limit.",
                            joint_value,
                            joint_limits_[0]);
    joint_value = joint_limits_[0];
  }

  return static_tf_ * Eigen::AngleAxisd(joint_value, axis_);
}

const Eigen::Vector3d& OFKTRevoluteNode::getAxis() const { return axis_; }

const Eigen::Vector2d& OFKTRevoluteNode::getJointLimits() const { return joint_limits_; }

/*********************************************************************/
/************************ CONTINUOUS NODE ****************************/
/*********************************************************************/
OFKTContinuousNode::OFKTContinuousNode(OFKTNode* parent,
                                       std::string link_name,
                                       std::string joint_name,
                                       Eigen::Isometry3d static_tf,
                                       Eigen::Vector3d axis)
  : OFKTBaseNode(tesseract_scene_graph::JointType::CONTINUOUS,
                 parent,
                 std::move(link_name),
                 std::move(joint_name),
                 static_tf)
  , axis_(axis.normalized())
{
  computeAndStoreLocalTransformation();
  computeAndStoreWorldTransformation();
}

void OFKTContinuousNode::computeAndStoreLocalTransformation()
{
  joint_tf_ = Eigen::AngleAxisd(joint_value_, axis_);
  local_tf_ = static_tf_ * joint_tf_;
  joint_value_changed_ = false;
}

Eigen::Isometry3d OFKTContinuousNode::computeLocalTransformation(double joint_value) const
{
  return static_tf_ * Eigen::AngleAxisd(joint_value, axis_);
}

const Eigen::Vector3d& OFKTContinuousNode::getAxis() const { return axis_; }

/*********************************************************************/
/************************* PRISMATIC NODE ****************************/
/*********************************************************************/
OFKTPrismaticNode::OFKTPrismaticNode(OFKTNode* parent,
                                     std::string link_name,
                                     std::string joint_name,
                                     Eigen::Isometry3d static_tf,
                                     Eigen::Vector3d axis)
  : OFKTBaseNode(tesseract_scene_graph::JointType::PRISMATIC,
                 parent,
                 std::move(link_name),
                 std::move(joint_name),
                 static_tf)
  , axis_(axis.normalized())
{
  computeAndStoreLocalTransformation();
  computeAndStoreWorldTransformation();
}

void OFKTPrismaticNode::computeAndStoreLocalTransformation()
{
  joint_tf_ = Eigen::Translation3d(joint_value_ * axis_);
  local_tf_ = static_tf_ * joint_tf_;
  joint_value_changed_ = false;
}

Eigen::Isometry3d OFKTPrismaticNode::computeLocalTransformation(double joint_value) const
{
  return static_tf_ * Eigen::Translation3d(joint_value * axis_);
}

const Eigen::Vector3d& OFKTPrismaticNode::getAxis() const { return axis_; }
}  // namespace tesseract_environment
