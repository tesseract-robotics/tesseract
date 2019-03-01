/**
 * @file kdl_kinematic_chain.cpp
 * @brief Tesseract KDL kinematics chain implementation.
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
#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/parser/kdl_parser.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
#include "tesseract_kinematics/kdl/kdl_utils.h"

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool KDLFwdKinChain::calcFwdKinHelper(Eigen::Isometry3d& pose,
                                      const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                      int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);

  return true;
}

bool KDLFwdKinChain::calcFwdKin(Eigen::Isometry3d& pose,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  return calcFwdKinHelper(pose, joint_angles);
}

bool KDLFwdKinChain::calcFwdKin(Eigen::Isometry3d& pose,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(segment_index_.find(link_name) != segment_index_.end());

  int segment_nr = segment_index_.at(link_name);
  if (calcFwdKinHelper(pose, joint_angles, segment_nr))
    return true;

  return false;
}

bool KDLFwdKinChain::calcJacobianHelper(KDL::Jacobian& jacobian,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                        int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  jacobian.resize(static_cast<unsigned>(joint_angles.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate jacobian");
    return false;
  }

  return true;
}

bool KDLFwdKinChain::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, joint_angles))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }

  return false;
}

bool KDLFwdKinChain::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                     const std::string& link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(segment_index_.find(link_name) != segment_index_.end());

  int segment_nr = segment_index_.at(link_name);
  KDL::Jacobian kdl_jacobian;

  if (calcJacobianHelper(kdl_jacobian, joint_angles, segment_nr))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }

  return false;
}

bool KDLFwdKinChain::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != robot_chain_.getNrOfJoints())
  {
    CONSOLE_BRIDGE_logError("Number of joint angles (%d) don't match robot_model (%d)",
                            static_cast<int>(vec.size()),
                            robot_chain_.getNrOfJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < joint_limits_(i, 0)) || (vec(i) > joint_limits_(i, 1)))
    {
      CONSOLE_BRIDGE_logWarn("Joint %s is out-of-range (%g < %g < %g)",
                             joint_list_[static_cast<size_t>(i)].c_str(),
                             joint_limits_(i, 0),
                             vec(i),
                             joint_limits_(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& KDLFwdKinChain::getJointNames() const
{
  assert(checkInitialized());
  return joint_list_;
}

const std::vector<std::string>& KDLFwdKinChain::getLinkNames() const
{
  assert(checkInitialized());
  return link_list_;
}

const Eigen::MatrixX2d& KDLFwdKinChain::getLimits() const { return joint_limits_; }

bool KDLFwdKinChain::init(tesseract_scene_graph::SceneGraphConstPtr scene_graph,
                          const std::string& base_link,
                          const std::string& tip_link,
                          const std::string name)
{
  initialized_ = false;

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  scene_graph_ = scene_graph;
  base_name_ = base_link;
  tip_name_ = tip_link;
  name_ = name;

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (!tesseract_scene_graph::parseSceneGraph(*scene_graph_, kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  if (!kdl_tree_.getChain(base_name_, tip_name_, robot_chain_))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize KDL between links: '%s' and '%s'", base_name_.c_str(), tip_name_.c_str());
    return false;
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);
  std::vector<int> joint_too_segment;
  joint_too_segment.resize(robot_chain_.getNrOfJoints());
  joint_too_segment.back() = -1;

  segment_index_[base_name_] = 0;
  link_list_.push_back(base_name_);
  for (unsigned i = 0, j = 0; i < robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment& seg = robot_chain_.getSegment(i);
    const KDL::Joint& jnt = seg.getJoint();
    link_list_.push_back(seg.getName());

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_list_[j] = jnt.getName();
    const tesseract_scene_graph::JointConstPtr& joint = scene_graph_->getJoint(jnt.getName());
    joint_limits_(j, 0) = joint->limits->lower;
    joint_limits_(j, 1) = joint->limits->upper;
    if (j > 0)
      joint_too_segment[j - 1] = static_cast<int>(i);

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optization library but may be nice to have
    if (joint->type == tesseract_scene_graph::JointType::CONTINUOUS &&
        std::abs(joint_limits_(j, 0) - joint_limits_(j, 1)) <= static_cast<double>(std::numeric_limits<float>::epsilon()))
    {
      joint_limits_(j, 0) = -4 * M_PI;
      joint_limits_(j, 1) = +4 * M_PI;
    }
    ++j;
  }

  for (unsigned i = 0; i < robot_chain_.getNrOfSegments(); ++i)
  {
    bool found = false;
    const KDL::Segment& seg = robot_chain_.getSegment(i);
    tesseract_scene_graph::LinkConstPtr link_model = scene_graph_->getLink(seg.getName());
    while (!found)
    {
      // Check if the link is the root
      std::vector<tesseract_scene_graph::JointConstPtr> parent_joints = scene_graph_->getInboundJoints(link_model->getName());
      if (parent_joints.empty())
      {
        segment_index_[seg.getName()] = 0;
        break;
      }

      std::string joint_name = parent_joints[0]->getName();
      std::vector<std::string>::const_iterator it = std::find(joint_list_.begin(), joint_list_.end(), joint_name);
      if (it != joint_list_.end())
      {
        unsigned joint_index = static_cast<unsigned>(it - joint_list_.begin());
        segment_index_[seg.getName()] = joint_too_segment[joint_index];
        found = true;
      }
      else
      {
        link_model = scene_graph_->getSourceLink(joint_name);
      }
    }
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  return initialized_;
}

KDLFwdKinChain& KDLFwdKinChain::operator=(const KDLFwdKinChain& rhs)
{
  initialized_ = rhs.initialized_;
  robot_chain_ = rhs.robot_chain_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
  scene_graph_ = rhs.scene_graph_;
  base_name_ = rhs.base_name_;
  tip_name_ = rhs.tip_name_;
  segment_index_ = rhs.segment_index_;

  return *this;
}
}
