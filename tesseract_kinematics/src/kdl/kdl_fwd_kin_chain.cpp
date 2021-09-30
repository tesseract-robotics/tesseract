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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/kdl_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

KDLFwdKinChain::KDLFwdKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                               const std::vector<std::pair<std::string, std::string>>& chains,
                               std::string solver_name)
  : solver_name_(std::move(solver_name))
{
  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (!parseSceneGraph(kdl_data_, scene_graph, chains))
    throw std::runtime_error("Failed to parse KDL data from Scene Graph");

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);
}

KDLFwdKinChain::KDLFwdKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                               const std::string& base_link,
                               const std::string& tip_link,
                               std::string solver_name)
  : KDLFwdKinChain(scene_graph, { std::make_pair(base_link, tip_link) }, std::move(solver_name))
{
}

ForwardKinematics::UPtr KDLFwdKinChain::clone() const { return std::make_unique<KDLFwdKinChain>(*this); }

KDLFwdKinChain::KDLFwdKinChain(const KDLFwdKinChain& other) { *this = other; }
KDLFwdKinChain& KDLFwdKinChain::operator=(const KDLFwdKinChain& other)
{
  name_ = other.name_;
  kdl_data_ = other.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);
  solver_name_ = other.solver_name_;
  return *this;
}

tesseract_common::TransformMap
KDLFwdKinChain::calcFwdKinHelperAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  if (joint_angles.rows() != kdl_data_.robot_chain.getNrOfJoints())
    throw std::runtime_error("kdl_joints size is not correct!");

  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  unsigned int j = 0;
  tesseract_common::TransformMap all_poses;
  const KDL::Segment& segment = kdl_data_.robot_chain.getSegment(j);
  KDL::Frame kdl_pose;
  if (segment.getJoint().getType() != KDL::Joint::None)
  {
    kdl_pose = segment.pose(kdl_joints(j));
    j++;
  }
  else
  {
    kdl_pose = segment.pose(0.0);
  }

  Eigen::Isometry3d pose;
  KDLToEigen(kdl_pose, pose);
  all_poses[segment.getName()] = pose;

  for (unsigned int i = 1; i < kdl_data_.robot_chain.getNrOfSegments(); i++)
  {
    const KDL::Segment& segment = kdl_data_.robot_chain.getSegment(i);
    if (segment.getJoint().getType() != KDL::Joint::None)
    {
      kdl_pose = kdl_pose * segment.pose(kdl_joints(j));
      j++;
    }
    else
    {
      kdl_pose = kdl_pose * segment.pose(0.0);
    }

    Eigen::Isometry3d pose;
    KDLToEigen(kdl_pose, pose);
    all_poses[segment.getName()] = pose;
  }

  tesseract_common::TransformMap poses;
  poses[kdl_data_.tip_link_name] = all_poses[kdl_data_.tip_link_name];

  return poses;
}

tesseract_common::TransformMap KDLFwdKinChain::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(joint_angles.size() == numJoints());

  return calcFwdKinHelperAll(joint_angles);
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

Eigen::MatrixXd KDLFwdKinChain::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                             const std::string& link_name) const
{
  assert(joint_angles.size() == numJoints());

  int segment_nr = kdl_data_.segment_index.at(link_name);
  KDL::Jacobian kdl_jacobian;

  if (calcJacobianHelper(kdl_jacobian, joint_angles, segment_nr))
  {
    Eigen::MatrixXd jacobian(6, numJoints());
    KDLToEigen(kdl_jacobian, jacobian);
    return jacobian;
  }

  throw std::runtime_error("KDLFwdKinChain: Failed to calculate jacobian.");
}

std::vector<std::string> KDLFwdKinChain::getJointNames() const { return kdl_data_.joint_names; }

Eigen::Index KDLFwdKinChain::numJoints() const { return static_cast<Eigen::Index>(kdl_data_.joint_names.size()); }

std::string KDLFwdKinChain::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::vector<std::string> KDLFwdKinChain::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string KDLFwdKinChain::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
