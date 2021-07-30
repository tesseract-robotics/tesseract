/**
 * @file rop_inverse_kinematics.cpp
 * @brief Tesseract Robot on Positioner Inverse kinematics implementation.
 *
 * @author Levi Armstrong
 * @date June 25, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/rop_inverse_kinematics.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr RobotOnPositionerInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<RobotOnPositionerInvKin>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool RobotOnPositionerInvKin::update()
{
  manip_inv_kin_->update();
  positioner_fwd_kin_->update();
  return init(scene_graph_, manip_inv_kin_, manip_reach_, positioner_fwd_kin_, positioner_sample_resolution_, name_);
}

IKSolutions RobotOnPositionerInvKin::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                      const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::VectorXd positioner_pose(positioner_fwd_kin_->numJoints());
  IKSolutions solutions;
  nested_ik(solutions, 0, dof_range_, pose, positioner_pose, seed);
  return solutions;
}

void RobotOnPositionerInvKin::nested_ik(IKSolutions& solutions,
                                        int loop_level,
                                        const std::vector<Eigen::VectorXd>& dof_range,
                                        const Eigen::Isometry3d& target_pose,
                                        Eigen::VectorXd& positioner_pose,
                                        const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= static_cast<int>(positioner_fwd_kin_->numJoints()))
  {
    ikAt(solutions, target_pose, positioner_pose, seed);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    positioner_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(solutions, loop_level + 1, dof_range, target_pose, positioner_pose, seed);
  }
}

void RobotOnPositionerInvKin::ikAt(IKSolutions& solutions,
                                   const Eigen::Isometry3d& target_pose,
                                   Eigen::VectorXd& positioner_pose,
                                   const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::Isometry3d positioner_tf = positioner_fwd_kin_->calcFwdKin(positioner_pose);
  Eigen::Isometry3d robot_target_pose = positioner_tf.inverse() * target_pose;
  if (robot_target_pose.translation().norm() > manip_reach_)
    return;

  auto robot_dof = static_cast<Eigen::Index>(manip_inv_kin_->numJoints());
  auto positioner_dof = static_cast<Eigen::Index>(positioner_pose.size());

  IKSolutions robot_solution_set = manip_inv_kin_->calcInvKin(robot_target_pose, seed.tail(robot_dof));
  if (robot_solution_set.empty())
    return;

  for (const auto& robot_solution : robot_solution_set)
  {
    Eigen::VectorXd full_sol;
    full_sol.resize(positioner_dof + robot_dof);
    full_sol.head(positioner_dof) = positioner_pose;
    full_sol.tail(robot_dof) = robot_solution;
    solutions.push_back(full_sol);
  }
}

IKSolutions RobotOnPositionerInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                                const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(pose, seed);
}

IKSolutions RobotOnPositionerInvKin::calcInvKin(const Eigen::Isometry3d& /*pose*/,
                                                const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                                const std::string& /*link_name*/) const
{
  assert(checkInitialized());
  throw std::runtime_error("This method call is not supported by RobotOnPositionerInvKin yet.");
}

bool RobotOnPositionerInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != dof_)
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), dof_);
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < limits_.joint_limits(i, 0)) || (vec(i) > limits_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_names_[static_cast<size_t>(i)].c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getJointNames() const
{
  assert(checkInitialized());
  return joint_names_;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getLinkNames() const
{
  assert(checkInitialized());
  return link_names_;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getActiveLinkNames() const
{
  assert(checkInitialized());
  return active_link_names_;
}

const tesseract_common::KinematicLimits& RobotOnPositionerInvKin::getLimits() const { return limits_; }

void RobotOnPositionerInvKin::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  unsigned int pj = positioner_fwd_kin_->numJoints();
  tesseract_common::KinematicLimits positioner_limits;
  positioner_limits.joint_limits = limits.joint_limits.topRows(pj);
  positioner_limits.velocity_limits = limits.velocity_limits.head(pj);
  positioner_limits.acceleration_limits = limits.acceleration_limits.head(pj);
  positioner_fwd_kin_->setLimits(positioner_limits);

  unsigned int mj = manip_inv_kin_->numJoints();
  tesseract_common::KinematicLimits manipulator_limits;
  manipulator_limits.joint_limits = limits.joint_limits.bottomRows(mj);
  manipulator_limits.velocity_limits = limits.velocity_limits.tail(mj);
  manipulator_limits.acceleration_limits = limits.acceleration_limits.tail(mj);
  manip_inv_kin_->setLimits(manipulator_limits);

  limits_ = std::move(limits);
}

std::vector<Eigen::Index> RobotOnPositionerInvKin::getRedundancyCapableJointIndices() const
{
  return redundancy_indices_;
}

unsigned int RobotOnPositionerInvKin::numJoints() const { return dof_; }

const std::string& RobotOnPositionerInvKin::getBaseLinkName() const { return positioner_fwd_kin_->getBaseLinkName(); }

const std::string& RobotOnPositionerInvKin::getTipLinkName() const { return manip_inv_kin_->getTipLinkName(); }

const std::string& RobotOnPositionerInvKin::getName() const { return name_; }

const std::string& RobotOnPositionerInvKin::getSolverName() const { return solver_name_; }

tesseract_scene_graph::SceneGraph::ConstPtr RobotOnPositionerInvKin::getSceneGraph() const { return scene_graph_; };

bool RobotOnPositionerInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

bool RobotOnPositionerInvKin::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                   InverseKinematics::Ptr manipulator,
                                   double manipulator_reach,
                                   ForwardKinematics::Ptr positioner,
                                   Eigen::VectorXd positioner_sample_resolution,
                                   std::string name,
                                   std::string solver_name)
{
  initialized_ = false;

  if (solver_name.empty())
  {
    CONSOLE_BRIDGE_logError("Solver name nust not be empty.");
    return false;
  }

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  if (!scene_graph->getLink(scene_graph->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (manipulator == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided manipulator is a nullptr");
    return false;
  }

  if (!(manipulator_reach > 0))
  {
    CONSOLE_BRIDGE_logError("Manipulator reach is not greather than zero");
    return false;
  }

  if (positioner == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided positioner is a nullptr");
    return false;
  }

  if (positioner_sample_resolution.size() != positioner->numJoints())
  {
    CONSOLE_BRIDGE_logError("Positioner sample resolution must be same size as positioner number of joints");
    return false;
  }

  // Check if the manipulator base link is the child of the positioner tip link.
  if (positioner->getTipLinkName() != manipulator->getBaseLinkName())
  {
    CONSOLE_BRIDGE_logWarn("Positioner tip link is not the base link of the manipulator.");
  }

  for (long i = 0; i < positioner_sample_resolution.size(); ++i)
  {
    if (!(positioner_sample_resolution(i) > 0))
    {
      CONSOLE_BRIDGE_logError("Positioner sample resolution is not greather than zero");
      return false;
    }
  }

  scene_graph_ = std::move(scene_graph);
  name_ = std::move(name);
  solver_name_ = std::move(solver_name);
  manip_inv_kin_ = manipulator->clone();
  manip_reach_ = manipulator_reach;
  positioner_fwd_kin_ = positioner->clone();
  positioner_sample_resolution_ = positioner_sample_resolution;
  dof_ = positioner_fwd_kin_->numJoints() + manip_inv_kin_->numJoints();

  limits_.joint_limits = Eigen::MatrixX2d(dof_, 2);
  limits_.joint_limits << positioner_fwd_kin_->getLimits().joint_limits, manip_inv_kin_->getLimits().joint_limits;

  limits_.velocity_limits = Eigen::VectorXd(dof_);
  limits_.velocity_limits << positioner_fwd_kin_->getLimits().velocity_limits,
      manip_inv_kin_->getLimits().velocity_limits;

  limits_.acceleration_limits = Eigen::VectorXd(dof_);
  limits_.acceleration_limits << positioner_fwd_kin_->getLimits().acceleration_limits,
      manip_inv_kin_->getLimits().acceleration_limits;

  joint_names_ = positioner_fwd_kin_->getJointNames();
  const auto& manip_joints = manip_inv_kin_->getJointNames();
  joint_names_.insert(joint_names_.end(), manip_joints.begin(), manip_joints.end());

  link_names_ = positioner_fwd_kin_->getLinkNames();
  const auto& manip_links = manip_inv_kin_->getLinkNames();
  link_names_.insert(link_names_.end(), manip_links.begin(), manip_links.end());

  // Remove duplicates
  std::sort(link_names_.begin(), link_names_.end());
  link_names_.erase(std::unique(link_names_.begin(), link_names_.end()), link_names_.end());

  active_link_names_ = positioner_fwd_kin_->getActiveLinkNames();
  active_link_names_.insert(active_link_names_.end(), manip_links.begin(), manip_links.end());

  // Remove duplicates
  std::sort(active_link_names_.begin(), active_link_names_.end());
  active_link_names_.erase(std::unique(active_link_names_.begin(), active_link_names_.end()), active_link_names_.end());

  // Get redundancy indices
  redundancy_indices_.clear();
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = scene_graph_->getJoint(joint_names_[i]);
    switch (joint->type)
    {
      case tesseract_scene_graph::JointType::REVOLUTE:
      case tesseract_scene_graph::JointType::CONTINUOUS:
        redundancy_indices_.push_back(static_cast<Eigen::Index>(i));
        break;
      default:
        break;
    }
  }

  auto positioner_num_joints = static_cast<int>(positioner_fwd_kin_->numJoints());
  const Eigen::MatrixX2d& positioner_limits = positioner_fwd_kin_->getLimits().joint_limits;

  // For the kinematics object to be sampled we need to create the joint values at the sampling resolution
  // The sampled joints results are stored in dof_range[joint index] to be used by the nested_ik function
  dof_range_.reserve(static_cast<std::size_t>(positioner_num_joints));
  for (int d = 0; d < positioner_num_joints; ++d)
  {
    // given the sampling resolution for the joint calculate the number of samples such that the resolution is not
    // exceeded.
    int cnt = static_cast<int>(std::ceil(std::abs(positioner_limits(d, 1) - positioner_limits(d, 0)) /
                                         positioner_sample_resolution_(d))) +
              1;
    dof_range_.push_back(Eigen::VectorXd::LinSpaced(cnt, positioner_limits(d, 0), positioner_limits(d, 1)));
  }

  initialized_ = true;
  return initialized_;
}

bool RobotOnPositionerInvKin::init(const RobotOnPositionerInvKin& kin)
{
  initialized_ = kin.initialized_;
  scene_graph_ = kin.scene_graph_;
  name_ = kin.name_;
  manip_inv_kin_ = kin.manip_inv_kin_->clone();
  manip_reach_ = kin.manip_reach_;
  positioner_fwd_kin_ = kin.positioner_fwd_kin_->clone();
  positioner_sample_resolution_ = kin.positioner_sample_resolution_;
  dof_ = kin.dof_;
  limits_ = kin.limits_;
  joint_names_ = kin.joint_names_;
  link_names_ = kin.link_names_;
  active_link_names_ = kin.active_link_names_;
  redundancy_indices_ = kin.redundancy_indices_;
  dof_range_ = kin.dof_range_;

  return initialized_;
}
}  // namespace tesseract_kinematics
