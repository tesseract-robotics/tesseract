/**
 * @file rep_inverse_kinematics.cpp
 * @brief Tesseract Robot with External Positioner Inverse kinematics implementation.
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

#include <tesseract_kinematics/core/rep_inverse_kinematics.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::UPtr RobotWithExternalPositionerInvKin::clone() const
{
  return std::make_unique<RobotWithExternalPositionerInvKin>(*this);
}

RobotWithExternalPositionerInvKin::RobotWithExternalPositionerInvKin(const RobotWithExternalPositionerInvKin& other)
{
  *this = other;
}

RobotWithExternalPositionerInvKin& RobotWithExternalPositionerInvKin::
operator=(const RobotWithExternalPositionerInvKin& other)
{
  initialized_ = other.initialized_;
  name_ = other.name_;
  manip_inv_kin_ = other.manip_inv_kin_->clone();
  positioner_fwd_kin_ = other.positioner_fwd_kin_->clone();
  manip_reach_ = other.manip_reach_;
  joint_names_ = other.joint_names_;
  manip_base_to_positioner_base_ = other.manip_base_to_positioner_base_;
  working_frames_ = other.working_frames_;
  tip_link_names_ = other.tip_link_names_;
  dof_ = other.dof_;
  dof_range_ = other.dof_range_;

  return *this;
}

IKSolutions RobotWithExternalPositionerInvKin::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                                const std::string& working_frame,
                                                                const std::string& link_name,
                                                                const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::VectorXd positioner_pose(positioner_fwd_kin_->numJoints());
  IKSolutions solutions;
  nested_ik(solutions, 0, dof_range_, pose, working_frame, link_name, positioner_pose, seed);
  return solutions;
}

void RobotWithExternalPositionerInvKin::nested_ik(IKSolutions& solutions,
                                                  int loop_level,
                                                  const std::vector<Eigen::VectorXd>& dof_range,
                                                  const Eigen::Isometry3d& target_pose,
                                                  const std::string& working_frame,
                                                  const std::string& link_name,
                                                  Eigen::VectorXd& positioner_pose,
                                                  const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= static_cast<int>(positioner_fwd_kin_->numJoints()))
  {
    ikAt(solutions, target_pose, working_frame, link_name, positioner_pose, seed);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    positioner_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(solutions, loop_level + 1, dof_range, target_pose, working_frame, link_name, positioner_pose, seed);
  }
}

void RobotWithExternalPositionerInvKin::ikAt(IKSolutions& solutions,
                                             const Eigen::Isometry3d& target_pose,
                                             const std::string& working_frame,
                                             const std::string& link_name,
                                             Eigen::VectorXd& positioner_pose,
                                             const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  tesseract_common::TransformMap positioner_poses = positioner_fwd_kin_->calcFwdKin(positioner_pose);
  Eigen::Isometry3d positioner_tf = positioner_poses[working_frame];
  Eigen::Isometry3d robot_target_pose = manip_base_to_positioner_base_ * positioner_tf * target_pose;
  if (robot_target_pose.translation().norm() > manip_reach_)
    return;

  auto robot_dof = static_cast<Eigen::Index>(manip_inv_kin_->numJoints());
  auto positioner_dof = static_cast<Eigen::Index>(positioner_pose.size());

  IKSolutions robot_solution_set =
      manip_inv_kin_->calcInvKin(robot_target_pose, manip_inv_kin_->getBaseLinkName(), link_name, seed.tail(robot_dof));
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

IKSolutions RobotWithExternalPositionerInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                                          const std::string& working_frame,
                                                          const std::string& link_name,
                                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  assert(std::find(working_frames_.begin(), working_frames_.end(), working_frame) != working_frames_.end());
  assert(std::find(tip_link_names_.begin(), tip_link_names_.end(), link_name) != tip_link_names_.end());

  return calcInvKinHelper(pose, working_frame, link_name, seed);
}

std::vector<std::string> RobotWithExternalPositionerInvKin::getJointNames() const
{
  assert(checkInitialized());
  return joint_names_;
}

Eigen::Index RobotWithExternalPositionerInvKin::numJoints() const { return dof_; }

std::string RobotWithExternalPositionerInvKin::getBaseLinkName() const { return manip_inv_kin_->getBaseLinkName(); }

std::vector<std::string> RobotWithExternalPositionerInvKin::getWorkingFrames() const
{
  return { positioner_fwd_kin_->getTipLinkNames() };
}

std::vector<std::string> RobotWithExternalPositionerInvKin::getTipLinkNames() const
{
  return manip_inv_kin_->getTipLinkNames();
}

std::string RobotWithExternalPositionerInvKin::getName() const { return name_; }

std::string RobotWithExternalPositionerInvKin::getSolverName() const { return solver_name_; }

bool RobotWithExternalPositionerInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

bool RobotWithExternalPositionerInvKin::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                                             const tesseract_scene_graph::SceneState& scene_state,
                                             InverseKinematics::UPtr manipulator,
                                             double manipulator_reach,
                                             ForwardKinematics::UPtr positioner,
                                             Eigen::VectorXd positioner_sample_resolution,
                                             std::string name,
                                             std::string solver_name)
{
  if (positioner == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided positioner is a nullptr");
    return false;
  }

  if (!scene_graph.getLink(scene_graph.getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  std::vector<std::string> joint_names = positioner->getJointNames();
  auto s = static_cast<Eigen::Index>(joint_names.size());
  Eigen::MatrixX2d positioner_limits;
  positioner_limits.resize(s, 2);
  for (Eigen::Index i = 0; i < s; ++i)
  {
    auto joint = scene_graph.getJoint(joint_names[static_cast<std::size_t>(i)]);
    positioner_limits(i, 0) = joint->limits->lower;
    positioner_limits(i, 1) = joint->limits->upper;
  }

  return init(scene_graph,
              scene_state,
              std::move(manipulator),
              manipulator_reach,
              std::move(positioner),
              positioner_limits,
              positioner_sample_resolution,
              name,
              solver_name);
}

bool RobotWithExternalPositionerInvKin::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                                             const tesseract_scene_graph::SceneState& scene_state,
                                             InverseKinematics::UPtr manipulator,
                                             double manipulator_reach,
                                             ForwardKinematics::UPtr positioner,
                                             Eigen::MatrixX2d poitioner_sample_range,
                                             Eigen::VectorXd positioner_sample_resolution,
                                             std::string name,
                                             std::string solver_name)
{
  initialized_ = false;

  if (solver_name.empty())
  {
    CONSOLE_BRIDGE_logError("Solver name must not be empty.");
    return false;
  }

  if (!scene_graph.getLink(scene_graph.getRoot()))
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
    CONSOLE_BRIDGE_logError("Manipulator reach is not greater than zero");
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

  for (long i = 0; i < positioner_sample_resolution.size(); ++i)
  {
    if (!(positioner_sample_resolution(i) > 0))
    {
      CONSOLE_BRIDGE_logError("Positioner sample resolution is not greater than zero");
      return false;
    }
  }

  manip_base_to_positioner_base_ = scene_state.link_transforms.at(manipulator->getBaseLinkName()).inverse() *
                                   scene_state.link_transforms.at(positioner->getBaseLinkName());
  name_ = std::move(name);
  solver_name_ = std::move(solver_name);
  manip_inv_kin_ = manipulator->clone();
  manip_reach_ = manipulator_reach;
  positioner_fwd_kin_ = positioner->clone();
  working_frames_ = positioner_fwd_kin_->getTipLinkNames();
  tip_link_names_ = manip_inv_kin_->getTipLinkNames();
  dof_ = positioner_fwd_kin_->numJoints() + manip_inv_kin_->numJoints();

  joint_names_ = positioner_fwd_kin_->getJointNames();
  const auto& manip_joints = manip_inv_kin_->getJointNames();
  joint_names_.insert(joint_names_.end(), manip_joints.begin(), manip_joints.end());

  // For the kinematics object to be sampled we need to create the joint values at the sampling resolution
  // The sampled joints results are stored in dof_range[joint index] to be used by the nested_ik function
  auto positioner_num_joints = static_cast<int>(positioner_fwd_kin_->numJoints());
  dof_range_.reserve(static_cast<std::size_t>(positioner_num_joints));
  for (int d = 0; d < positioner_num_joints; ++d)
  {
    // given the sampling resolution for the joint calculate the number of samples such that the resolution is not
    // exceeded.
    int cnt = static_cast<int>(std::ceil(std::abs(poitioner_sample_range(d, 1) - poitioner_sample_range(d, 0)) /
                                         positioner_sample_resolution(d))) +
              1;
    dof_range_.push_back(Eigen::VectorXd::LinSpaced(cnt, poitioner_sample_range(d, 0), poitioner_sample_range(d, 1)));
  }

  initialized_ = true;
  return initialized_;
}
}  // namespace tesseract_kinematics
