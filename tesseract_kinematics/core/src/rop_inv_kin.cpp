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

#include <tesseract_kinematics/core/rop_inv_kin.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

ROPInvKin::ROPInvKin(const tesseract_scene_graph::SceneGraph& scene_graph,
                     const tesseract_scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     ForwardKinematics::UPtr positioner,
                     const Eigen::VectorXd& positioner_sample_resolution,
                     std::string solver_name)
{
  if (positioner == nullptr)
    throw std::runtime_error("Provided positioner is a nullptr");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

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

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(positioner),
       positioner_limits,
       positioner_sample_resolution,
       std::move(solver_name));
}

ROPInvKin::ROPInvKin(const tesseract_scene_graph::SceneGraph& scene_graph,
                     const tesseract_scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     ForwardKinematics::UPtr positioner,
                     const Eigen::MatrixX2d& positioner_sample_range,
                     const Eigen::VectorXd& positioner_sample_resolution,
                     std::string solver_name)
{
  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(positioner),
       positioner_sample_range,
       positioner_sample_resolution,
       std::move(solver_name));
}

void ROPInvKin::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                     const tesseract_scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     ForwardKinematics::UPtr positioner,
                     const Eigen::MatrixX2d& poitioner_sample_range,
                     const Eigen::VectorXd& positioner_sample_resolution,
                     std::string solver_name)
{
  if (solver_name.empty())
    throw std::runtime_error("Solver name nust not be empty.");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  if (!(manipulator_reach > 0))
    throw std::runtime_error("Manipulator reach is not greather than zero");

  if (positioner == nullptr)
    throw std::runtime_error("Provided positioner is a nullptr");

  if (positioner_sample_resolution.size() != positioner->numJoints())
    throw std::runtime_error("Positioner sample resolution must be same size as positioner number of joints");

  for (long i = 0; i < positioner_sample_resolution.size(); ++i)
  {
    if (!(positioner_sample_resolution(i) > 0))
      throw std::runtime_error("Positioner sample resolution is not greather than zero");
  }

  // Check if the manipulator base link is the child of the positioner tip link.
  if (positioner->getTipLinkNames()[0] != manipulator->getBaseLinkName())
  {
    positioner_to_robot_ = scene_state.link_transforms.at(positioner->getTipLinkNames()[0]).inverse() *
                           scene_state.link_transforms.at(manipulator->getBaseLinkName());
  }

  solver_name_ = std::move(solver_name);
  manip_inv_kin_ = std::move(manipulator);
  positioner_fwd_kin_ = std::move(positioner);
  manip_tip_link_ = manip_inv_kin_->getTipLinkNames()[0];
  positioner_tip_link_ = positioner_fwd_kin_->getTipLinkNames()[0];
  manip_reach_ = manipulator_reach;
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
    dof_range_.emplace_back(
        Eigen::VectorXd::LinSpaced(cnt, poitioner_sample_range(d, 0), poitioner_sample_range(d, 1)));
  }
}

InverseKinematics::UPtr ROPInvKin::clone() const { return std::make_unique<ROPInvKin>(*this); }

ROPInvKin::ROPInvKin(const ROPInvKin& other) { *this = other; }

ROPInvKin& ROPInvKin::operator=(const ROPInvKin& other)
{
  manip_inv_kin_ = other.manip_inv_kin_->clone();
  positioner_fwd_kin_ = other.positioner_fwd_kin_->clone();
  manip_tip_link_ = other.manip_tip_link_;
  positioner_tip_link_ = other.positioner_tip_link_;
  manip_reach_ = other.manip_reach_;
  joint_names_ = other.joint_names_;
  dof_ = other.dof_;
  dof_range_ = other.dof_range_;

  return *this;
}

IKSolutions ROPInvKin::calcInvKinHelper(const tesseract_common::TransformMap& tip_link_poses,
                                        const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::VectorXd positioner_pose(positioner_fwd_kin_->numJoints());
  IKSolutions solutions;
  nested_ik(solutions, 0, dof_range_, tip_link_poses, positioner_pose, seed);
  return solutions;
}

void ROPInvKin::nested_ik(IKSolutions& solutions,
                          int loop_level,
                          const std::vector<Eigen::VectorXd>& dof_range,
                          const tesseract_common::TransformMap& tip_link_poses,
                          Eigen::VectorXd& positioner_pose,
                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= positioner_fwd_kin_->numJoints())
  {
    ikAt(solutions, tip_link_poses, positioner_pose, seed);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    positioner_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(solutions, loop_level + 1, dof_range, tip_link_poses, positioner_pose, seed);
  }
}

void ROPInvKin::ikAt(IKSolutions& solutions,
                     const tesseract_common::TransformMap& tip_link_poses,
                     Eigen::VectorXd& positioner_pose,
                     const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  tesseract_common::TransformMap positioner_poses = positioner_fwd_kin_->calcFwdKin(positioner_pose);
  Eigen::Isometry3d positioner_tf = positioner_poses[positioner_tip_link_] * positioner_to_robot_;
  Eigen::Isometry3d robot_target_pose = positioner_tf.inverse() * tip_link_poses.at(manip_tip_link_);
  if (robot_target_pose.translation().norm() > manip_reach_)
    return;

  tesseract_common::TransformMap robot_target_poses{ std::make_pair(manip_tip_link_, robot_target_pose) };
  auto robot_dof = static_cast<Eigen::Index>(manip_inv_kin_->numJoints());
  auto positioner_dof = static_cast<Eigen::Index>(positioner_pose.size());

  IKSolutions robot_solution_set = manip_inv_kin_->calcInvKin(robot_target_poses, seed.tail(robot_dof));
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

IKSolutions ROPInvKin::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                  const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(manip_tip_link_) != tip_link_poses.end());  // NOLINT

  return calcInvKinHelper(tip_link_poses, seed);  // NOLINT
}

std::vector<std::string> ROPInvKin::getJointNames() const { return joint_names_; }

Eigen::Index ROPInvKin::numJoints() const { return dof_; }

std::string ROPInvKin::getBaseLinkName() const { return positioner_fwd_kin_->getBaseLinkName(); }

std::string ROPInvKin::getWorkingFrame() const { return positioner_fwd_kin_->getBaseLinkName(); }

std::vector<std::string> ROPInvKin::getTipLinkNames() const { return manip_inv_kin_->getTipLinkNames(); }

std::string ROPInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
