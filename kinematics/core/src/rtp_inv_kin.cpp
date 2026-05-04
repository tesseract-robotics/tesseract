/**
 * @file rtp_inv_kin.cpp
 * @brief Robot with Tool Positioner Inverse kinematics implementation.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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

#include <cassert>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/scene_state.h>

namespace tesseract::kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  Eigen::MatrixX2d tool_limits = gatherJointLimits(scene_graph, tool_positioner->getJointNames());

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(tool_positioner),
       tool_limits,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(tool_positioner),
       tool_sample_range,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");
  if (manipulator->getTipLinkNames().size() != 1)
    throw std::runtime_error("RTPInvKin requires a manipulator with exactly one tip link");

  Eigen::MatrixX2d tool_limits = gatherJointLimits(scene_graph, tool_positioner->getJointNames());

  const double auto_reach =
      computeChainReachUpperBound(scene_graph, manipulator->getBaseLinkName(), manipulator->getTipLinkNames()[0]);

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       auto_reach,
       std::move(tool_positioner),
       tool_limits,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");
  if (manipulator->getTipLinkNames().size() != 1)
    throw std::runtime_error("RTPInvKin requires a manipulator with exactly one tip link");

  const double auto_reach =
      computeChainReachUpperBound(scene_graph, manipulator->getBaseLinkName(), manipulator->getTipLinkNames()[0]);

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       auto_reach,
       std::move(tool_positioner),
       tool_sample_range,
       tool_sample_resolution,
       std::move(solver_name));
}

void RTPInvKin::init(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (solver_name.empty())
    throw std::runtime_error("Solver name must not be empty.");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  if (manipulator->getTipLinkNames().size() != 1)
    throw std::runtime_error("RTPInvKin requires a manipulator with exactly one tip link");

  if (!(manipulator_reach > 0))
    throw std::runtime_error("Manipulator reach is not greater than zero");

  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");

  if (tool_sample_resolution.size() != tool_positioner->numJoints())
    throw std::runtime_error("Tool sample resolution must be same size as tool positioner number of joints");

  if (tool_sample_range.rows() != tool_positioner->numJoints())
    throw std::runtime_error("Tool sample range must have one row per tool positioner joint");

  for (long i = 0; i < tool_sample_resolution.size(); ++i)
  {
    if (!(tool_sample_resolution(i) > 0))
      throw std::runtime_error("Tool sample resolution is not greater than zero");
    if (tool_sample_range(i, 0) > tool_sample_range(i, 1))
      throw std::runtime_error("Tool sample range minimum is greater than maximum");
  }

  // The static-offset model below assumes the tool positioner's base is rigidly attached to the
  // manipulator tip. If an active joint sits between them, sampling the tool kinematics no longer
  // produces a deterministic wrist target and IK results would silently be wrong.
  const std::string manip_tip = manipulator->getTipLinkNames()[0];
  const std::string tool_base = tool_positioner->getBaseLinkName();
  if (manip_tip != tool_base)
  {
    if (scene_graph.getLink(tool_base) == nullptr)
      throw std::runtime_error("Tool positioner base link '" + tool_base + "' not found in scene graph");
    const auto path = scene_graph.getShortestPath(manip_tip, tool_base);
    if (path.links.size() < 2)
      throw std::runtime_error("Tool positioner base link '" + tool_base +
                               "' is not connected to manipulator tip link '" + manip_tip + "'");
    if (!path.active_joints.empty())
      throw std::runtime_error("Tool positioner base link '" + tool_base +
                               "' must be rigidly attached to manipulator tip link '" + manip_tip +
                               "'; found active joint '" + path.active_joints.front() + "' on path");
  }

  // Static offset from the manipulator tip to the tool positioner's base link (usually identity
  // because the tool chain begins at the manipulator tip, but allow any fixed transform).
  manip_tip_to_tool_base_ = scene_state.link_transforms.at(manipulator->getTipLinkNames()[0]).inverse() *
                            scene_state.link_transforms.at(tool_positioner->getBaseLinkName());

  solver_name_ = std::move(solver_name);
  manip_tip_link_ = manipulator->getTipLinkNames()[0];
  manip_inv_kin_ = std::move(manipulator);
  tool_tip_link_ = tool_positioner->getTipLinkNames()[0];
  tool_fwd_kin_ = std::move(tool_positioner);
  manip_reach_ = manipulator_reach;
  dof_ = manip_inv_kin_->numJoints() + tool_fwd_kin_->numJoints();

  // Internal invariants the public ctors are expected to honor; these catch bugs that bypass
  // the throw checks above (e.g. a future ctor that forgets to call gatherJointLimits correctly).
  assert(tool_sample_resolution.size() == tool_fwd_kin_->numJoints());       // NOLINT
  assert(tool_sample_range.rows() == tool_fwd_kin_->numJoints());            // NOLINT
  assert(dof_ == manip_inv_kin_->numJoints() + tool_fwd_kin_->numJoints());  // NOLINT

  // Joint order: manipulator first, tool second.
  joint_names_ = manip_inv_kin_->getJointNames();
  const auto& tool_joints = tool_fwd_kin_->getJointNames();
  joint_names_.insert(joint_names_.end(), tool_joints.begin(), tool_joints.end());

  dof_range_ = buildSampleGrid(tool_sample_range, tool_sample_resolution);

  grid_size_ = 1;
  for (const auto& d : dof_range_)
    grid_size_ *= static_cast<std::size_t>(d.size());

  // OPW solvers return up to 8 solutions per grid sample; numerical solvers return ≤ 1.
  // Reserve conservatively against the OPW upper bound so the vector does not regrow
  // mid-IK. Wasted memory is bounded by 8× grid_size_.
  constexpr std::size_t kMaxInnerSolutionsPerSample = 8;
  solutions_capacity_hint_ = grid_size_ * kMaxInnerSolutionsPerSample;
}

InverseKinematics::UPtr RTPInvKin::clone() const { return std::make_unique<RTPInvKin>(*this); }

RTPInvKin::RTPInvKin(const RTPInvKin& other)
  : joint_names_(other.joint_names_)
  , manip_inv_kin_(other.manip_inv_kin_->clone())
  , tool_fwd_kin_(other.tool_fwd_kin_->clone())
  , manip_tip_link_(other.manip_tip_link_)
  , tool_tip_link_(other.tool_tip_link_)
  , manip_reach_(other.manip_reach_)
  , manip_tip_to_tool_base_(other.manip_tip_to_tool_base_)
  , dof_(other.dof_)
  , grid_size_(other.grid_size_)
  , solutions_capacity_hint_(other.solutions_capacity_hint_)
  , dof_range_(other.dof_range_)
  , solver_name_(other.solver_name_)
{
}

RTPInvKin& RTPInvKin::operator=(const RTPInvKin& other)
{
  if (this == &other)
    return *this;

  manip_inv_kin_ = other.manip_inv_kin_->clone();
  tool_fwd_kin_ = other.tool_fwd_kin_->clone();
  manip_tip_link_ = other.manip_tip_link_;
  tool_tip_link_ = other.tool_tip_link_;
  manip_reach_ = other.manip_reach_;
  joint_names_ = other.joint_names_;
  manip_tip_to_tool_base_ = other.manip_tip_to_tool_base_;
  dof_ = other.dof_;
  grid_size_ = other.grid_size_;
  solutions_capacity_hint_ = other.solutions_capacity_hint_;
  dof_range_ = other.dof_range_;
  solver_name_ = other.solver_name_;
  return *this;
}

void RTPInvKin::calcInvKinHelper(IKSolutions& solutions,
                                 const tesseract::common::TransformMap& tip_link_poses,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  solutions.reserve(solutions_capacity_hint_);
  const Eigen::Isometry3d& target_tool_tip = tip_link_poses.at(tool_tip_link_);
  Eigen::VectorXd tool_pose(tool_fwd_kin_->numJoints());
  nested_ik(solutions, 0, dof_range_, target_tool_tip, tool_pose, seed);
}

void RTPInvKin::nested_ik(IKSolutions& solutions,
                          Eigen::Index loop_level,
                          const std::vector<Eigen::VectorXd>& dof_range,
                          const Eigen::Isometry3d& target_tool_tip,
                          Eigen::VectorXd& tool_pose,
                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= static_cast<Eigen::Index>(dof_range.size()))
  {
    ikAt(solutions, target_tool_tip, tool_pose, seed);
    return;
  }

  const Eigen::VectorXd& samples = dof_range[static_cast<std::size_t>(loop_level)];
  const Eigen::Index n = samples.size();
  for (Eigen::Index i = 0; i < n; ++i)
  {
    tool_pose(loop_level) = samples(i);
    nested_ik(solutions, loop_level + 1, dof_range, target_tool_tip, tool_pose, seed);
  }
}

void RTPInvKin::ikAt(IKSolutions& solutions,
                     const Eigen::Isometry3d& target_tool_tip,
                     Eigen::VectorXd& tool_pose,
                     const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  TESSERACT_THREAD_LOCAL tesseract::common::TransformMap tool_poses;
  tool_poses.clear();
  tool_fwd_kin_->calcFwdKin(tool_poses, tool_pose);
  Eigen::Isometry3d tool_tf = tool_poses.at(tool_tip_link_);

  // T_manip_tip = T_target_tool_tip * (T_manip_tip_to_tool_base * T_tool_base_to_tool_tip(q_tool))^-1
  Eigen::Isometry3d robot_target_pose = target_tool_tip * (manip_tip_to_tool_base_ * tool_tf).inverse();

  if (robot_target_pose.translation().norm() > manip_reach_)
    return;

  TESSERACT_THREAD_LOCAL tesseract::common::TransformMap robot_target_poses;
  robot_target_poses.clear();
  robot_target_poses[manip_tip_link_] = robot_target_pose;

  auto robot_dof = manip_inv_kin_->numJoints();
  auto tool_dof = static_cast<Eigen::Index>(tool_pose.size());

  TESSERACT_THREAD_LOCAL IKSolutions robot_solution_set;
  robot_solution_set.clear();
  manip_inv_kin_->calcInvKin(robot_solution_set, robot_target_poses, seed.head(robot_dof));
  if (robot_solution_set.empty())
    return;

  for (const auto& robot_solution : robot_solution_set)
  {
    solutions.emplace_back(robot_dof + tool_dof);
    Eigen::VectorXd& full_sol = solutions.back();
    full_sol.head(robot_dof) = robot_solution;
    full_sol.tail(tool_dof) = tool_pose;
  }
}

void RTPInvKin::calcInvKin(IKSolutions& solutions,
                           const tesseract::common::TransformMap& tip_link_poses,
                           const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(tool_tip_link_) != tip_link_poses.end());                      // NOLINT
  assert(std::abs(1.0 - tip_link_poses.at(tool_tip_link_).matrix().determinant()) < 1e-6);  // NOLINT

  return calcInvKinHelper(solutions, tip_link_poses, seed);  // NOLINT
}

std::vector<std::string> RTPInvKin::getJointNames() const { return joint_names_; }

Eigen::Index RTPInvKin::numJoints() const { return dof_; }

std::string RTPInvKin::getBaseLinkName() const { return manip_inv_kin_->getBaseLinkName(); }

std::string RTPInvKin::getWorkingFrame() const { return manip_inv_kin_->getBaseLinkName(); }

std::vector<std::string> RTPInvKin::getTipLinkNames() const { return { tool_tip_link_ }; }

std::string RTPInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract::kinematics
