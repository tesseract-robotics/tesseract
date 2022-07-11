/**
 * @file kinematic_group.cpp
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_common/utils.h>

#include <tesseract_scene_graph/kdl_parser.h>

namespace tesseract_kinematics
{
KinematicGroup::KinematicGroup(std::string name,
                               std::vector<std::string> joint_names,
                               InverseKinematics::UPtr inv_kin,
                               const tesseract_scene_graph::SceneGraph& scene_graph,
                               const tesseract_scene_graph::SceneState& scene_state)
  : JointGroup(std::move(name), joint_names, scene_graph, scene_state)
  , joint_names_(std::move(joint_names))
  , inv_kin_(std::move(inv_kin))
{
  std::vector<std::string> inv_kin_joint_names = inv_kin_->getJointNames();

  if (static_cast<Eigen::Index>(joint_names_.size()) != inv_kin_->numJoints())
    throw std::runtime_error("KinematicGroup: joint_names is not the correct size");

  if (!tesseract_common::isIdentical(joint_names_, inv_kin_joint_names, false))
    throw std::runtime_error("KinematicGroup: joint_names does not match same names in inverse kinematics object!");

  reorder_required_ = !tesseract_common::isIdentical(joint_names_, inv_kin_joint_names, true);

  if (reorder_required_)
  {
    inv_kin_joint_map_.reserve(joint_names_.size());
    for (const auto& joint_name : joint_names_)
      inv_kin_joint_map_.push_back(std::distance(
          inv_kin_joint_names.begin(), std::find(inv_kin_joint_names.begin(), inv_kin_joint_names.end(), joint_name)));
  }

  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
  std::string working_frame = inv_kin_->getWorkingFrame();

  auto it = std::find(active_link_names.begin(), active_link_names.end(), working_frame);
  if (it == active_link_names.end())
  {
    working_frames_.reserve(static_link_names_.size());
    std::copy(static_link_names_.begin(), static_link_names_.end(), std::back_inserter(working_frames_));
  }
  else
  {
    std::vector<std::string> child_link_names = scene_graph.getLinkChildrenNames(working_frame);
    working_frames_.reserve(child_link_names.size() + 1);
    working_frames_.push_back(working_frame);
    std::copy(child_link_names.begin(), child_link_names.end(), std::back_inserter(working_frames_));
  }

  for (const auto& tip_link : inv_kin_->getTipLinkNames())
  {
    inv_tip_links_map_[tip_link] = tip_link;
    for (const auto& child : scene_graph.getLinkChildrenNames(tip_link))
      inv_tip_links_map_[child] = tip_link;
  }

  inv_to_fwd_base_ = state_.link_transforms.at(inv_kin_->getBaseLinkName()).inverse() *
                     state_.link_transforms.at(state_solver_->getBaseLinkName());

  if (static_link_names_.size() + active_link_names.size() != scene_graph.getLinks().size())
    throw std::runtime_error("KinematicGroup: Static link names are not correct!");
}

KinematicGroup::KinematicGroup(const KinematicGroup& other) : JointGroup(other) { *this = other; }

KinematicGroup& KinematicGroup::operator=(const KinematicGroup& other)
{
  JointGroup::operator=(other);
  joint_names_ = other.joint_names_;
  reorder_required_ = other.reorder_required_;
  inv_kin_joint_map_ = other.inv_kin_joint_map_;
  inv_kin_ = other.inv_kin_->clone();
  inv_to_fwd_base_ = other.inv_to_fwd_base_;
  working_frames_ = other.working_frames_;
  inv_tip_links_map_ = other.inv_tip_links_map_;
  return *this;
}

IKSolutions KinematicGroup::calcInvKin(const KinGroupIKInputs& tip_link_poses,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  // Convert to IK Inputs
  tesseract_common::TransformMap ik_inputs;
  for (const auto& tip_link_pose : tip_link_poses)
  {
    assert(std::find(working_frames_.begin(), working_frames_.end(), tip_link_pose.working_frame) !=
           working_frames_.end());
    assert(std::abs(1.0 - tip_link_pose.pose.matrix().determinant()) < 1e-6);  // NOLINT

    // The IK Solvers tip link and working frame
    std::string ik_solver_tip_link = inv_tip_links_map_.at(tip_link_pose.tip_link_name);
    std::string working_frame = inv_kin_->getWorkingFrame();

    // Get transform from working frame to user working frame (reference frame for the target IK pose)
    const Eigen::Isometry3d& world_to_user_wf = state_.link_transforms.at(tip_link_pose.working_frame);
    const Eigen::Isometry3d& world_to_wf = state_.link_transforms.at(working_frame);
    const Eigen::Isometry3d wf_to_user_wf = world_to_wf.inverse() * world_to_user_wf;

    // Get the transform from IK solver tip link to the user tip link
    const Eigen::Isometry3d& world_to_user_tl = state_.link_transforms.at(tip_link_pose.tip_link_name);
    const Eigen::Isometry3d& world_to_tl = state_.link_transforms.at(ik_solver_tip_link);
    const Eigen::Isometry3d tl_to_user_tl = world_to_tl.inverse() * world_to_user_tl;

    // Get the transformation from the IK solver working frame to the IK solver tip frame
    const Eigen::Isometry3d& user_wf_to_user_tl = tip_link_pose.pose;  // an unnecessary but helpful alias
    const Eigen::Isometry3d wf_to_tl = wf_to_user_wf * user_wf_to_user_tl * tl_to_user_tl.inverse();

    ik_inputs[ik_solver_tip_link] = wf_to_tl;
  }

  // format seed for inverse kinematic solver
  if (reorder_required_)
  {
    Eigen::VectorXd ordered_seed = seed;
    for (Eigen::Index i = 0; i < inv_kin_->numJoints(); ++i)
      ordered_seed(inv_kin_joint_map_[static_cast<std::size_t>(i)]) = seed(i);

    IKSolutions solutions = inv_kin_->calcInvKin(ik_inputs, ordered_seed);
    IKSolutions solutions_filtered;
    solutions_filtered.reserve(solutions.size());
    for (const auto& solution : solutions)
    {
      Eigen::VectorXd ordered_sol = solution;
      for (Eigen::Index i = 0; i < inv_kin_->numJoints(); ++i)
        ordered_sol(i) = solution(inv_kin_joint_map_[static_cast<std::size_t>(i)]);

      if (tesseract_common::satisfiesPositionLimits<double>(ordered_sol, limits_.joint_limits))
        solutions_filtered.push_back(ordered_sol);
    }

    return solutions_filtered;
  }

  IKSolutions solutions = inv_kin_->calcInvKin(ik_inputs, seed);
  IKSolutions solutions_filtered;
  solutions_filtered.reserve(solutions.size());
  for (auto& solution : solutions)
  {
    if (tesseract_common::satisfiesPositionLimits<double>(solution, limits_.joint_limits))
      solutions_filtered.push_back(solution);
  }

  return solutions_filtered;
}

IKSolutions KinematicGroup::calcInvKin(const KinGroupIKInput& tip_link_pose,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  return calcInvKin(KinGroupIKInputs{ tip_link_pose }, seed);
}

std::vector<std::string> KinematicGroup::getAllValidWorkingFrames() const { return working_frames_; }

std::vector<std::string> KinematicGroup::getAllPossibleTipLinkNames() const
{
  std::vector<std::string> ik_tip_links;
  ik_tip_links.reserve(inv_tip_links_map_.size());
  for (const auto& pair : inv_tip_links_map_)
    ik_tip_links.push_back(pair.first);

  return ik_tip_links;
}
}  // namespace tesseract_kinematics
