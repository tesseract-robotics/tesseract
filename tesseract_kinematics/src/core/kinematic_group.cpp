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
KinematicGroup::KinematicGroup(InverseKinematics::UPtr inv_kin,
                               const tesseract_scene_graph::SceneGraph& scene_graph,
                               const tesseract_scene_graph::SceneState& scene_state)
  : JointGroup(inv_kin->getName(), inv_kin->getJointNames(), scene_graph, scene_state)
{
  inv_kin_ = std::move(inv_kin);

  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
  std::string working_frame = inv_kin_->getWorkingFrame();
  auto it = std::find(active_link_names.begin(), active_link_names.end(), working_frame);
  if (it == active_link_names.end())
  {
    working_frames_.insert(working_frames_.end(), static_link_names_.begin(), static_link_names_.end());
    inv_working_frames_map_[working_frame] = working_frame;
    for (const auto& static_link_name : static_link_names_)
      inv_working_frames_map_[static_link_name] = working_frame;
  }
  else
  {
    std::vector<std::string> child_link_names = scene_graph.getLinkChildrenNames(working_frame);
    working_frames_.push_back(working_frame);
    working_frames_.insert(working_frames_.end(), child_link_names.begin(), child_link_names.end());

    inv_working_frames_map_[working_frame] = working_frame;
    for (const auto& child : child_link_names)
      inv_working_frames_map_[child] = working_frame;
  }

  tip_link_names_ = inv_kin_->getTipLinkNames();
  for (const auto& tip_link : inv_kin_->getTipLinkNames())
  {
    std::vector<std::string> child_link_names = scene_graph.getLinkChildrenNames(working_frame);
    tip_link_names_.insert(tip_link_names_.end(), child_link_names.begin(), child_link_names.end());

    inv_tip_links_map_[tip_link] = tip_link;
    for (const auto& child : child_link_names)
      inv_tip_links_map_[child] = tip_link;
  }

  inv_to_fwd_base_ = state_.link_transforms.at(inv_kin_->getBaseLinkName()).inverse() *
                     state_.link_transforms.at(state_solver_->getBaseLinkName());
}

KinematicGroup::KinematicGroup(const KinematicGroup& other) : JointGroup(other) { *this = other; }

KinematicGroup& KinematicGroup::operator=(const KinematicGroup& other)
{
  JointGroup::operator=(other);
  inv_kin_ = other.inv_kin_->clone();
  inv_to_fwd_base_ = other.inv_to_fwd_base_;
  working_frames_ = other.working_frames_;
  tip_link_names_ = other.tip_link_names_;
  inv_working_frames_map_ = other.inv_working_frames_map_;
  return *this;
}

IKSolutions KinematicGroup::calcInvKin(const KinGroupIKInputs& tip_link_poses,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  // Convert to IK Inputs
  IKInput ik_inputs;
  for (const auto& tip_link_pose : tip_link_poses)
  {
    assert(std::find(tip_link_names_.begin(), tip_link_names_.end(), tip_link_pose.tip_link_name) !=
           tip_link_names_.end());
    assert(std::find(working_frames_.begin(), working_frames_.end(), tip_link_pose.working_frame) !=
           working_frames_.end());

    std::string inv_tip_link = inv_tip_links_map_.at(tip_link_pose.tip_link_name);
    std::string inv_working_frame = inv_working_frames_map_.at(tip_link_pose.working_frame);

    Eigen::Isometry3d wf_pose_offset =
        state_.link_transforms.at(inv_working_frame).inverse() * state_.link_transforms.at(tip_link_pose.working_frame);
    Eigen::Isometry3d tl_pose_offset =
        state_.link_transforms.at(inv_tip_link).inverse() * state_.link_transforms.at(tip_link_pose.tip_link_name);

    /** @todo Check this math (Levi) */
    Eigen::Isometry3d inv_pose = wf_pose_offset * tip_link_pose.pose * tl_pose_offset.inverse();
    ik_inputs[inv_tip_link] = inv_pose;
  }

  IKSolutions solutions = inv_kin_->calcInvKin(ik_inputs, seed);
  IKSolutions solutions_filtered;
  solutions_filtered.reserve(solutions.size());
  for (auto& solution : solutions)
  {
    if (tesseract_common::satisfiesPositionLimits(solution, limits_.joint_limits))
      solutions_filtered.push_back(solution);
  }

  return solutions_filtered;
}

std::vector<std::string> KinematicGroup::getWorkingFrames() const { return working_frames_; }

std::vector<std::string> KinematicGroup::getTipLinkNames() const { return tip_link_names_; }
}  // namespace tesseract_kinematics
