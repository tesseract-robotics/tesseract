/**
 * @file kinematic_group.cpp
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <utility>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>

/**
 * @brief Returns the link IDs that comprise a fixed-joint kinematic tree with the input link
 * @details This method first traverses from the input link "up" the scene graph hierarchy, following the link's
 * in-bound joints, to find the input link's highest level parents that are connected to it by fixed joints. Next, it
 * identifies all the fixed-joint children of these highest level parent links. Together, the highest level parent links
 * and their fixed-joint children comprise a set of links (including the input link), between all of which exists a tree
 * of fixed joints. If the link does not have any fixed-joint parents, the link itself will be returned.
 * @param links
 * @param scene_graph
 * @return
 */
std::vector<tesseract::common::LinkId>
getLinksInFixedJointKinematicTree(const tesseract::common::LinkId& input_link,
                                  const tesseract::scene_graph::SceneGraph& scene_graph)
{
  // Create a set to contain the links of the fixed-joint kinematic tree
  std::set<tesseract::common::LinkId> fixed_joint_tree_links;

  // Create a list of links to traverse, populated initially with only the link in question
  std::vector<tesseract::common::LinkId> links = { input_link };

  while (!links.empty())
  {
    // Pop the back entry
    const tesseract::common::LinkId link = links.back();
    links.pop_back();

    // Traverse through the inbound joints until we find a non-fixed joint
    for (const std::shared_ptr<const tesseract::scene_graph::Joint>& joint : scene_graph.getInboundJoints(link.name()))
    {
      switch (joint->type)
      {
        case tesseract::scene_graph::JointType::FIXED:
          // Add this joint's parent link to the list of links to traverse
          links.push_back(joint->parent_link_id);
          break;
        default:
        {
          // Since this link is connected to its parent by a non-fixed joint:
          //   1. Add this link to the list of relatives
          fixed_joint_tree_links.insert(link);

          //   2. Add the fixed-joint children of this link to the list of relatives
          const std::vector<std::string> children = scene_graph.getLinkChildrenNames(link.name());
          for (const auto& child : children)
            fixed_joint_tree_links.insert(tesseract::common::LinkId::fromName(child));
        }
        break;
      }
    }
  }

  // Convert to vector
  std::vector<tesseract::common::LinkId> output;
  output.reserve(fixed_joint_tree_links.size());
  std::copy(fixed_joint_tree_links.begin(), fixed_joint_tree_links.end(), std::back_inserter(output));

  return output;
}

namespace tesseract::kinematics
{
using tesseract::common::LinkId;

KinGroupIKInput::KinGroupIKInput(Eigen::Isometry3d p, tesseract::common::LinkId wf, tesseract::common::LinkId tl)
  : pose(std::move(p)), working_frame(std::move(wf)), tip_link_id(std::move(tl))
{
}

KinematicGroup::KinematicGroup(std::string name,
                               const std::vector<std::string>& joint_names,
                               std::unique_ptr<InverseKinematics> inv_kin,
                               const tesseract::scene_graph::SceneGraph& scene_graph,
                               const tesseract::scene_graph::SceneState& scene_state)
  : JointGroup(std::move(name), joint_names, scene_graph, scene_state), inv_kin_(std::move(inv_kin))
{
  // joint_names parameter is still valid (JointGroup's by-value param made a copy)
  const auto& inv_kin_joint_ids = inv_kin_->getJointIds();
  std::vector<std::string> inv_kin_joint_names;
  inv_kin_joint_names.reserve(inv_kin_joint_ids.size());
  for (const auto& joint_id : inv_kin_joint_ids)
    inv_kin_joint_names.push_back(joint_id.name());

  if (static_cast<Eigen::Index>(joint_names.size()) != inv_kin_->numJoints())
    throw std::runtime_error("KinematicGroup: joint_names is not the correct size");

  if (!tesseract::common::isIdentical(joint_names, inv_kin_joint_names, false))
    throw std::runtime_error("KinematicGroup: joint_names does not match same names in inverse kinematics object!");

  reorder_required_ = !tesseract::common::isIdentical(joint_names, inv_kin_joint_names, true);

  if (reorder_required_)
  {
    inv_kin_joint_map_.reserve(joint_names.size());
    for (const auto& joint_name : joint_names)
      inv_kin_joint_map_.push_back(std::distance(
          inv_kin_joint_names.begin(), std::find(inv_kin_joint_names.begin(), inv_kin_joint_names.end(), joint_name)));
  }

  // Get the IK solver working frame name, and check that it exists in the scene state
  const auto working_frame_id = inv_kin_->getWorkingFrameId();
  if (state_.link_transforms.find(working_frame_id) == state_.link_transforms.end())
    throw std::runtime_error("Working frame '" + working_frame_id.name() + "' is not a link in the scene state");

  // Get the IK solver tip link names, and make sure they exist in the scene state
  const auto tip_links = inv_kin_->getTipLinkIds();
  for (const auto& link : tip_links)
    if (state_.link_transforms.find(link) == state_.link_transforms.end())
      throw std::runtime_error("Tip link '" + link.name() + "' is not a link in the scene state");

  // Configure working frames — use active_link_ids_ set (O(1)) instead of linear scan
  if (active_link_ids_.count(working_frame_id) == 0)
  {
    // Working frame is static — all static links are valid working frames
    working_frame_ids_.reserve(static_link_ids_.size());
    std::copy(static_link_ids_.begin(), static_link_ids_.end(), std::back_inserter(working_frame_ids_));
  }
  else
  {
    working_frame_ids_.push_back(working_frame_id);

    const std::vector<LinkId> working_frame_fixed_joint_kin_tree_links =
        getLinksInFixedJointKinematicTree(working_frame_id, scene_graph);
    for (const LinkId& link_id : working_frame_fixed_joint_kin_tree_links)
    {
      if (state_.link_transforms.find(link_id) == state_.link_transforms.end())
        throw std::runtime_error("Working frame '" + link_id.name() + "' is not a link in the scene state");

      working_frame_ids_.push_back(link_id);
    }
  }

  // Configure the tip link frames
  for (const auto& tip_link : tip_links)
  {
    const std::vector<LinkId> tip_link_fixed_joint_kin_tree_links =
        getLinksInFixedJointKinematicTree(tip_link, scene_graph);
    for (const LinkId& link_id : tip_link_fixed_joint_kin_tree_links)
    {
      if (state_.link_transforms.find(link_id) == state_.link_transforms.end())
        throw std::runtime_error("Tip link '" + link_id.name() + "' is not a link in the scene state");

      inv_tip_links_map_[link_id] = tip_link;
    }
  }

  inv_to_fwd_base_ = state_.link_transforms.at(inv_kin_->getBaseLinkId()).inverse() *
                     state_.link_transforms.at(state_solver_->getBaseLinkId());

  if (static_link_ids_.size() + active_link_ids_.size() != scene_graph.getLinks().size())
    throw std::runtime_error("KinematicGroup: Static link names are not correct!");
}

KinematicGroup::KinematicGroup(const KinematicGroup& other)
  : JointGroup(other)
  , reorder_required_(other.reorder_required_)
  , inv_kin_joint_map_(other.inv_kin_joint_map_)
  , inv_kin_(other.inv_kin_->clone())
  , inv_to_fwd_base_(other.inv_to_fwd_base_)
  , working_frame_ids_(other.working_frame_ids_)
  , inv_tip_links_map_(other.inv_tip_links_map_)
{
}

KinematicGroup::~KinematicGroup() = default;

KinematicGroup& KinematicGroup::operator=(const KinematicGroup& other)
{
  if (this == &other)
    return *this;

  JointGroup::operator=(other);
  reorder_required_ = other.reorder_required_;
  inv_kin_joint_map_ = other.inv_kin_joint_map_;
  inv_kin_ = other.inv_kin_->clone();
  inv_to_fwd_base_ = other.inv_to_fwd_base_;
  working_frame_ids_ = other.working_frame_ids_;
  inv_tip_links_map_ = other.inv_tip_links_map_;
  return *this;
}

IKSolutions KinematicGroup::calcInvKin(const KinGroupIKInputs& tip_link_poses,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  IKSolutions solutions;
  calcInvKin(solutions, tip_link_poses, seed);
  return solutions;
}

IKSolutions KinematicGroup::calcInvKin(const KinGroupIKInput& tip_link_pose,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  return calcInvKin(KinGroupIKInputs{ tip_link_pose }, seed);  // NOLINT
}

void KinematicGroup::calcInvKin(IKSolutions& solutions,
                                const KinGroupIKInputs& tip_link_poses,
                                const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  tesseract::common::LinkIdTransformMap ik_inputs;
  for (const auto& tip_link_pose : tip_link_poses)
  {
    // Check working frame
    if (std::find(working_frame_ids_.begin(), working_frame_ids_.end(), tip_link_pose.working_frame) ==
        working_frame_ids_.end())
    {
      std::stringstream ss;
      ss << "Specified working frame (" << tip_link_pose.working_frame.name()
         << ") is not in the list of identified working frames. Available working frames are: [";
      for (const auto& f : working_frame_ids_)
        ss << f.name() << ", ";
      ss << "].";
      throw std::runtime_error(ss.str());
    }

    // Check tip link
    auto tip_it = inv_tip_links_map_.find(tip_link_pose.tip_link_id);
    if (tip_it == inv_tip_links_map_.end())
    {
      std::stringstream ss;
      ss << "Failed to find specified tip link (" << tip_link_pose.tip_link_id.name()
         << "). Available tip links are: [";
      for (const auto& pair : inv_tip_links_map_)
        ss << pair.first.name() << ", ";
      ss << "].";
      throw std::runtime_error(ss.str());
    }

    assert(std::abs(1.0 - tip_link_pose.pose.matrix().determinant()) < 1e-6);  // NOLINT

    // The IK Solver's tip link (as LinkId) and working frame
    const LinkId ik_solver_tip_link_id = tip_it->second;
    const auto ik_working_frame_id = inv_kin_->getWorkingFrameId();

    const Eigen::Isometry3d& world_to_user_wf = state_.link_transforms.at(tip_link_pose.working_frame);
    const Eigen::Isometry3d& world_to_wf = state_.link_transforms.at(ik_working_frame_id);
    const Eigen::Isometry3d wf_to_user_wf = world_to_wf.inverse() * world_to_user_wf;

    const Eigen::Isometry3d& world_to_user_tl = state_.link_transforms.at(tip_link_pose.tip_link_id);
    const Eigen::Isometry3d& world_to_tl = state_.link_transforms.at(ik_solver_tip_link_id);
    const Eigen::Isometry3d tl_to_user_tl = world_to_tl.inverse() * world_to_user_tl;

    const Eigen::Isometry3d& user_wf_to_user_tl = tip_link_pose.pose;
    const Eigen::Isometry3d wf_to_tl = wf_to_user_wf * user_wf_to_user_tl * tl_to_user_tl.inverse();

    ik_inputs[ik_solver_tip_link_id] = wf_to_tl;
  }

  const long num_sol = static_cast<long>(solutions.size());

  if (reorder_required_)
  {
    Eigen::VectorXd ordered = seed;
    for (Eigen::Index i = 0; i < inv_kin_->numJoints(); ++i)
      ordered(inv_kin_joint_map_[static_cast<std::size_t>(i)]) = seed(i);

    inv_kin_->calcInvKin(solutions, ik_inputs, ordered);

    auto ne = std::remove_if(solutions.begin() + num_sol, solutions.end(), [&](Eigen::VectorXd& solution) {
      for (Eigen::Index i = 0; i < inv_kin_->numJoints(); ++i)
        ordered(i) = solution(inv_kin_joint_map_[static_cast<std::size_t>(i)]);

      tesseract::kinematics::harmonizeTowardMedian<double>(solution, redundancy_indices_, limits_.joint_limits);
      return (!tesseract::common::satisfiesLimits<double>(solution, limits_.joint_limits));
    });
    solutions.erase(ne, solutions.end());
    return;
  }

  inv_kin_->calcInvKin(solutions, ik_inputs, seed);
  auto ne = std::remove_if(solutions.begin() + num_sol, solutions.end(), [&](Eigen::VectorXd& solution) {
    tesseract::kinematics::harmonizeTowardMedian<double>(solution, redundancy_indices_, limits_.joint_limits);
    return (!tesseract::common::satisfiesLimits<double>(solution, limits_.joint_limits));
  });
  solutions.erase(ne, solutions.end());
}

void KinematicGroup::calcInvKin(IKSolutions& solutions,
                                const KinGroupIKInput& tip_link_pose,
                                const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  calcInvKin(solutions, KinGroupIKInputs{ tip_link_pose }, seed);  // NOLINT
}

std::vector<tesseract::common::LinkId> KinematicGroup::getAllValidWorkingFrameIds() const { return working_frame_ids_; }

std::vector<std::string> KinematicGroup::getAllValidWorkingFrames() const
{
  std::vector<std::string> frames;
  frames.reserve(working_frame_ids_.size());
  for (const auto& id : working_frame_ids_)
    frames.push_back(id.name());
  return frames;
}

std::vector<tesseract::common::LinkId> KinematicGroup::getAllPossibleTipLinkIds() const
{
  std::vector<tesseract::common::LinkId> ik_tip_links;
  ik_tip_links.reserve(inv_tip_links_map_.size());
  for (const auto& pair : inv_tip_links_map_)
    ik_tip_links.push_back(pair.first);
  return ik_tip_links;
}

std::vector<std::string> KinematicGroup::getAllPossibleTipLinkNames() const
{
  std::vector<std::string> ik_tip_links;
  ik_tip_links.reserve(inv_tip_links_map_.size());
  for (const auto& pair : inv_tip_links_map_)
    ik_tip_links.push_back(pair.first.name());
  return ik_tip_links;
}

const InverseKinematics& KinematicGroup::getInverseKinematics() const { return *inv_kin_; }

}  // namespace tesseract::kinematics
