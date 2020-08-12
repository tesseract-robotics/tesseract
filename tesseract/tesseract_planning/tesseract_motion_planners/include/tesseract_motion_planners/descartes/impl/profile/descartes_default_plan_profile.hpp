/**
 * @file descartes_default_plan_profile.hpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>

#include <descartes_light/interface/collision_interface.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/compound_edge_evaluator.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::Isometry3d& cartesian_waypoint,
                                                   const Instruction& parent_instruction,
                                                   const ManipulatorInfo& manip_info,
                                                   const std::vector<std::string>& active_links,
                                                   int index)
{
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.isEmpty() && base_instruction->getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction->getManipulatorInfo().isEmpty()) ? manip_info : base_instruction->getManipulatorInfo();

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain) */
  auto it = std::find(active_links.begin(), active_links.end(), prob.manip_inv_kin->getBaseLinkName());
  if (it != active_links.end() && prob.manip_inv_kin->getBaseLinkName() != mi.working_frame)
    throw std::runtime_error("DescartesDefaultPlanProfile: Assigned dynamic waypoint but parent instruction working is "
                             "not set to the base link of manipulator!");

  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                         active_links,
                                                         prob.manip_inv_kin->getJointNames(),
                                                         collision_safety_margin,
                                                         debug);

  Eigen::Isometry3d manip_baselink_to_waypoint = Eigen::Isometry3d::Identity();
  if (it == active_links.end())
  {
    // Check if the waypoint is not relative to the manipulator base coordinate system
    Eigen::Isometry3d world_to_waypoint = cartesian_waypoint;
    if (!mi.working_frame.empty())
      world_to_waypoint = prob.env_state->link_transforms.at(mi.working_frame) * cartesian_waypoint;

    Eigen::Isometry3d world_to_base_link = prob.env_state->link_transforms.at(prob.manip_inv_kin->getBaseLinkName());
    manip_baselink_to_waypoint = world_to_base_link.inverse() * world_to_waypoint;
  }

  auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(
      manip_baselink_to_waypoint, target_pose_sampler, prob.manip_inv_kin, ci, mi.tcp, allow_collision, is_valid);
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
        compound_evaluator->push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_saftey_margin,
                                                                         edge_longest_valid_segment_length,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(
      descartes_core::TimingConstraint<FloatType>(static_cast<FloatType>(timing_constraint)));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(&tesseract_kinematics::isWithinLimits<FloatType>,
                         std::placeholders::_1,
                         prob.manip_inv_kin->getLimits().joint_limits);

  prob.num_threads = num_threads;
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::VectorXd& joint_waypoint,
                                                   const Instruction& /*parent_instruction*/,
                                                   const ManipulatorInfo& /*manip_info*/,
                                                   const std::vector<std::string>& active_links,
                                                   int index)
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();
        compound_evaluator->push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
        compound_evaluator->push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironmentConst(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_saftey_margin,
                                                                         edge_longest_valid_segment_length,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  // Add timing Constraint
  prob.timing_constraints.push_back(
      descartes_core::TimingConstraint<FloatType>(static_cast<FloatType>(timing_constraint)));

  // Add isValid function
  if (is_valid == nullptr)
    is_valid = std::bind(&tesseract_kinematics::isWithinLimits<FloatType>,
                         std::placeholders::_1,
                         prob.manip_inv_kin->getLimits().joint_limits);

  prob.num_threads = num_threads;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
