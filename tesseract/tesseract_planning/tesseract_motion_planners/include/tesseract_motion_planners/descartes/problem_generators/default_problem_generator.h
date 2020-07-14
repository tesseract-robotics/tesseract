/**
 * @file default_problem_generator.h
 * @brief Generates a Descartes Problem
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DEFAULT_PROBLEM_GENERATOR_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DEFAULT_PROBLEM_GENERATOR_H

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_kinematics/core/validate.h>

namespace tesseract_planning
{
template <typename FloatType>
inline DescartesProblem<FloatType>
DefaultDescartesProblemGenerator(const PlannerRequest& request, const DescartesPlanProfileMap<FloatType>& plan_profiles)
{
  DescartesProblem<FloatType> prob;

  // Clear descartes data
  prob.edge_evaluators.clear();
  prob.timing_constraints.clear();
  prob.samplers.clear();

  // Get Manipulator Information
  prob.manip_fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(request.manipulator);
  if (request.manipulator_ik_solver.empty())
    prob.manip_inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(request.manipulator);
  else
    prob.manip_inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(
        request.manipulator, request.manipulator_ik_solver);

  if (!prob.manip_fwd_kin)
  {
    CONSOLE_BRIDGE_logError("No Forward Kinematics solver found");
    return prob;
  }
  if (!prob.manip_inv_kin)
  {
    CONSOLE_BRIDGE_logError("No Inverse Kinematics solver found");
    return prob;
  }
  prob.env_state = request.env_state;
  prob.tesseract = request.tesseract;

  // Process instructions
  if (!tesseract_kinematics::checkKinematics(prob.manip_fwd_kin, prob.manip_inv_kin))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");

  std::vector<std::string> active_link_names = prob.manip_inv_kin->getActiveLinkNames();
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      request.tesseract->getEnvironmentConst()->getSceneGraph(), active_link_names, request.env_state->link_transforms);
  const std::vector<std::string>& active_links = adjacency_map->getActiveLinkNames();

  // Check and make sure it does not contain any composite instruction
  for (const auto& instruction : request.instructions)
    if (isCompositeInstruction(instruction))
      throw std::runtime_error("Descartes planner does not support child composite instructions.");

  Waypoint start_waypoint = NullWaypoint();
  if (request.instructions.hasStartWaypoint())
  {
    start_waypoint = request.instructions.getStartWaypoint();
  }
  else
  {
    Eigen::VectorXd current_jv = prob.env_state->getJointValues(prob.manip_inv_kin->getJointNames());
    JointWaypoint temp(current_jv);
    temp.joint_names = prob.manip_inv_kin->getJointNames();
    start_waypoint = temp;
  }

  // Transform plan instructions into descartes samplers
  int index = 0;
  bool found_plan_instruction{ false };
  for (std::size_t i = 0; i < request.instructions.size(); ++i)
  {
    const auto& instruction = request.instructions[i];
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.template cast_const<PlanInstruction>();

      assert(isCompositeInstruction(request.seed[i]));
      const auto* seed_composite = request.seed[i].template cast_const<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite->size());

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      if (profile.empty())
        profile = "DEFAULT";

      typename DescartesPlanProfile<FloatType>::Ptr cur_plan_profile{ nullptr };
      auto it = plan_profiles.find(profile);
      if (it == plan_profiles.end())
        cur_plan_profile = std::make_shared<DescartesDefaultPlanProfile<FloatType>>();
      else
        cur_plan_profile = it->second;

      if (!found_plan_instruction)
      {
        // If this is the first plan instruction we reduce the interpolate cnt because the seed includes the start state
        --interpolate_cnt;

        // Add start waypoint
        if (isCartesianWaypoint(start_waypoint))
        {
          const auto* cwp = start_waypoint.cast_const<Eigen::Isometry3d>();
          cur_plan_profile->apply(prob, *cwp, *plan_instruction, active_links, index);
        }
        else if (isJointWaypoint(start_waypoint))
        {
          const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
          cur_plan_profile->apply(prob, *jwp, *plan_instruction, active_links, index);
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
        }

        ++index;
      }

      if (plan_instruction->isLinear())
      {
        if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp =
              plan_instruction->getWaypoint().template cast_const<tesseract_planning::CartesianWaypoint>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint))
          {
            const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
            if (!prob.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
              throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

            prev_pose = prob.env_state->link_transforms.at(prob.manip_fwd_kin->getBaseLinkName()) * prev_pose *
                        plan_instruction->getTCP();
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(prob, poses[p], *plan_instruction, active_links, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().template cast_const<JointWaypoint>();
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!prob.manip_fwd_kin->calcFwdKin(cur_pose, *cur_wp))
            throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

          cur_pose = prob.env_state->link_transforms.at(prob.manip_fwd_kin->getBaseLinkName()) * cur_pose *
                     plan_instruction->getTCP();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint))
          {
            const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
            if (!prob.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
              throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

            prev_pose = prob.env_state->link_transforms.at(prob.manip_fwd_kin->getBaseLinkName()) * prev_pose *
                        plan_instruction->getTCP();
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(prob, poses[p], *plan_instruction, active_links, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().template cast_const<tesseract_planning::JointWaypoint>();

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp =
              plan_instruction->getWaypoint().template cast_const<tesseract_planning::CartesianWaypoint>();

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else
      {
        throw std::runtime_error("DescartesMotionPlannerConfig: Unsupported!");
      }

      found_plan_instruction = true;
      start_waypoint = plan_instruction->getWaypoint();
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return prob;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DEFAULT_PROBLEM_GENERATOR_H
