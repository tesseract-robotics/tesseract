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
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_kinematics/core/validate.h>

namespace tesseract_planning
{
template <typename FloatType>
inline std::shared_ptr<DescartesProblem<FloatType>>
DefaultDescartesProblemGenerator(const std::string& name,
                                 const PlannerRequest& request,
                                 const DescartesPlanProfileMap<FloatType>& plan_profiles)
{
  auto prob = std::make_shared<DescartesProblem<FloatType>>();

  // Clear descartes data
  prob->edge_evaluators.clear();
  prob->samplers.clear();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  // Get Manipulator Information
  prob->manip_fwd_kin =
      request.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(composite_mi.manipulator);
  if (composite_mi.manipulator_ik_solver.empty())
    prob->manip_inv_kin =
        request.tesseract->getEnvironment()->getManipulatorManager()->getInvKinematicSolver(composite_mi.manipulator);
  else
    prob->manip_inv_kin = request.tesseract->getEnvironment()->getManipulatorManager()->getInvKinematicSolver(
        composite_mi.manipulator, composite_mi.manipulator_ik_solver);

  if (!prob->manip_fwd_kin)
  {
    CONSOLE_BRIDGE_logError("No Forward Kinematics solver found");
    return prob;
  }
  if (!prob->manip_inv_kin)
  {
    CONSOLE_BRIDGE_logError("No Inverse Kinematics solver found");
    return prob;
  }
  prob->env_state = request.env_state;
  prob->tesseract = request.tesseract;

  // Process instructions
  if (!tesseract_kinematics::checkKinematics(prob->manip_fwd_kin, prob->manip_inv_kin))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");

  std::vector<std::string> active_link_names = prob->manip_inv_kin->getActiveLinkNames();
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      request.tesseract->getEnvironment()->getSceneGraph(), active_link_names, request.env_state->link_transforms);
  const std::vector<std::string>& active_links = adjacency_map->getActiveLinkNames();

  // Flatten the input for planning
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat = flattenProgramToPattern(request.seed, request.instructions);

  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  std::string profile;
  Waypoint start_waypoint = NullWaypoint();
  Instruction placeholder_instruction = NullInstruction();
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isPlanInstruction(*start_instruction))
    {
      const auto* temp = start_instruction->cast_const<PlanInstruction>();
      assert(temp->isStart());
      start_waypoint = temp->getWaypoint();
      profile = temp->getProfile();
    }
    else
    {
      throw std::runtime_error("Descartes DefaultProblemGenerator: Unsupported start instruction type!");
    }
    ++start_index;
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(prob->manip_inv_kin->getJointNames());
    StateWaypoint swp(prob->manip_inv_kin->getJointNames(), current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  profile = getProfileString(profile, name, request.plan_profile_remapping);
  auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
      profile, plan_profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
  if (!cur_plan_profile)
    throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

  // Add start waypoint
  if (isCartesianWaypoint(start_waypoint))
  {
    const auto* cwp = start_waypoint.cast_const<Eigen::Isometry3d>();
    cur_plan_profile->apply(*prob, *cwp, *start_instruction, composite_mi, active_links, index);
  }
  else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
  {
    const Eigen::VectorXd& position = getJointPosition(start_waypoint);
    cur_plan_profile->apply(*prob, position, *start_instruction, composite_mi, active_links, index);
  }
  else
  {
    throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
  }

  ++index;

  // Transform plan instructions into descartes samplers
  for (std::size_t i = start_index; i < instructions_flat.size(); ++i)
  {
    const auto& instruction = instructions_flat[i].get();
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.template cast_const<PlanInstruction>();

      // If plan instruction has manipulator information then use it over the one provided by the composite.
      ManipulatorInfo mi = composite_mi.getCombined(plan_instruction->getManipulatorInfo());
      Eigen::Isometry3d tcp = request.tesseract->findTCP(mi);

      // The seed should always have a start instruction
      assert(request.seed.hasStartInstruction());
      std::size_t seed_idx = (start_index == 0) ? i + 1 : i;
      assert(isCompositeInstruction(seed_flat[seed_idx].get()));
      const auto* seed_composite =
          seed_flat[seed_idx].get().template cast_const<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite->size());

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      profile = getProfileString(profile, name, request.plan_profile_remapping);
      auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
          profile, plan_profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
      if (!cur_plan_profile)
        throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

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
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!prob->manip_fwd_kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

            prev_pose = prob->env_state->link_transforms.at(prob->manip_fwd_kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*prob, poses[p], *plan_instruction, composite_mi, active_links, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*prob, *cur_wp, *plan_instruction, composite_mi, active_links, index);

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction->getWaypoint());
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!prob->manip_fwd_kin->calcFwdKin(cur_pose, cur_position))
            throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

          cur_pose = prob->env_state->link_transforms.at(prob->manip_fwd_kin->getBaseLinkName()) * cur_pose * tcp;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!prob->manip_fwd_kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

            prev_pose = prob->env_state->link_transforms.at(prob->manip_fwd_kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*prob, poses[p], *plan_instruction, composite_mi, active_links, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*prob, cur_position, *plan_instruction, composite_mi, active_links, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction->getWaypoint());

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*prob, cur_position, *plan_instruction, composite_mi, active_links, index);

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
          cur_plan_profile->apply(*prob, *cur_wp, *plan_instruction, composite_mi, active_links, index);

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

      start_waypoint = plan_instruction->getWaypoint();
      start_instruction = &instruction;
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return prob;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DEFAULT_PROBLEM_GENERATOR_H
