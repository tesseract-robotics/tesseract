/**
 * @file default_problem_generator.cpp
 * @brief Generates a OMPL problem from a planner request
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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

#include <tesseract_kinematics/core/validate.h>
#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
OMPLProblem::Ptr CreateOMPLSubProblem(const PlannerRequest& request,
                                      const tesseract_kinematics::ForwardKinematics::Ptr& manip_fwd_kin,
                                      const tesseract_kinematics::InverseKinematics::Ptr& manip_inv_kin,
                                      const std::vector<std::string>& active_link_names)
{
  auto sub_prob = std::make_unique<OMPLProblem>();
  sub_prob->tesseract = request.tesseract;
  sub_prob->env_state = request.env_state;
  sub_prob->state_solver = request.tesseract->getEnvironment()->getStateSolver();
  sub_prob->state_solver->setState(request.env_state->joints);
  sub_prob->manip_fwd_kin = manip_fwd_kin;
  sub_prob->manip_inv_kin = manip_inv_kin;
  sub_prob->contact_checker = request.tesseract->getEnvironment()->getDiscreteContactManager();
  sub_prob->contact_checker->setCollisionObjectsTransform(request.env_state->link_transforms);
  sub_prob->contact_checker->setActiveCollisionObjects(active_link_names);
  return sub_prob;
}

std::vector<OMPLProblem::Ptr> DefaultOMPLProblemGenerator(const std::string& name,
                                                          const PlannerRequest& request,
                                                          const OMPLPlanProfileMap& plan_profiles)
{
  std::vector<OMPLProblem::Ptr> problem;
  std::vector<std::string> active_link_names_;
  tesseract_kinematics::ForwardKinematics::Ptr manip_fwd_kin_;
  tesseract_kinematics::InverseKinematics::Ptr manip_inv_kin_;

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());

  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  manip_fwd_kin_ =
      request.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(composite_mi.manipulator);
  if (composite_mi.manipulator_ik_solver.empty())
    manip_inv_kin_ =
        request.tesseract->getEnvironment()->getManipulatorManager()->getInvKinematicSolver(composite_mi.manipulator);
  else
    manip_inv_kin_ = request.tesseract->getEnvironment()->getManipulatorManager()->getInvKinematicSolver(
        composite_mi.manipulator, composite_mi.manipulator_ik_solver);
  if (!manip_fwd_kin_)
  {
    CONSOLE_BRIDGE_logError("No Forward Kinematics solver found");
    return problem;
  }
  if (!manip_inv_kin_)
  {
    CONSOLE_BRIDGE_logError("No Inverse Kinematics solver found");
    return problem;
  }
  // Process instructions
  if (!tesseract_kinematics::checkKinematics(manip_fwd_kin_, manip_inv_kin_))
  {
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");
  }

  // Get Active Link Names
  {
    std::vector<std::string> active_link_names = manip_inv_kin_->getActiveLinkNames();
    auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        request.tesseract->getEnvironment()->getSceneGraph(), active_link_names, request.env_state->link_transforms);
    active_link_names_ = adjacency_map->getActiveLinkNames();
  }

  // Check and make sure it does not contain any composite instruction
  for (const auto& instruction : request.instructions)
    if (isCompositeInstruction(instruction))
      throw std::runtime_error("OMPL planner does not support child composite instructions.");

  int index = 0;
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
    }
    else
    {
      throw std::runtime_error("OMPL DefaultProblemGenerator: Unsupported start instruction type!");
    }
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(manip_fwd_kin_->getJointNames());
    StateWaypoint swp(manip_fwd_kin_->getJointNames(), current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  // Transform plan instructions into ompl problem
  for (std::size_t i = 0; i < request.instructions.size(); ++i)
  {
    const auto& instruction = request.instructions[i];
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();

      assert(isCompositeInstruction(request.seed[i]));
      const auto* seed_composite = request.seed[i].cast_const<tesseract_planning::CompositeInstruction>();

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      profile = getProfileString(profile, name, request.plan_profile_remapping);
      auto cur_plan_profile =
          getProfile<OMPLPlanProfile>(profile, plan_profiles, std::make_shared<OMPLDefaultPlanProfile>());
      if (!cur_plan_profile)
        throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Invalid profile");

      /** @todo Should check that the joint names match the order of the manipulator */
      OMPLProblem::Ptr sub_prob = CreateOMPLSubProblem(request, manip_fwd_kin_, manip_inv_kin_, active_link_names_);
      cur_plan_profile->setup(*sub_prob);
      sub_prob->n_output_states = static_cast<int>(seed_composite->size()) + 1;

      if (plan_instruction->isLinear())
      {
        /** @todo Add support for linear motion to ompl planner */
        if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          problem.push_back(nullptr);
          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(start_waypoint))
        {
          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          // const Eigen::VectorXd& position = getJointPosition(start_waypoint);
          problem.push_back(nullptr);
          ++index;
        }
        else
        {
          throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction->getWaypoint());
          cur_plan_profile->applyGoalStates(
              *sub_prob, cur_position, *plan_instruction, composite_mi, active_link_names_, index);

          if (index == 0)
          {
            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
            {
              const Eigen::VectorXd& prev_position = getJointPosition(start_waypoint);
              cur_plan_profile->applyStartStates(
                  *sub_prob, prev_position, *start_instruction, composite_mi, active_link_names_, index);
            }
            else if (isCartesianWaypoint(start_waypoint))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::CartesianWaypoint>();
              cur_plan_profile->applyStartStates(
                  *sub_prob, *prev_wp, *start_instruction, composite_mi, active_link_names_, index);
            }
            else
            {
              throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
            }

            problem.push_back(std::move(sub_prob));
            ++index;
          }
          else
          {
            /** @todo Update. Extract the solution for the previous plan and set as the start */
            assert(false);
          }
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          cur_plan_profile->applyGoalStates(
              *sub_prob, *cur_wp, *plan_instruction, composite_mi, active_link_names_, index);

          if (index == 0)
          {
            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
            {
              const Eigen::VectorXd& prev_position = getJointPosition(start_waypoint);
              cur_plan_profile->applyStartStates(
                  *sub_prob, prev_position, *start_instruction, composite_mi, active_link_names_, index);
            }
            else if (isCartesianWaypoint(start_waypoint))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::CartesianWaypoint>();
              cur_plan_profile->applyStartStates(
                  *sub_prob, *prev_wp, *start_instruction, composite_mi, active_link_names_, index);
            }
            else
            {
              throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
            }
          }
          else
          {
            /** @todo Update. Extract the solution for the previous plan and set as the start */
            assert(false);
          }

          problem.push_back(std::move(sub_prob));
          ++index;
        }
      }
      else
      {
        throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Unsupported!");
      }

      start_waypoint = plan_instruction->getWaypoint(); /** @todo need to extract the solution */
      start_instruction = &instruction;
    }
  }

  return problem;
}
}  // namespace tesseract_planning
