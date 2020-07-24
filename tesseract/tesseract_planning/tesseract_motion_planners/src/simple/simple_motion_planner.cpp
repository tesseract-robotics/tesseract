/**
 * @file simple_motion_planner.cpp
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 *
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_environment/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace trajopt;

namespace tesseract_planning
{
SimpleMotionPlannerStatusCategory::SimpleMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& SimpleMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string SimpleMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case ErrorInvalidInput:
    {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

SimpleMotionPlanner::SimpleMotionPlanner(std::string name)
  : MotionPlanner(std::move(name)), status_category_(std::make_shared<const SimpleMotionPlannerStatusCategory>(name_))
{
  plan_profiles["DEFAULT"] = std::make_shared<SimplePlannerDefaultPlanProfile>();
}

bool SimpleMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing planning is not implemented yet");
  return false;
}

void SimpleMotionPlanner::clear() {}

tesseract_common::StatusCode SimpleMotionPlanner::solve(const PlannerRequest& request,
                                                        PlannerResponse& response,
                                                        bool /*verbose*/) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(SimpleMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Initialize
  tesseract_environment::EnvState::ConstPtr current_state = request.env_state;
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin =
      request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  Waypoint start_waypoint{ NullWaypoint() };

  // Create seed
  CompositeInstruction seed;

  // Get the start waypoint/instruction
  MoveInstruction start_instruction = getStartInstruction(request, current_state, fwd_kin);
  start_waypoint = start_instruction.getWaypoint();
  seed.setStartInstruction(start_instruction);

  // Process the instructions into the seed
  seed = processCompositeInstruction(request.instructions, start_waypoint, request);

  // Set start instruction
  seed.setStartInstruction(start_instruction);

  // Fill out the response
  response.results = seed;

  // Return success
  response.status = tesseract_common::StatusCode(SimpleMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

MoveInstruction
SimpleMotionPlanner::getStartInstruction(const PlannerRequest& request,
                                         const tesseract_environment::EnvState::ConstPtr& current_state,
                                         const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin) const
{
  // Create start instruction
  Waypoint start_waypoint{ NullWaypoint() };
  MoveInstruction start_instruction_seed(start_waypoint, MoveInstructionType::START);

  if (request.instructions.hasStartInstruction())
  {
    assert(isMoveInstruction(request.instructions.getStartInstruction()));
    const auto* start_instruction = request.instructions.getStartInstruction().cast_const<MoveInstruction>();
    assert(start_instruction->isStart());
    start_waypoint = start_instruction->getWaypoint();

    start_instruction_seed.setWaypoint(start_waypoint);
    start_instruction_seed.setManipulatorInfo(start_instruction->getManipulatorInfo());
  }
  else
  {
    JointWaypoint temp(current_state->getJointValues(fwd_kin->getJointNames()));
    temp.joint_names = fwd_kin->getJointNames();
    start_waypoint = temp;

    start_instruction_seed.setWaypoint(start_waypoint);
  }
  return start_instruction_seed;
}

CompositeInstruction SimpleMotionPlanner::processCompositeInstruction(const CompositeInstruction& instructions,
                                                                      const Waypoint& initial_start_waypoint,
                                                                      const PlannerRequest& request) const
{
  Waypoint start_waypoint = initial_start_waypoint;
  CompositeInstruction seed;
  for (const auto& instruction : instructions)
  {
    if (isCompositeInstruction(instruction))
    {
      seed.push_back(
          processCompositeInstruction(*instruction.cast_const<CompositeInstruction>(), start_waypoint, request));
    }
    else if (isPlanInstruction(instruction))
    {
      auto plan_instruction = *instruction.cast_const<PlanInstruction>();
      if (plan_instruction.getManipulatorInfo().isEmpty())
        plan_instruction.setManipulatorInfo(instructions.getManipulatorInfo());

      bool is_cwp1 = isCartesianWaypoint(start_waypoint);
      bool is_jwp1 = isJointWaypoint(start_waypoint);
      bool is_cwp2 = isCartesianWaypoint(plan_instruction.getWaypoint());
      bool is_jwp2 = isJointWaypoint(plan_instruction.getWaypoint());

      assert(is_cwp1 || is_jwp1);
      assert(is_cwp2 || is_jwp2);

      std::string profile = plan_instruction.getProfile();
      if (profile.empty())
        profile = "DEFAULT";
      auto current_profile = plan_profiles.find(profile);

      // TODO: Catch exceptions
      if (plan_instruction.isLinear())
      {
        if (is_cwp1 && is_cwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_cwp = plan_instruction.getWaypoint().cast_const<CartesianWaypoint>();

          auto step = current_profile->second->cart_cart_linear(*pre_cwp, *cur_cwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_cwp1 && is_jwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_jwp = plan_instruction.getWaypoint().cast_const<JointWaypoint>();

          auto step = current_profile->second->cart_joint_linear(*pre_cwp, *cur_jwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_cwp2 && is_jwp1)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction.getWaypoint().cast_const<CartesianWaypoint>();

          auto step = current_profile->second->joint_cart_linear(*pre_jwp, *cur_cwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_jwp1 && is_jwp2)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_jwp = plan_instruction.getWaypoint().cast_const<JointWaypoint>();

          auto step = current_profile->second->joint_joint_linear(*pre_jwp, *cur_jwp, plan_instruction, request);
          seed.push_back(step);
        }
        else
        {
          throw std::runtime_error("SimpleMotionPlanner::processCompositeInstruction: Unsupported waypoints provided!");
        }
      }
      else if (plan_instruction.isFreespace())
      {
        if (is_cwp1 && is_cwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_cwp = plan_instruction.getWaypoint().cast_const<CartesianWaypoint>();

          auto step = current_profile->second->cart_cart_freespace(*pre_cwp, *cur_cwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_cwp1 && is_jwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_jwp = plan_instruction.getWaypoint().cast_const<JointWaypoint>();

          auto step = current_profile->second->cart_joint_freespace(*pre_cwp, *cur_jwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_cwp2 && is_jwp1)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction.getWaypoint().cast_const<CartesianWaypoint>();

          auto step = current_profile->second->joint_cart_freespace(*pre_jwp, *cur_cwp, plan_instruction, request);
          seed.push_back(step);
        }
        else if (is_jwp1 && is_jwp2)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_jwp = plan_instruction.getWaypoint().cast_const<JointWaypoint>();

          auto step = current_profile->second->joint_joint_freespace(*pre_jwp, *cur_jwp, plan_instruction, request);
          seed.push_back(step);
        }
        else
        {
          throw std::runtime_error("SimpleMotionPlanner::processCompositeInstruction: Unsupported waypoints provided!");
        }
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }
      start_waypoint = plan_instruction.getWaypoint();
    }
    else  // isPlanInstruction(instruction)
    {
      seed.push_back(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

bool SimpleMotionPlanner::checkUserInput(const PlannerRequest& request) const
{
  // Check that parameters are valid
  if (request.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In SimpleMotionPlanner: tesseract is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("SimpleMotionPlanner requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
