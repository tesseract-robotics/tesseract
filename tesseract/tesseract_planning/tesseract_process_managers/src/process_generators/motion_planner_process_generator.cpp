/**
 * @file motion_planner_process_generator.cpp
 * @brief Perform motion planning
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
MotionPlannerProcessGenerator::MotionPlannerProcessGenerator(std::shared_ptr<MotionPlanner> planner)
  : planner_(planner), name_(planner->getName())
{
}

const std::string& MotionPlannerProcessGenerator::getName() const { return name_; }

std::function<void()> MotionPlannerProcessGenerator::generateTask(ProcessInput input, std::size_t unique_id)
{
  return [=]() { process(input, unique_id); };
}

std::function<int()> MotionPlannerProcessGenerator::generateConditionalTask(ProcessInput input, std::size_t unique_id)
{
  return [=]() { return conditionalProcess(input, unique_id); };
}

int MotionPlannerProcessGenerator::conditionalProcess(ProcessInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<MotionPlannerProcessInfo>(unique_id, name_);
  info->return_value = 0;
  input.addProcessInfo(info);

  // --------------------
  // Check that inputs are valid
  // --------------------
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    info->message =
        "Input instructions to MotionPlannerProcessGenerator: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return 0;
  }

  Instruction* input_results = input.getResults();
  if (!isCompositeInstruction(*input_results))
  {
    info->message = "Input seed to MotionPlannerProcessGenerator: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return 0;
  }

  // Make a non-const copy of the input instructions to update the start/end
  CompositeInstruction instructions = *input_instruction->cast_const<CompositeInstruction>();
  assert(!(input.manip_info.empty() && input.manip_info.empty()));
  instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(input.manip_info));

  // If the start and end waypoints need to be updated prior to planning
  Instruction start_instruction = input.getStartInstruction();
  Instruction end_instruction = input.getEndInstruction();

  if (!isNullInstruction(start_instruction))
  {
    // add start
    if (isCompositeInstruction(start_instruction))
    {
      // if provided a composite instruction as the start instruction it will extract the last move instruction
      const auto* ci = start_instruction.cast_const<CompositeInstruction>();
      auto* lmi = getLastMoveInstruction(*ci);
      assert(lmi != nullptr);
      assert(isMoveInstruction(*lmi));
      PlanInstruction si(lmi->getWaypoint(), PlanInstructionType::START, lmi->getProfile(), lmi->getManipulatorInfo());
      instructions.setStartInstruction(si);
    }
    else
    {
      assert(isPlanInstruction(start_instruction) || isMoveInstruction(start_instruction));
      if (isPlanInstruction(start_instruction))
      {
        instructions.setStartInstruction(start_instruction);
        instructions.getStartInstruction().cast<PlanInstruction>()->setPlanType(PlanInstructionType::START);
      }
      else if (isMoveInstruction(start_instruction))
      {
        auto* lmi = start_instruction.cast<MoveInstruction>();
        PlanInstruction si(
            lmi->getWaypoint(), PlanInstructionType::START, lmi->getProfile(), lmi->getManipulatorInfo());
        instructions.setStartInstruction(si);
      }
    }
  }
  if (!isNullInstruction(end_instruction))
  {
    // add end
    if (isCompositeInstruction(end_instruction))
    {
      // if provided a composite instruction as the end instruction it will extract the first move instruction
      const auto* ci = end_instruction.cast_const<CompositeInstruction>();
      auto* fmi = getFirstMoveInstruction(*ci);
      assert(fmi != nullptr);
      assert(isMoveInstruction(*fmi));
      getLastPlanInstruction(instructions)->setWaypoint(fmi->getWaypoint());
    }
    else
    {
      assert(isMoveInstruction(end_instruction) || isPlanInstruction(end_instruction));
      auto* lpi = getLastPlanInstruction(instructions);
      if (isMoveInstruction(end_instruction))
      {
        lpi->setWaypoint(end_instruction.cast_const<MoveInstruction>()->getWaypoint());
      }
      else if (isPlanInstruction(end_instruction))
      {
        lpi->setWaypoint(end_instruction.cast_const<PlanInstruction>()->getWaypoint());
      }
    }
  }

  // It should always have a start instruction which required by the motion planners
  assert(instructions.hasStartInstruction());

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = *input_results->cast<CompositeInstruction>();
  request.env_state = input.env->getCurrentState();
  request.env = input.env;
  request.instructions = instructions;
  request.plan_profile_remapping = input.plan_profile_remapping;
  request.composite_profile_remapping = input.composite_profile_remapping;

  // --------------------
  // Fill out response
  // --------------------
  PlannerResponse response;

  bool verbose = false;
  if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
    verbose = true;
  auto status = planner_->solve(request, response, verbose);

  // --------------------
  // Verify Success
  // --------------------
  if (status)
  {
    *input_results = response.results;
    CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
    info->return_value = 1;
    return 1;
  }

  CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                           planner_->getName().c_str(),
                           status.message().c_str(),
                           input_instruction->getDescription().c_str());
  info->message = status.message();
  return 0;
}

void MotionPlannerProcessGenerator::process(ProcessInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

MotionPlannerProcessInfo::MotionPlannerProcessInfo(std::size_t unique_id, std::string name)
  : ProcessInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning
