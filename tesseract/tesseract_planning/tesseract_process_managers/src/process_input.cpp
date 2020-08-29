/**
 * @file process_input.cpp
 * @brief Process input
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#include <vector>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_input.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
static const ManipulatorInfo EMPTY_MANIPULATOR_INFO;
static const PlannerProfileRemapping EMPTY_PROFILE_MAPPING;

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           const ManipulatorInfo& manip_info,
                           Instruction* seed)
  : tesseract(std::move(tesseract))
  , instruction(instruction)
  , manip_info(manip_info)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , results(seed)
  , start_instruction(NullInstruction())
  , end_instruction(NullInstruction())
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           const ManipulatorInfo& manip_info,
                           const PlannerProfileRemapping& plan_profile_remapping,
                           const PlannerProfileRemapping& composite_profile_remapping,
                           Instruction* seed)
  : tesseract(std::move(tesseract))
  , instruction(instruction)
  , manip_info(manip_info)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , results(seed)
  , start_instruction(NullInstruction())
  , end_instruction(NullInstruction())
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           const PlannerProfileRemapping& plan_profile_remapping,
                           const PlannerProfileRemapping& composite_profile_remapping,
                           Instruction* seed)
  : tesseract(std::move(tesseract))
  , instruction(instruction)
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , results(seed)
  , start_instruction(NullInstruction())
  , end_instruction(NullInstruction())
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract, const Instruction* instruction, Instruction* seed)
  : tesseract(std::move(tesseract))
  , instruction(instruction)
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , results(seed)
  , start_instruction(NullInstruction())
  , end_instruction(NullInstruction())
{
}

ProcessInput ProcessInput::operator[](std::size_t index)
{
  if (isCompositeInstruction(*instruction))
  {
    const auto* composite_instruction = instruction->cast_const<CompositeInstruction>();
    auto* composite_seed = results->cast<CompositeInstruction>();
    return ProcessInput(tesseract, &(composite_instruction->at(index)), manip_info, &((*composite_seed)[index]));
  }

  if (index > 0)
    CONSOLE_BRIDGE_logWarn("ProcessInput[] called with index > 0 when component instructions are not "
                           "CompositeInstructions");

  return ProcessInput(tesseract, nullptr, manip_info, nullptr);
}

std::size_t ProcessInput::size()
{
  //  if (isCompositeInstruction(instruction) && isCompositeInstruction(results))
  //    assert(instruction.cast_const<CompositeInstruction>()->size() == results.cast<CompositeInstruction>()->size());

  if (isCompositeInstruction(*instruction))
    return instruction->cast_const<CompositeInstruction>()->size();

  return 0;
}

}  // namespace tesseract_planning
