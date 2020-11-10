/**
 * @file fix_state_bounds_process_generator.cpp
 * @brief Process generator that changes the plan instructions to make push them back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_process_managers/process_generators/fix_state_bounds_process_generator.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
FixStateBoundsProcessGenerator::FixStateBoundsProcessGenerator(std::string name) : name_(std::move(name))
{
  // Register default profile
  composite_profiles["DEFAULT"] = std::make_shared<FixStateBoundsProfile>();
}

const std::string& FixStateBoundsProcessGenerator::getName() const { return name_; }

std::function<void()> FixStateBoundsProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> FixStateBoundsProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int FixStateBoundsProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("Input instruction to FixStateBounds must be a composite instruction");
    return 0;
  }

  const auto* ci = input_instruction->cast_const<CompositeInstruction>();
  const ManipulatorInfo& manip_info = input.manip_info;
  const auto fwd_kin =
      input.tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);

  // Get Composite profile
  std::string profile = ci->getProfile();
  if (profile.empty())
    profile = "DEFAULT";

  // Check for remapping of composite profile
  {
    auto remap = input.composite_profile_remapping.find(name_);
    if (remap != input.composite_profile_remapping.end())
    {
      auto p = remap->second.find(profile);
      if (p != remap->second.end())
        profile = p->second;
    }
  }

  // Get the parameters associated with this profile
  typename FixStateBoundsProfile::Ptr cur_composite_profile{ nullptr };
  auto it = composite_profiles.find(profile);
  if (it == composite_profiles.end())
    cur_composite_profile = std::make_shared<FixStateBoundsProfile>();
  else
    cur_composite_profile = it->second;

  if (cur_composite_profile->mode == FixStateBoundsProfile::Settings::DISABLED)
    return 1;

  auto limits = fwd_kin->getLimits().joint_limits;
  switch (cur_composite_profile->mode)
  {
    case FixStateBoundsProfile::Settings::START_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getFirstPlanInstruction(*ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        if (!isWithinJointLimits(mutable_instruction->getWaypoint(), limits))
        {
          CONSOLE_BRIDGE_logInform("FixStateBoundsProcessGenerator is modifying the const input instructions");
          if (!clampToJointLimits(
                  mutable_instruction->getWaypoint(), limits, cur_composite_profile->max_deviation_global))
            return 0;
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::END_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getLastPlanInstruction(*ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        if (!isWithinJointLimits(mutable_instruction->getWaypoint(), limits))
        {
          CONSOLE_BRIDGE_logInform("FixStateBoundsProcessGenerator is modifying the const input instructions");
          if (!clampToJointLimits(
                  mutable_instruction->getWaypoint(), limits, cur_composite_profile->max_deviation_global))
            return 0;
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::ALL:
    {
      auto flattened = flatten(*ci, planFilter);
      if (flattened.empty())
      {
        CONSOLE_BRIDGE_logWarn("FixStateBoundsProcessGenerator found no PlanInstructions to process");
        return 1;
      }

      bool outside_limits = false;
      for (const auto& instruction : flattened)
      {
        outside_limits |= isWithinJointLimits(instruction.get().cast_const<PlanInstruction>()->getWaypoint(), limits);
      }
      if (!outside_limits)
        break;

      CONSOLE_BRIDGE_logInform("FixStateBoundsProcessGenerator is modifying the const input instructions");
      for (const auto& instruction : flattened)
      {
        const Instruction* instr_const_ptr = &instruction.get();
        Instruction* mutable_instruction = const_cast<Instruction*>(instr_const_ptr);
        PlanInstruction* plan = mutable_instruction->cast<PlanInstruction>();
        if (!clampToJointLimits(plan->getWaypoint(), limits, cur_composite_profile->max_deviation_global))
          return 0;
      }
    }
    break;
    case FixStateBoundsProfile::Settings::DISABLED:
      return 1;
  }

  CONSOLE_BRIDGE_logDebug("FixStateBoundsProcessGenerator succeeded");
  return 1;
}

void FixStateBoundsProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool FixStateBoundsProcessGenerator::getAbort() const { return abort_; }
void FixStateBoundsProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
