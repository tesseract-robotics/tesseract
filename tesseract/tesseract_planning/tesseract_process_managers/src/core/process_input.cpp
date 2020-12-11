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

#include <tesseract_process_managers/core/process_input.h>
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
                           Instruction* seed,
                           bool has_seed,
                           ProfileDictionary::ConstPtr profiles)
  : tesseract(std::move(tesseract))
  , manip_info(manip_info)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           const ManipulatorInfo& manip_info,
                           const PlannerProfileRemapping& plan_profile_remapping,
                           const PlannerProfileRemapping& composite_profile_remapping,
                           Instruction* seed,
                           bool has_seed,
                           ProfileDictionary::ConstPtr profiles)
  : tesseract(std::move(tesseract))
  , manip_info(manip_info)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           const PlannerProfileRemapping& plan_profile_remapping,
                           const PlannerProfileRemapping& composite_profile_remapping,
                           Instruction* seed,
                           bool has_seed,
                           ProfileDictionary::ConstPtr profiles)
  : tesseract(std::move(tesseract))
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

ProcessInput::ProcessInput(tesseract::Tesseract::ConstPtr tesseract,
                           const Instruction* instruction,
                           Instruction* seed,
                           bool has_seed,
                           ProfileDictionary::ConstPtr profiles)
  : tesseract(std::move(tesseract))
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

ProcessInput ProcessInput::operator[](std::size_t index)
{
  ProcessInput pi(*this);
  pi.instruction_indice_.push_back(index);

  return pi;
}

std::size_t ProcessInput::size()
{
  const Instruction* ci = instruction_;
  for (const auto& i : instruction_indice_)
  {
    if (isCompositeInstruction(*ci))
    {
      const auto* composite = ci->cast_const<CompositeInstruction>();
      ci = &(composite->at(i));
    }
    else
    {
      return 0;
    }
  }

  if (isCompositeInstruction(*ci))
  {
    const auto* composite = ci->cast_const<CompositeInstruction>();
    return composite->size();
  }

  return 0;
}

const Instruction* ProcessInput::getInstruction() const
{
  const Instruction* ci = instruction_;
  for (const auto& i : instruction_indice_)
  {
    if (isCompositeInstruction(*ci))
    {
      const auto* composite = ci->cast_const<CompositeInstruction>();
      ci = &(composite->at(i));
    }
    else
    {
      return nullptr;
    }
  }
  return ci;
}

Instruction* ProcessInput::getResults()
{
  Instruction* ci = results_;
  for (const auto& i : instruction_indice_)
  {
    if (isCompositeInstruction(*ci))
    {
      auto* composite = ci->cast<CompositeInstruction>();
      ci = &(composite->at(i));
    }
    else
    {
      return nullptr;
    }
  }
  return ci;
}

ProcessInterface::Ptr ProcessInput::getProcessInterface() { return interface_; }

bool ProcessInput::isAborted() const { return interface_->isAborted(); }

void ProcessInput::abort() { interface_->abort(); }

void ProcessInput::setStartInstruction(Instruction start)
{
  start_instruction_ = start;
  start_instruction_indice_.clear();
}

void ProcessInput::setStartInstruction(std::vector<std::size_t> start)
{
  start_instruction_indice_ = start;
  start_instruction_ = NullInstruction();
}

Instruction ProcessInput::getStartInstruction() const
{
  if (!isNullInstruction(start_instruction_))
    return start_instruction_;

  if (start_instruction_indice_.empty())
    return NullInstruction();

  Instruction* ci = results_;
  for (const auto& i : start_instruction_indice_)
  {
    if (isCompositeInstruction(*ci))
    {
      auto* composite = ci->cast<CompositeInstruction>();
      ci = &(composite->at(i));
    }
    else
    {
      return NullInstruction();
    }
  }

  if (isCompositeInstruction(*ci))
    return *getLastMoveInstruction(*(ci->cast<CompositeInstruction>()));

  return *ci;
}

void ProcessInput::setEndInstruction(Instruction end)
{
  end_instruction_ = end;
  end_instruction_indice_.clear();
}

void ProcessInput::setEndInstruction(std::vector<std::size_t> end)
{
  end_instruction_indice_ = end;
  end_instruction_ = NullInstruction();
}

Instruction ProcessInput::getEndInstruction() const
{
  if (!isNullInstruction(end_instruction_))
    return end_instruction_;

  if (end_instruction_indice_.empty())
    return NullInstruction();

  Instruction* ci = results_;
  for (const auto& i : end_instruction_indice_)
  {
    if (isCompositeInstruction(*ci))
    {
      auto* composite = ci->cast<CompositeInstruction>();
      ci = &(composite->at(i));
    }
    else
    {
      return NullInstruction();
    }
  }

  if (isCompositeInstruction(*ci))
  {
    auto* composite = ci->cast<CompositeInstruction>();
    return composite->getStartInstruction();
  }

  return *ci;
}

void ProcessInput::addProcessInfo(const ProcessInfo::ConstPtr& process_info)
{
  process_infos_->addProcessInfo(process_info);
}

ProcessInfo::ConstPtr ProcessInput::getProcessInfo(const std::size_t& index) const { return (*process_infos_)[index]; }

std::map<std::size_t, ProcessInfo::ConstPtr> ProcessInput::getProcessInfoMap() const
{
  return process_infos_->getProcessInfoMap();
}
}  // namespace tesseract_planning
