/**
 * @file get_instruction_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#include <algorithm>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
const Instruction* getFirstInstructionHelper(const CompositeInstruction& composite_instruction,
                                             locateFilterFn locate_filter,
                                             bool process_child_composites,
                                             bool first_composite)
{
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
      return &(composite_instruction.getStartInstruction());

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        return &instruction;

      if (isCompositeInstruction(instruction))
      {
        const Instruction* result = getFirstInstructionHelper(
            *(instruction.cast_const<CompositeInstruction>()), locate_filter, process_child_composites, false);
        if (result)
          return result;
      }
    }

    return nullptr;
  }

  for (const auto& instruction : composite_instruction)
    if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
      return &instruction;

  return nullptr;
}

Instruction* getFirstInstructionHelper(CompositeInstruction& composite_instruction,
                                       locateFilterFn locate_filter,
                                       bool process_child_composites,
                                       bool first_composite)
{
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
      return &(composite_instruction.getStartInstruction());

  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        return &instruction;

      if (isCompositeInstruction(instruction))
      {
        Instruction* result = getFirstInstructionHelper(
            *(instruction.cast<CompositeInstruction>()), locate_filter, process_child_composites, false);
        if (result)
          return result;
      }
    }

    return nullptr;
  }

  for (auto& instruction : composite_instruction)
    if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
      return &instruction;

  return nullptr;
}

const Instruction* getLastInstructionHelper(const CompositeInstruction& composite_instruction,
                                            locateFilterFn locate_filter,
                                            bool process_child_composites,
                                            bool first_composite)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
        return &(*it);

      if (isCompositeInstruction(*it))
      {
        const Instruction* result = getLastInstructionHelper(
            *(it->cast_const<CompositeInstruction>()), locate_filter, process_child_composites, false);
        if (result)
          return result;
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (!locate_filter ||
          locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
        return &(composite_instruction.getStartInstruction());

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
      return &(*it);

  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
      return &(composite_instruction.getStartInstruction());

  return nullptr;
}

Instruction* getLastInstructionHelper(CompositeInstruction& composite_instruction,
                                      locateFilterFn locate_filter,
                                      bool process_child_composites,
                                      bool first_composite)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
        return &(*it);

      if (isCompositeInstruction(*it))
      {
        Instruction* result = getLastInstructionHelper(
            *(it->cast<CompositeInstruction>()), locate_filter, process_child_composites, false);
        if (result)
          return result;
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (!locate_filter ||
          locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
        return &(composite_instruction.getStartInstruction());

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (!locate_filter || locate_filter(*it, composite_instruction, first_composite))
      return &(*it);

  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
      return &(composite_instruction.getStartInstruction());

  return nullptr;
}

long getInstructionCountHelper(const CompositeInstruction& composite_instruction,
                               locateFilterFn locate_filter,
                               bool process_child_composites,
                               bool first_composite)
{
  long cnt = 0;
  if (composite_instruction.hasStartInstruction())
    if (!locate_filter ||
        locate_filter(composite_instruction.getStartInstruction(), composite_instruction, first_composite))
      ++cnt;

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (!locate_filter || locate_filter(instruction, composite_instruction, first_composite))
        ++cnt;

      if (isCompositeInstruction(instruction))
        cnt += getInstructionCountHelper(
            *(instruction.cast_const<CompositeInstruction>()), locate_filter, process_child_composites, false);
    }
    return cnt;
  }

  cnt += std::count_if(composite_instruction.begin(), composite_instruction.end(), [=](const auto& i) {
    return (!locate_filter || locate_filter(i, composite_instruction, first_composite));
  });

  return cnt;
}

const Instruction* getFirstInstruction(const CompositeInstruction& composite_instruction,
                                       locateFilterFn locate_filter,
                                       bool process_child_composites)
{
  return getFirstInstructionHelper(composite_instruction, locate_filter, process_child_composites, true);
}

Instruction* getFirstInstruction(CompositeInstruction& composite_instruction,
                                 locateFilterFn locate_filter,
                                 bool process_child_composites)
{
  return getFirstInstructionHelper(composite_instruction, locate_filter, process_child_composites, true);
}

const Instruction* getLastInstruction(const CompositeInstruction& composite_instruction,
                                      locateFilterFn locate_filter,
                                      bool process_child_composites)
{
  return getLastInstructionHelper(composite_instruction, locate_filter, process_child_composites, true);
}

Instruction* getLastInstruction(CompositeInstruction& composite_instruction,
                                locateFilterFn locate_filter,
                                bool process_child_composites)
{
  return getLastInstructionHelper(composite_instruction, locate_filter, process_child_composites, true);
}

long getInstructionCount(const CompositeInstruction& composite_instruction,
                         locateFilterFn locate_filter,
                         bool process_child_composites)
{
  return getInstructionCountHelper(composite_instruction, locate_filter, process_child_composites, true);
}
}  // namespace tesseract_planning
