/**
 * @file flatten_utils.cpp
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

#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @param first_composite Indicates if the composite being processed is the top most composite
 */
void flattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                   CompositeInstruction& composite,
                   flattenFilterFn filter,
                   bool first_composite)
{
  if (composite.hasStartInstruction())
    if (!filter || filter(composite.getStartInstruction(), composite, first_composite))
      flattened.emplace_back(composite.getStartInstruction());

  for (auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(i, composite, first_composite))
          flattened.emplace_back(i);

      flattenHelper(flattened, *(i.cast<CompositeInstruction>()), filter, false);
    }
    else if (!filter || (filter && filter(i, composite, first_composite)))
    {
      flattened.emplace_back(i);
    }
  }
}

std::vector<std::reference_wrapper<Instruction>> flatten(CompositeInstruction& composite_instruction,
                                                         flattenFilterFn filter)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  flattenHelper(flattened, composite_instruction, filter, true);
  return flattened;
}

/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @param first_composite Indicates if the composite being processed is the top most composite
 */
void flattenHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                   const CompositeInstruction& composite,
                   flattenFilterFn filter,
                   bool first_composite)
{
  if (composite.hasStartInstruction())
    if (!filter || filter(composite.getStartInstruction(), composite, first_composite))
      flattened.emplace_back(composite.getStartInstruction());

  for (const auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(i, composite, first_composite))
          flattened.emplace_back(i);

      flattenHelper(flattened, *(i.cast_const<CompositeInstruction>()), filter, false);
    }
    else if (!filter || filter(i, composite, first_composite))
    {
      flattened.emplace_back(i);
    }
  }
}

std::vector<std::reference_wrapper<const Instruction>> flatten(const CompositeInstruction& composite_instruction,
                                                               flattenFilterFn filter)
{
  std::vector<std::reference_wrapper<const Instruction>> flattened;
  flattenHelper(flattened, composite_instruction, filter, true);
  return flattened;
}

/**
 * @brief Helper function used by FlattenToPattern. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @param first_composite Indicates if the composite being processed is the top most composite
 */
void flattenToPatternHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                            CompositeInstruction& composite,
                            const CompositeInstruction& pattern,
                            flattenFilterFn filter,
                            bool first_composite)
{
  if (composite.size() != pattern.size() || composite.hasStartInstruction() != pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  if (composite.hasStartInstruction())
    if (!filter || filter(composite.getStartInstruction(), composite, first_composite))
      flattened.emplace_back(composite.getStartInstruction());

  for (std::size_t i = 0; i < pattern.size(); i++)
  {
    if (isCompositeInstruction(pattern.at(i)) && isCompositeInstruction(composite[i]))
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(composite[i], composite, first_composite))
          flattened.emplace_back(composite[i]);

      flattenToPatternHelper(flattened,
                             *(composite[i].cast<CompositeInstruction>()),
                             *pattern.at(i).cast_const<CompositeInstruction>(),
                             filter,
                             false);
    }
    else
    {
      flattened.emplace_back(composite[i]);
    }
  }
}

std::vector<std::reference_wrapper<Instruction>> flattenToPattern(CompositeInstruction& composite_instruction,
                                                                  const CompositeInstruction& pattern,
                                                                  flattenFilterFn filter)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  flattenToPatternHelper(flattened, composite_instruction, pattern, filter, true);
  return flattened;
}

/**
 * @brief Helper function used by FlattenToPattern. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param filter Used to filter only what should be considered. Should return true to include otherwise false
 * @param first_composite Indicates if the composite being processed is the top most composite
 */
void flattenToPatternHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                            const CompositeInstruction& composite,
                            const CompositeInstruction& pattern,
                            flattenFilterFn filter,
                            bool first_composite)
{
  if (composite.size() != pattern.size() || composite.hasStartInstruction() != pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  if (composite.hasStartInstruction())
    if (!filter || filter(composite.getStartInstruction(), composite, first_composite))
      flattened.emplace_back(composite.getStartInstruction());

  for (std::size_t i = 0; i < pattern.size(); i++)
  {
    if (isCompositeInstruction(pattern.at(i)) && isCompositeInstruction(composite[i]))
    {
      // By default composite instructions will not be stored just it children, but this allows for the filter to
      // indicate that they should be stored.
      if (filter)
        if (filter(composite[i], composite, first_composite))
          flattened.emplace_back(composite[i]);

      flattenToPatternHelper(flattened,
                             *(composite[i].cast_const<CompositeInstruction>()),
                             *pattern.at(i).cast_const<CompositeInstruction>(),
                             filter,
                             false);
    }
    else
    {
      flattened.emplace_back(composite[i]);
    }
  }
}

std::vector<std::reference_wrapper<const Instruction>>
flattenToPattern(const CompositeInstruction& composite_instruction,
                 const CompositeInstruction& pattern,
                 flattenFilterFn filter)
{
  std::vector<std::reference_wrapper<const Instruction>> flattened;
  flattenToPatternHelper(flattened, composite_instruction, pattern, filter, true);
  return flattened;
}

}  // namespace tesseract_planning
