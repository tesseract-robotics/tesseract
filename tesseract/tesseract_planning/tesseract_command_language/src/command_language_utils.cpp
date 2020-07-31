/**
 * @file command_language_utils.cpp
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

#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites)
{
  if (composite_instruction.hasStartInstruction())
    if (isPlanInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast_const<PlanInstruction>();

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
      {
        const PlanInstruction* result =
            getFirstPlanInstruction(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(instruction))
      {
        return instruction.cast_const<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if (isPlanInstruction(*it))
      return it->cast_const<PlanInstruction>();

  return nullptr;
}

const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(*it))
      {
        const PlanInstruction* result =
            getLastPlanInstruction(*(it->cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(*it))
      {
        return it->cast_const<PlanInstruction>();
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (isPlanInstruction(composite_instruction.getStartInstruction()))
        return composite_instruction.getStartInstruction().cast_const<PlanInstruction>();

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isPlanInstruction(*it))
      return it->cast_const<PlanInstruction>();

  if (composite_instruction.hasStartInstruction())
    if (isPlanInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast_const<PlanInstruction>();

  return nullptr;
}

PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (composite_instruction.hasStartInstruction())
    if (isPlanInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast<PlanInstruction>();

  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
      {
        PlanInstruction* result =
            getFirstPlanInstruction(*(instruction.cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(instruction))
      {
        return instruction.cast<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if (isPlanInstruction(*it))
      return it->cast<PlanInstruction>();

  return nullptr;
}

PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(*it))
      {
        PlanInstruction* result = getLastPlanInstruction(*(it->cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(*it))
      {
        return it->cast<PlanInstruction>();
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (isPlanInstruction(composite_instruction.getStartInstruction()))
        return composite_instruction.getStartInstruction().cast<PlanInstruction>();

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isPlanInstruction(*it))
      return it->cast<PlanInstruction>();

  if (composite_instruction.hasStartInstruction())
    if (isPlanInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast<PlanInstruction>();

  return nullptr;
}

const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites)
{
  if (composite_instruction.hasStartInstruction())
    if (isMoveInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast_const<MoveInstruction>();

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
      {
        const MoveInstruction* result =
            getFirstMoveInstruction(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(instruction))
      {
        return instruction.cast_const<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if (isMoveInstruction(*it))
      return it->cast_const<MoveInstruction>();

  return nullptr;
}

const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction,
                                              bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(*it))
      {
        const MoveInstruction* result =
            getLastMoveInstruction(*(it->cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(*it))
      {
        return it->cast_const<MoveInstruction>();
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (isMoveInstruction(composite_instruction.getStartInstruction()))
        return composite_instruction.getStartInstruction().cast_const<MoveInstruction>();

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isMoveInstruction(*it))
      return it->cast_const<MoveInstruction>();

  if (composite_instruction.hasStartInstruction())
    if (isMoveInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast_const<MoveInstruction>();

  return nullptr;
}

MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (composite_instruction.hasStartInstruction())
    if (isMoveInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast<MoveInstruction>();

  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
      {
        MoveInstruction* result =
            getFirstMoveInstruction(*(instruction.cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(instruction))
      {
        return instruction.cast<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if (isMoveInstruction(*it))
      return it->cast<MoveInstruction>();

  return nullptr;
}

MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(*it))
      {
        MoveInstruction* result = getLastMoveInstruction(*(it->cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(*it))
      {
        return it->cast<MoveInstruction>();
      }
    }

    if (composite_instruction.hasStartInstruction())
      if (isMoveInstruction(composite_instruction.getStartInstruction()))
        return composite_instruction.getStartInstruction().cast<MoveInstruction>();

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isMoveInstruction(*it))
      return it->cast<MoveInstruction>();

  if (composite_instruction.hasStartInstruction())
    if (isMoveInstruction(composite_instruction.getStartInstruction()))
      return composite_instruction.getStartInstruction().cast<MoveInstruction>();

  return nullptr;
}

long getMoveInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  long cnt = 0;
  if (composite_instruction.hasStartInstruction())
    if (isMoveInstruction(composite_instruction.getStartInstruction()))
      ++cnt;

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
        cnt += getMoveInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isMoveInstruction(instruction))
        ++cnt;
    }
    return cnt;
  }

  cnt += std::count_if(
      composite_instruction.begin(), composite_instruction.end(), [](const auto& i) { return isMoveInstruction(i); });

  return cnt;
}

long getPlanInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  long cnt = 0;
  if (composite_instruction.hasStartInstruction())
    if (isPlanInstruction(composite_instruction.getStartInstruction()))
      ++cnt;

  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
        cnt += getPlanInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isPlanInstruction(instruction))
        ++cnt;
    }
    return cnt;
  }

  cnt += std::count_if(
      composite_instruction.begin(), composite_instruction.end(), [](const auto& i) { return isPlanInstruction(i); });

  return cnt;
}

const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint)
{
  if (isJointWaypoint(waypoint))
    return *(waypoint.cast_const<JointWaypoint>());

  if (isStateWaypoint(waypoint))
    return waypoint.cast_const<StateWaypoint>()->position;

  throw std::runtime_error("Unsupported waypoint type.");
}

/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param include_composite If true, CompositeInstructions will be included in the final flattened vector
 */
void flattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                   CompositeInstruction& composite,
                   const bool& process_child_composites)
{
  for (auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      if (process_child_composites)
        flattened.emplace_back(i);
      flattenHelper(flattened, *(i.cast<CompositeInstruction>()), process_child_composites);
    }
    else
      flattened.emplace_back(i);
  }
}

std::vector<std::reference_wrapper<Instruction>> flatten(CompositeInstruction& composite_instruction,
                                                         const bool process_child_composites)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  if (composite_instruction.hasStartInstruction())
    flattened.emplace_back(composite_instruction.getStartInstruction());

  flattenHelper(flattened, composite_instruction, process_child_composites);
  return flattened;
}

/**
 * @brief Helper function used by Flatten. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param include_composite If true, CompositeInstructions will be included in the final flattened vector
 */
void flattenHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                   const CompositeInstruction& composite,
                   const bool& process_child_composites)
{
  for (auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      if (process_child_composites)
        flattened.emplace_back(i);
      flattenHelper(flattened, *(i.cast_const<CompositeInstruction>()), process_child_composites);
    }
    else
      flattened.emplace_back(i);
  }
}

std::vector<std::reference_wrapper<const Instruction>> flatten(const CompositeInstruction& composite_instruction,
                                                               const bool process_child_composites)
{
  std::vector<std::reference_wrapper<const Instruction>> flattened;

  if (composite_instruction.hasStartInstruction())
    flattened.emplace_back(composite_instruction.getStartInstruction());

  flattenHelper(flattened, composite_instruction, process_child_composites);
  return flattened;
}

/**
 * @brief Helper function used by FlattenToPattern. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param include_composite If true, CompositeInstructions will be included in the final flattened vector
 */
void flattenToPatternHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                            CompositeInstruction& composite,
                            const CompositeInstruction& pattern,
                            const bool& process_child_composites)
{
  if (composite.size() != pattern.size())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  for (std::size_t i = 0; i < pattern.size(); i++)
  {
    if (isCompositeInstruction(pattern.at(i)) && isCompositeInstruction(composite[i]))
    {
      if (process_child_composites)
        flattened.emplace_back(composite[i]);
      flattenToPatternHelper(flattened,
                             *(composite[i].cast<CompositeInstruction>()),
                             *pattern.at(i).cast_const<CompositeInstruction>(),
                             process_child_composites);
    }
    else
      flattened.emplace_back(composite[i]);
  }
}

std::vector<std::reference_wrapper<Instruction>> flattenToPattern(CompositeInstruction& composite_instruction,
                                                                  const CompositeInstruction& pattern,
                                                                  const bool process_child_composites)
{
  if (composite_instruction.size() != pattern.size() &&
      composite_instruction.hasStartInstruction() == pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return std::vector<std::reference_wrapper<Instruction>>();
  }

  std::vector<std::reference_wrapper<Instruction>> flattened;

  if (composite_instruction.hasStartInstruction())
    flattened.emplace_back(composite_instruction.getStartInstruction());

  flattenToPatternHelper(flattened, composite_instruction, pattern, process_child_composites);
  return flattened;
}

/**
 * @brief Helper function used by FlattenToPattern. Not intended for direct use
 * @param flattened Vector of instructions representing the full flattened composite
 * @param composite Composite instruction to be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @param include_composite If true, CompositeInstructions will be included in the final flattened vector
 */
void flattenToPatternHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                            const CompositeInstruction& composite,
                            const CompositeInstruction& pattern,
                            const bool& process_child_composites)
{
  if (composite.size() != pattern.size())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return;
  }

  for (std::size_t i = 0; i < pattern.size(); i++)
  {
    if (isCompositeInstruction(pattern.at(i)) && isCompositeInstruction(composite[i]))
    {
      if (process_child_composites)
        flattened.emplace_back(composite[i]);
      flattenToPatternHelper(flattened,
                             *(composite[i].cast_const<CompositeInstruction>()),
                             *pattern.at(i).cast_const<CompositeInstruction>(),
                             process_child_composites);
    }
    else
      flattened.emplace_back(composite[i]);
  }
}

std::vector<std::reference_wrapper<const Instruction>>
flattenToPattern(const CompositeInstruction& composite_instruction,
                 const CompositeInstruction& pattern,
                 const bool process_child_composites)
{
  if (composite_instruction.size() != pattern.size() &&
      composite_instruction.hasStartInstruction() == pattern.hasStartInstruction())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return std::vector<std::reference_wrapper<const Instruction>>();
  }

  std::vector<std::reference_wrapper<const Instruction>> flattened;

  if (composite_instruction.hasStartInstruction())
    flattened.emplace_back(composite_instruction.getStartInstruction());

  flattenToPatternHelper(flattened, composite_instruction, pattern, process_child_composites);
  return flattened;
}

void generateSkeletonSeedHelper(CompositeInstruction& composite_instructions)
{
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateSkeletonSeedHelper(*(i.cast<CompositeInstruction>()));
    }
    else if (isPlanInstruction(i))
    {
      CompositeInstruction ci;
      const auto* pi = i.cast<PlanInstruction>();
      ci.setProfile(pi->getProfile());
      ci.setDescription(pi->getDescription());
      ci.setManipulatorInfo(pi->getManipulatorInfo());

      i = ci;
    }
  }
}

CompositeInstruction generateSkeletonSeed(const CompositeInstruction& composite_instructions)
{
  CompositeInstruction seed = composite_instructions;
  generateSkeletonSeedHelper(seed);
  return seed;
}

}  // namespace tesseract_planning
