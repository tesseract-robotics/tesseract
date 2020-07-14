
#include <algorithm>
#include <console_bridge/console.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites)
{
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

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isPlanInstruction(*it))
      return it->cast_const<PlanInstruction>();

  return nullptr;
}

PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
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

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isPlanInstruction(*it))
      return it->cast<PlanInstruction>();

  return nullptr;
}

const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction,
                                               bool process_child_composites)
{
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

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isMoveInstruction(*it))
      return it->cast_const<MoveInstruction>();

  return nullptr;
}

MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
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

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if (isMoveInstruction(*it))
      return it->cast<MoveInstruction>();

  return nullptr;
}

long getMoveInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    long cnt = 0;
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
        cnt += getMoveInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isMoveInstruction(instruction))
        ++cnt;
    }
    return cnt;
  }

  return std::count_if(
      composite_instruction.begin(), composite_instruction.end(), [](const auto& i) { return isMoveInstruction(i); });
}

long getPlanInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    long cnt = 0;
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction))
        cnt += getPlanInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isPlanInstruction(instruction))
        ++cnt;
    }
    return cnt;
  }

  return std::count_if(
      composite_instruction.begin(), composite_instruction.end(), [](const auto& i) { return isPlanInstruction(i); });
}

void flattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                   CompositeInstruction& composite,
                   const bool& include_composite)
{
  for (auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      if (include_composite)
        flattened.emplace_back(i);
      flattenHelper(flattened, *(i.cast<CompositeInstruction>()), include_composite);
    }
    else
      flattened.emplace_back(i);
  }
}

std::vector<std::reference_wrapper<Instruction>> flatten(CompositeInstruction& instruction,
                                                         const bool include_composite)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  flattenHelper(flattened, instruction, include_composite);
  return flattened;
}

void flattenHelper(std::vector<std::reference_wrapper<const Instruction>>& flattened,
                   const CompositeInstruction& composite,
                   const bool& include_composite)
{
  for (auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      if (include_composite)
        flattened.emplace_back(i);
      flattenHelper(flattened, *(i.cast_const<CompositeInstruction>()), include_composite);
    }
    else
      flattened.emplace_back(i);
  }
}

/**
 * @brief flattens a CompositeInstruction into a vector of Instruction&
 * @param instruction Input composite instruction to be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<const Instruction>> flatten(const CompositeInstruction& instruction,
                                                                      const bool include_composite)
{
  std::vector<std::reference_wrapper<const Instruction>> flattened;
  flattenHelper(flattened, instruction, include_composite);
  return flattened;
}

inline void flattenToPatternHelper(std::vector<std::reference_wrapper<Instruction>>& flattened,
                                   CompositeInstruction& composite,
                                   const CompositeInstruction& pattern,
                                   const bool& include_composite)
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
      if (include_composite)
        flattened.emplace_back(composite[i]);
      flattenToPatternHelper(flattened,
                             *(composite[i].cast<CompositeInstruction>()),
                             *pattern.at(i).cast_const<CompositeInstruction>(),
                             include_composite);
    }
    else
      flattened.emplace_back(composite[i]);
  }
}

std::vector<std::reference_wrapper<Instruction>> flattenToPattern(CompositeInstruction& instruction,
                                                                  const CompositeInstruction& pattern,
                                                                  const bool include_composite)
{
  if (instruction.size() != pattern.size())
  {
    CONSOLE_BRIDGE_logError("Instruction and pattern sizes are mismatched");
    return std::vector<std::reference_wrapper<Instruction>>();
  }

  std::vector<std::reference_wrapper<Instruction>> flattened;
  flattenToPatternHelper(flattened, instruction, pattern, include_composite);
  return flattened;
}

}  // namespace tesseract_planning
