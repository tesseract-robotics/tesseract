
#include <algorithm>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction.getType()))
      {
        const PlanInstruction* result = getFirstPlanInstruction(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(instruction.getType()))
      {
        return instruction.cast_const<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if(isPlanInstruction(it->getType()))
      return it->cast_const<PlanInstruction>();

  return nullptr;
}

const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(it->getType()))
      {
        const PlanInstruction* result = getLastPlanInstruction(*(it->cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(it->getType()))
      {
        return it->cast_const<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if(isPlanInstruction(it->getType()))
      return it->cast_const<PlanInstruction>();

  return nullptr;
}

PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction.getType()))
      {
        PlanInstruction* result = getFirstPlanInstruction(*(instruction.cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(instruction.getType()))
      {
        return instruction.cast<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if(isPlanInstruction(it->getType()))
      return it->cast<PlanInstruction>();

  return nullptr;
}

PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(it->getType()))
      {
        PlanInstruction* result = getLastPlanInstruction(*(it->cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isPlanInstruction(it->getType()))
      {
        return it->cast<PlanInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if(isPlanInstruction(it->getType()))
      return it->cast<PlanInstruction>();

  return nullptr;
}

const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction.getType()))
      {
        const MoveInstruction* result = getFirstMoveInstruction(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(instruction.getType()))
      {
        return instruction.cast_const<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if(isMoveInstruction(it->getType()))
      return it->cast_const<MoveInstruction>();

  return nullptr;
}

const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(it->getType()))
      {
        const MoveInstruction* result = getLastMoveInstruction(*(it->cast_const<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(it->getType()))
      {
        return it->cast_const<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if(isMoveInstruction(it->getType()))
      return it->cast_const<MoveInstruction>();

  return nullptr;
}

MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction.getType()))
      {
        MoveInstruction* result = getFirstMoveInstruction(*(instruction.cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(instruction.getType()))
      {
        return instruction.cast<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.begin(); it != composite_instruction.end(); ++it)
    if(isMoveInstruction(it->getType()))
      return it->cast<MoveInstruction>();

  return nullptr;
}

MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    {
      if (isCompositeInstruction(it->getType()))
      {
        MoveInstruction* result = getLastMoveInstruction(*(it->cast<CompositeInstruction>()), process_child_composites);
        if (result)
          return result;
      }
      else if (isMoveInstruction(it->getType()))
      {
        return it->cast<MoveInstruction>();
      }
    }

    return nullptr;
  }

  for (auto it = composite_instruction.rbegin(); it != composite_instruction.rend(); ++it)
    if(isMoveInstruction(it->getType()))
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
      if (isCompositeInstruction(instruction.getType()))
        cnt+=getMoveInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isMoveInstruction(instruction.getType()))
        ++cnt;
    }
    return cnt;
  }

  return std::count_if(composite_instruction.begin(), composite_instruction.end(), [](const auto& i){return isMoveInstruction(i.getType());});
}

long getPlanInstructionsCount(const CompositeInstruction& composite_instruction, bool process_child_composites)
{
  if (process_child_composites)
  {
    long cnt = 0;
    for (const auto& instruction : composite_instruction)
    {
      if (isCompositeInstruction(instruction.getType()))
        cnt+=getPlanInstructionsCount(*(instruction.cast_const<CompositeInstruction>()), process_child_composites);
      else if (isPlanInstruction(instruction.getType()))
        ++cnt;
    }
    return cnt;
  }

  return std::count_if(composite_instruction.begin(), composite_instruction.end(), [](const auto& i){return isPlanInstruction(i.getType());});
}

}
