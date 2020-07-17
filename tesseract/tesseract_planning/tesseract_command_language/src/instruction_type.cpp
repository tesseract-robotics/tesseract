#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/core/instruction.h>

namespace tesseract_planning
{
bool isCommentInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::COMMENT_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::VARIABLE_INSTRUCTION));
}

bool isVariableInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::VARIABLE_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::ANALOG_INSTRUCTION));
}

bool isAnalogInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::ANALOG_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::IO_INSTRUCTION));
}

bool isIOInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::IO_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION));
}

bool isCompositeInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::MOVE_INSTRUCTION));
}

bool isMoveInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::MOVE_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::PLAN_INSTRUCTION));
}

bool isPlanInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::PLAN_INSTRUCTION) &&
          instruction.getType() > static_cast<int>(InstructionType::NULL_INSTRUCTION));
}

bool isNullInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::NULL_INSTRUCTION));
}

}  // namespace tesseract_planning
