#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H

namespace tesseract_planning
{
class Instruction;

enum class InstructionType : int
{
  // Everything before must be a Null Instruction
  NULL_INSTRUCTION,

  // Everything before must be a motion plan Instruction
  PLAN_INSTRUCTION,

  // Everything before must be a motion Instruction
  MOVE_INSTRUCTION,

  // Everything before must be a composite Instruction
  COMPOSITE_INSTRUCTION,

  // Everything before must be a I/O Instruction
  IO_INSTRUCTION,

  // Everything before must be a Analog Instruction
  ANALOG_INSTRUCTION,

  // Everything before must be a variable Instruction
  VARIABLE_INSTRUCTION,

  // Everything before must be a comment Instruction
  COMMENT_INSTRUCTION,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

bool isCommentInstruction(const Instruction& instruction);

bool isVariableInstruction(const Instruction& instruction);

bool isAnalogInstruction(const Instruction& instruction);

bool isIOInstruction(const Instruction& instruction);

bool isCompositeInstruction(const Instruction& instruction);

bool isMoveInstruction(const Instruction& instruction);

bool isPlanInstruction(const Instruction& instruction);

bool isNullInstruction(const Instruction& instruction);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
