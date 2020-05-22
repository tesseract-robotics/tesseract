#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H

namespace tesseract_planning
{

enum class InstructionType : int
{
  //Everything before must be a motion plan Instruction
  PLAN_INSTRUCTION,

  //Everything before must be a motion Instruction
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

inline bool isCommentInstruction(int type)
{
  return (type<= static_cast<int>(InstructionType::COMMENT_INSTRUCTION) && type > static_cast<int>(InstructionType::VARIABLE_INSTRUCTION));
}

inline bool isVariableInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::VARIABLE_INSTRUCTION) && type > static_cast<int>(InstructionType::ANALOG_INSTRUCTION));
}

inline bool isAnalogInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::ANALOG_INSTRUCTION) && type > static_cast<int>(InstructionType::IO_INSTRUCTION));
}

inline bool isIOInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::IO_INSTRUCTION) && type > static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION));
}

inline bool isCompositeInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) && type > static_cast<int>(InstructionType::MOVE_INSTRUCTION));
}

inline bool isMoveInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::MOVE_INSTRUCTION) && type > static_cast<int>(InstructionType::PLAN_INSTRUCTION));
}

inline bool isPlanInstruction(int type)
{
  return (type <= static_cast<int>(InstructionType::PLAN_INSTRUCTION));
}

}

#endif // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
