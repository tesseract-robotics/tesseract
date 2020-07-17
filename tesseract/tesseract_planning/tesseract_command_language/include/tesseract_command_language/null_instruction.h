#ifndef TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H

#include <tesseract_command_language/instruction_type.h>
#include <iostream>

namespace tesseract_planning
{
class NullInstruction
{
public:
  int getType() const { return static_cast<int>(InstructionType::NULL_INSTRUCTION); }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(std::string prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Null Instruction, Type: " << getType() << "  Description: " << getDescription() << std::endl;
  }

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Null Instruction" };
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H
