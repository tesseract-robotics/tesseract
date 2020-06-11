#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <vector>
#include <string>

namespace tesseract_planning
{

enum class CompositeInstructionOrder
{
  ORDERED, // Must go in forward
  UNORDERED, // Any order is allowed
  ORDERED_AND_REVERABLE // Can go forward or reverse the order
};

class CompositeInstruction : public std::vector<Instruction>
{
public:
  using Ptr = std::shared_ptr<CompositeInstruction>;
  using ConstPtr = std::shared_ptr<const CompositeInstruction>;

  CompositeInstruction(std::string profile = "DEFAULT", CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED);

  CompositeInstructionOrder getOrder() const;

  int getType() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  bool isComposite() const;

  bool isPlan() const;

  bool isMove() const;

  void print() const;

  CompositeInstruction flatten() const;

private:
  int type_ { static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) };

  /** @brief The description of the instruction */
  std::string description_ {"Tesseract Composite Instruction"};

  /**
   * @brief The profile applied its child plan instructions
   *
   * If it has a child composite instruction it uses the child composites profile for that section
   */
  std::string profile_ {"DEFAULT"};

  CompositeInstructionOrder order_;

  void flattenHelper(CompositeInstruction& flattened, const CompositeInstruction& composite) const;

};

}

#endif // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
