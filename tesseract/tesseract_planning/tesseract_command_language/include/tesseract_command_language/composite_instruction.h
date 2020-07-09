#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/null_waypoint.h>
#include <vector>
#include <string>

namespace tesseract_planning
{
enum class CompositeInstructionOrder
{
  ORDERED,               // Must go in forward
  UNORDERED,             // Any order is allowed
  ORDERED_AND_REVERABLE  // Can go forward or reverse the order
};

class CompositeInstruction : public std::vector<Instruction>
{
public:
  using Ptr = std::shared_ptr<CompositeInstruction>;
  using ConstPtr = std::shared_ptr<const CompositeInstruction>;

  CompositeInstruction(std::string profile = "DEFAULT",
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED);

  CompositeInstructionOrder getOrder() const;

  int getType() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  // TODO: May need to create a class that includes not only the waypoint but I/O, environment, etc.
  void setStartWaypoint(Waypoint waypoint);
  const Waypoint& getStartWaypoint() const;
  bool hasStartWaypoint() const;

  /**
   * @brief This is a useful function for setting the last plan instructions waypoint
   * This loops over the vector in reverse to find the last plan instruction. It currently skips over child composite
   * instructions.
   */
  void setEndWaypoint(Waypoint waypoint, bool process_child_composites = false);

  bool isComposite() const;

  bool isPlan() const;

  bool isMove() const;

  void print(std::string prefix = "") const;

  CompositeInstruction flatten() const;

private:
  int type_{ static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) };

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Composite Instruction" };

  /**
   * @brief The profile applied its child plan instructions
   *
   * If it has a child composite instruction it uses the child composites profile for that section
   */
  std::string profile_{ "DEFAULT" };

  /** @brief The order of the composite instruction */
  CompositeInstructionOrder order_ { CompositeInstructionOrder::ORDERED };

  /**
   * @brief The start waypoint to use for composite instruction.
   *
   * If not provided, the planner should use the current state of the robot.
   */
  Waypoint start_waypoint_ { NullWaypoint() };

  void flattenHelper(CompositeInstruction& flattened, const CompositeInstruction& composite) const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
