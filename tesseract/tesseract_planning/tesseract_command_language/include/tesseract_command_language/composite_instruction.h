/**
 * @file composite_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/manipulator_info.h>
#include <tesseract_command_language/null_instruction.h>

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
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED,
                       ManipulatorInfo manipulator_info = ManipulatorInfo());

  CompositeInstructionOrder getOrder() const;

  int getType() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  void setManipulatorInfo(ManipulatorInfo info);
  const ManipulatorInfo& getManipulatorInfo() const;
  ManipulatorInfo& getManipulatorInfo();

  void setStartInstruction(Instruction instruction);
  void resetStartInstruction();
  const Instruction& getStartInstruction() const;
  Instruction& getStartInstruction();
  bool hasStartInstruction() const;

  void print(std::string prefix = "") const;

private:
  int type_{ static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) };

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Composite Instruction" };

  /** @brief Contains information about the manipulator associated with this instruction*/
  ManipulatorInfo manipulator_info_;

  /**
   * @brief The profile applied its child plan instructions
   *
   * If it has a child composite instruction it uses the child composites profile for that section
   */
  std::string profile_{ "DEFAULT" };

  /** @brief The order of the composite instruction */
  CompositeInstructionOrder order_{ CompositeInstructionOrder::ORDERED };

  /**
   * @brief The start instruction to use for composite instruction. This should be of type PlanInstruction or
   * MoveInstruction but is stored as type Instruction because it is not required
   *
   * If not provided, the planner should use the current state of the robot is used and defined as fixed.
   */
  Instruction start_instruction_{ NullInstruction() };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
