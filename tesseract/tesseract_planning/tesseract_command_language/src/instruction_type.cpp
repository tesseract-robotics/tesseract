/**
 * @file instruction_type.cpp
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
