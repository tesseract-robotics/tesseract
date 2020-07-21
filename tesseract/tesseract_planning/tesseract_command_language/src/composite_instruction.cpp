/**
 * @file composite_instruction.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(std::string profile, CompositeInstructionOrder order)
  : profile_(std::move(profile)), order_(order)
{
}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

int CompositeInstruction::getType() const { return type_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

void CompositeInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? "DEFAULT" : profile;
}
const std::string& CompositeInstruction::getProfile() const { return profile_; }

void CompositeInstruction::setStartInstruction(Instruction instruction) { start_instruction_ = instruction; }

void CompositeInstruction::resetStartInstruction() { start_instruction_ = NullInstruction(); }

const Instruction& CompositeInstruction::getStartInstruction() const { return start_instruction_; }

Instruction& CompositeInstruction::getStartInstruction() { return start_instruction_; }

bool CompositeInstruction::hasStartInstruction() const { return (!isNullInstruction(start_instruction_)); }

void CompositeInstruction::print(std::string prefix) const
{
  std::cout << prefix + "Composite Instruction, Description: " << getDescription() << std::endl;
  std::cout << prefix + "--- Start Instruction, Description: " << start_instruction_.getDescription() << std::endl;
  std::cout << prefix + "{" << std::endl;
  for (const auto& i : *this)
    i.print(prefix + "  ");
  std::cout << prefix + "}" << std::endl;
}

}  // namespace tesseract_planning
