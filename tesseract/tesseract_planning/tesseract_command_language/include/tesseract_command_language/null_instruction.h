/**
 * @file null_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/visibility_control.h>

namespace tesseract_planning
{
class TESSERACT_COMMAND_LANGUAGE_PUBLIC NullInstruction
{
public:
  int getType() const { return static_cast<int>(InstructionType::NULL_INSTRUCTION); }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(std::string prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Null Instruction, Type: " << getType() << "  Description: " << getDescription() << std::endl;
  }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
    xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_null_instruction = doc.NewElement("NullInstruction");
    tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
    xml_description->SetText(getDescription().c_str());

    xml_null_instruction->InsertEndChild(xml_description);
    xml_instruction->InsertEndChild(xml_null_instruction);

    return xml_instruction;
  }

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Null Instruction" };
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_INSTRUCTION_H
