/**
 * @file move_instruction.cpp
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
#include <iostream>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint,
                                 MoveInstructionType type,
                                 const std::string& profile,
                                 ManipulatorInfo manipulator_info)
  : move_type_(type), profile_(profile), waypoint_(std::move(waypoint)), manipulator_info_(manipulator_info)
{
}

void MoveInstruction::setWaypoint(Waypoint waypoint)
{
  if (!isStateWaypoint(waypoint))
    CONSOLE_BRIDGE_logWarn("MoveInstruction usually expects to be provided a State Waypoint!");

  waypoint_ = waypoint;
}

Waypoint& MoveInstruction::getWaypoint() { return waypoint_; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

void MoveInstruction::setManipulatorInfo(ManipulatorInfo info) { manipulator_info_ = info; }
const ManipulatorInfo& MoveInstruction::getManipulatorInfo() const { return manipulator_info_; }
ManipulatorInfo& MoveInstruction::getManipulatorInfo() { return manipulator_info_; }

void MoveInstruction::setProfile(const std::string& profile) { profile_ = (profile.empty()) ? "DEFAULT" : profile; }
const std::string& MoveInstruction::getProfile() const { return profile_; }

int MoveInstruction::getType() const { return type_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

void MoveInstruction::print(std::string prefix) const
{
  std::cout << prefix + "Move Instruction, Type: " << getType() << ", Plan Type: " << static_cast<int>(move_type_)
            << ", ";
  getWaypoint().print();
  std::cout << ", Description: " << getDescription() << std::endl;
}

void MoveInstruction::setMoveType(MoveInstructionType move_type) { move_type_ = move_type; }

MoveInstructionType MoveInstruction::getMoveType() const { return move_type_; }

bool MoveInstruction::isLinear() const { return (move_type_ == MoveInstructionType::LINEAR); }

bool MoveInstruction::isFreespace() const { return (move_type_ == MoveInstructionType::FREESPACE); }

bool MoveInstruction::isCircular() const { return (move_type_ == MoveInstructionType::CIRCULAR); }

bool MoveInstruction::isStart() const { return (move_type_ == MoveInstructionType::START); }

tinyxml2::XMLElement* MoveInstruction::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
  xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

  tinyxml2::XMLElement* xml_move_instruction = doc.NewElement("MoveInstruction");
  xml_move_instruction->SetAttribute("type", std::to_string(static_cast<int>(getMoveType())).c_str());

  tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
  xml_description->SetText(getDescription().c_str());
  xml_move_instruction->InsertEndChild(xml_description);

  tinyxml2::XMLElement* xml_profile = doc.NewElement("Profile");
  xml_profile->SetText(getProfile().c_str());
  xml_move_instruction->InsertEndChild(xml_profile);

  if (!getManipulatorInfo().empty())
  {
    tinyxml2::XMLElement* xml_manip_info = getManipulatorInfo().toXML(doc);
    xml_move_instruction->InsertEndChild(xml_manip_info);
  }

  tinyxml2::XMLElement* xml_waypoint = getWaypoint().toXML(doc);
  xml_move_instruction->InsertEndChild(xml_waypoint);

  xml_instruction->InsertEndChild(xml_move_instruction);

  return xml_instruction;
}

}  // namespace tesseract_planning
